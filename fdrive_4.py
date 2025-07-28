#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Основная программа управления беспилотным автомобилем.
Осуществляет движение по коридору на основе данных с датчиков VL53L5CX.
Добавлено сглаживание данных датчиков.
"""

import mmap
import os
import time
import struct
import signal
import sys
import numpy as np
from typing import Dict, Optional, List, Tuple

# Подавление предупреждений от RPi.GPIO, если он недоступен (например, при разработке на ПК)
try:
    import RPi.GPIO as GPIO
    import posix_ipc
    ON_RASPBERRY = True
except (ImportError, ModuleNotFoundError):
    print("WARNING: RPi.GPIO or posix_ipc not found. Running in simulation mode.")
    ON_RASPBERRY = False

# --- КОНСТАНТЫ И НАСТРОЙКИ ---

# Пины GPIO
SERVO_PIN = 18
MOTOR_PIN = 6

# Параметры ШИМ (PWM)
PWM_FREQUENCY = 5000  # Гц, стандарт для сервоприводов
PWM_FREQUENCY_SERVO = 50
MOTOR_START_DC = 60  # Начальный газ для старта (в процентах)
MOTOR_DRIVE_DC = 10 # Рабочий газ (в процентах)
MOTOR_STOP_DC = 0    # Газ при остановке

# Параметры сервопривода
SERVO_MIN_ANGLE = 70  # Минимальный угол поворота
SERVO_MAX_ANGLE = 145  # Максимальный угол поворота
SERVO_CENTER_ANGLE = 105  # Центральное положение

# Имена сегментов разделяемой памяти
LEFT_SENSOR_SHM = "vl53l5cx_left"
RIGHT_SENSOR_SHM = "vl53l5cx_right"

# Коэффициенты для 4 ПИД-регуляторов (Kp, Ki, Kd)
PID_COEFFS: List[Tuple[float, float, float]] = [
    (0.008, 0.002, 0.001),  # E0: внешние лучи
    (0.011, 0.002, 0.001),  # E1 0.06 0 0.01
    (0.008, 0.002, 0.002),  # E2 0.1 0.003 0.015
    (0.003, 0.002, 0.001),  # E3: внутренние лучи 0.08
]

# Коэффициент масштабирования для рулевого управления
STEERING_SCALING_FACTOR = 0.22
# --- КЛАСС ПИД-РЕГУЛЯТОРА ---

class PIDController:
    """Простой ПИД-регулятор."""

    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.reset()

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, setpoint: float, current_value: float) -> float:
        """Рассчитывает управляющее воздействие."""
        error = setpoint - current_value
        current_time = time.time()
        dt = current_time - self.last_time

        if dt <= 1e-9:
            return 0.0

        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative

        self.prev_error = error
        self.last_time = current_time

        return P + I + D

# --- КЛАССЫ ДЛЯ ЧТЕНИЯ ДАННЫХ ИЗ SHARED MEMORY ---

class SensorData:
    """Структура для хранения данных с датчика."""

    def __init__(self, data: bytes):
        header = struct.unpack("<IBBBB", data[:8])
        self.timestamp = header[0]
        self.sensor_type = header[1]
        self.resolution = header[2]
        self.data_format = header[3]

        if self.data_format != 0:
            matrix_size = self.resolution
            distances = struct.unpack(f"<{matrix_size}H", data[8:8+matrix_size*2])
            statuses = struct.unpack(f"<{matrix_size}B", data[8+64*2:8+64*2+matrix_size])
            self.distances = list(distances)
            self.statuses = list(statuses)
        else:
            self.distances = []
            self.statuses = []

class SensorReader:
    """Класс для чтения данных датчиков из shared memory."""

    def __init__(self):
        self.shm_handles: Dict[str, tuple] = {}
        if not ON_RASPBERRY:
            return

    def open_shared_memory(self, shm_name: str) -> Optional[tuple]:
        try:
            fd = os.open(f"/dev/shm/{shm_name}", os.O_RDONLY)
            stat = os.fstat(fd)
            size = stat.st_size
            mmap_obj = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ)
            sem = posix_ipc.Semaphore(f"/sem_{shm_name}")
            return (fd, mmap_obj, sem)
        except Exception:
            return None

    def read_sensor_data(self, shm_name: str) -> Optional[SensorData]:
        if not ON_RASPBERRY:
            return None

        if shm_name not in self.shm_handles:
            handle = self.open_shared_memory(shm_name)
            if handle is None:
                return None
            self.shm_handles[shm_name] = handle

        _, mmap_obj, sem = self.shm_handles[shm_name]

        try:
            sem.acquire(timeout=0.1)
            mmap_obj.seek(0)
            header_data = mmap_obj.read(8)
            if len(header_data) < 8:
                return None

            header = struct.unpack("<IBBBB", header_data)
            resolution = header[2]
            data_format = header[3]

            if data_format == 0:
                data_size = 16
            else:
                data_size = 8 + 64 * 2 + 64
                if resolution == 0:
                    return None

            mmap_obj.seek(0)
            data = mmap_obj.read(data_size)

            if len(data) == data_size:
                return SensorData(data)
            return None
        except Exception:
            return None
        finally:
            sem.release()

    def cleanup(self):
        for shm_name in list(self.shm_handles.keys()):
            fd, mmap_obj, sem = self.shm_handles[shm_name]
            mmap_obj.close()
            os.close(fd)
            sem.close()
            del self.shm_handles[shm_name]

# --- ОСНОВНОЙ КЛАСС УПРАВЛЕНИЯ ---

class CarController:
    """Класс, инкапсулирующий логику управления автомобилем."""

    def __init__(self):
        self.running = True
        self.sensor_reader = SensorReader()
        self.pids = [PIDController(*coeffs) for coeffs in PID_COEFFS]
        self.start_time = None
        self.end_time = None
        # Буферы для сглаживания данных
        self.buffer_size = 3
        self.left_buffers = [[] for _ in range(4)]
        self.right_buffers = [[] for _ in range(4)]

        if ON_RASPBERRY:
            self._setup_gpio()

    def _setup_gpio(self):
        """Настройка GPIO пинов и ШИМ."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        GPIO.setup(MOTOR_PIN, GPIO.OUT)

        self.pwm_servo = GPIO.PWM(SERVO_PIN, PWM_FREQUENCY_SERVO)
        self.pwm_motor = GPIO.PWM(MOTOR_PIN, PWM_FREQUENCY)

        self.pwm_servo.start(0)
        self.pwm_motor.start(0)

    def _set_servo_angle(self, angle: float):
        """Устанавливает угол поворота сервопривода."""
        duty = angle / 18 + 2
        self.pwm_servo.ChangeDutyCycle(duty)

    def _set_motor_speed(self, duty_cycle: float):
        """Устанавливает скорость мотора."""
        self.pwm_motor.ChangeDutyCycle(duty_cycle)

    def _smooth_sensor_data(self, values, statuses, buffers):
        """Сглаживание данных датчиков."""
        smoothed = []
        for i in range(4):
            if statuses[i] == 255:
                current_value = 5500
                add_to_buffer = True
            else:
                current_value = values[i]
                add_to_buffer = True
            
            if add_to_buffer:
                buffers[i].append(current_value)
                if len(buffers[i]) > self.buffer_size:
                    buffers[i].pop(0)
            
            if buffers[i]:
                smoothed_value = np.median(buffers[i])
            else:
                smoothed_value = current_value
            
            smoothed.append(smoothed_value)
        
        return smoothed

    def signal_handler(self, signum, frame):
        """Обработчик сигналов для корректного завершения."""
        print(f"\nSignal {signum} received, stopping...")
        self.end_time = time.time()
        print("time: ",self.end_time - self.start_time)
        self.running = False

    def cleanup(self):
        """Очистка ресурсов."""
        print("Cleaning up resources...")
        if ON_RASPBERRY:
            self._set_motor_speed(MOTOR_STOP_DC)
            self._set_servo_angle(SERVO_CENTER_ANGLE)
            time.sleep(0.25)
            self.pwm_servo.stop()
            self.pwm_motor.stop()
            GPIO.cleanup()
        self.sensor_reader.cleanup()
        print("Cleanup complete. Exiting.")

    def run(self):
        """Основной цикл управления."""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        self.start_time = time.time()
        print("Starting car controller...")
        if ON_RASPBERRY:
            self._set_servo_angle(SERVO_CENTER_ANGLE)
            print("Servo centered. Starting motor sequence...")
            self._set_motor_speed(MOTOR_START_DC)
            time.sleep(0.5)
            self._set_motor_speed(MOTOR_DRIVE_DC)
            print(f"Motor running at {MOTOR_DRIVE_DC}% duty cycle.")

        print("Corridor following started. Press Ctrl+C to stop.")

        while self.running:
            # Чтение данных с датчиков
            left_data = self.sensor_reader.read_sensor_data(LEFT_SENSOR_SHM)
            right_data = self.sensor_reader.read_sensor_data(RIGHT_SENSOR_SHM)

            if (not left_data or not right_data or 
                not left_data.distances or not right_data.distances):
                print("Waiting for sensor data...", end="\r")
                time.sleep(0.05)
                continue

            # Извлекаем 2-ю строку (индекс 1) из матрицы 4x4
            left_row_orig = left_data.distances[4:8]
            right_row_orig = right_data.distances[4:8]
            left_status_row = left_data.statuses[4:8]
            right_status_row = right_data.statuses[4:8]

            # Сглаживание данных
            left_row = self._smooth_sensor_data(
                left_row_orig, left_status_row, self.left_buffers
            )
            right_row = self._smooth_sensor_data(
                right_row_orig, right_status_row, self.right_buffers
            )

            print(f"LEFT orig: {left_row_orig} smoothed: {left_row}")
            print(f"RIGHT orig: {right_row_orig} smoothed: {right_row}")

            total_correction = 0.0
            for i in range(4):
                dist_left = left_row[i]
                dist_right = right_row[3 - i]
                error = dist_left - dist_right
                correction = self.pids[i].compute(setpoint=0, current_value=error)
                total_correction += correction

            steer_adjustment = total_correction * STEERING_SCALING_FACTOR
            target_angle = SERVO_CENTER_ANGLE + steer_adjustment
            clamped_angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, target_angle))

            angle_deviation = abs(clamped_angle - SERVO_CENTER_ANGLE)
            if angle_deviation <= 8:
                motor_speed = MOTOR_DRIVE_DC
            else:
                max_deviation = max(
                    abs(SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE),
                    abs(SERVO_MIN_ANGLE - SERVO_CENTER_ANGLE),
                )
                min_speed = MOTOR_DRIVE_DC / 1.5
                scale = min(1.0, (angle_deviation - 8) / (max_deviation - 8))
                motor_speed = MOTOR_DRIVE_DC - (MOTOR_DRIVE_DC - min_speed) * scale

            if ON_RASPBERRY:
                self._set_servo_angle(clamped_angle)
                self._set_motor_speed(motor_speed)

            print(f"Angle: {clamped_angle:.1f}° | Speed: {motor_speed:.1f}%")

            time.sleep(0.02)

def main():
    """Главная функция."""
    controller = CarController()
    try:
        controller.run()
    except KeyboardInterrupt:

        print("\nKeyboardInterrupt caught.")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main()

