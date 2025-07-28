##!/usr/bin/env python3
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
import board
from adafruit_bitbangio import I2C
import adafruit_tcs34725

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
MOTOR_DRIVE_DC = 30  # Рабочий газ (в процентах)
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
    (0.002, 0.008, 0.001),  # E0: внешние лучи
    (0.003, 0.008, 0.001),  # E1 0.06 0 0.01
    (0.002, 0.008, 0.002),  # E2 0.1 0.003 0.015
    (0.001, 0.008, 0.002),  # E3: внутренние лучи 0.08
]

# Коэффициент масштабирования для рулевого управления
STEERING_SCALING_FACTOR = 0.15

# Параметры обнаружения линии
LUX_BUFFER_SIZE = 3  # Размер буфера для сглаживания lux
BANNER_JUMP_THRESHOLD = 1.8  # Во сколько раз должен увеличиться lux для обнаружения баннера
LINE_JUMP_THRESHOLD = 4.0  # Во сколько раз должен уменьшиться lux для обнаружения линии
STABILITY_ITERATIONS = 2  # Сколько итераций должно пройти для подтверждения скачка
LINE_COUNT_TIMEOUT = 0.3  # Лимит времени для обнаружения 1-2 линий (в секундах)
LINE_DETECTION_DELAY = 1.5  # Задержка обнаружения линий после старта (в секундах)

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

        # --- Инициализация датчика цвета ---
        self.i2c = I2C(board.D9, board.D10)
        self.color_sensor = adafruit_tcs34725.TCS34725(self.i2c)
        
        # Буферы для нового алгоритма обнаружения линии
        self.lux_buffer = []
        self.lux_buffer_size = LUX_BUFFER_SIZE
        self.baseline_lux = None  # Базовое значение lux (пол)
        self.banner_lux = None    # Значение lux на баннере
        self.line_detected = False
        self.banner_detected = False
        self.jump_counter = 0     # Счётчик итераций для подтверждения скачка
        self.last_lux = None      # Предыдущее значение lux
        self.current_motor_speed = MOTOR_DRIVE_DC
        
        # Переменные для подсчёта линий
        self.line_count = 0       # Количество обнаруженных линий
        self.line_start_time = None  # Время начала обнаружения линий
        self.last_line_time = None   # Время последней обнаруженной линии
        self.line_status = 'normal'  # normal, slow_for_stop, stop_on_line
        self.line_detection_enabled = False  # Флаг включения обнаружения линий
        # Удаляем старые переменные, которые больше не нужны
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

    def _reset_line_detection(self):
        """Сброс состояния обнаружения линий."""
        self.line_count = 0
        self.line_start_time = None
        self.last_line_time = None
        self.line_status = 'normal'
        self.line_detected = False
        self.banner_detected = False
        self.line_detection_enabled = False  # Отключаем обнаружение до следующего старта
        self.current_motor_speed = MOTOR_DRIVE_DC
        print("[TCS] Состояние обнаружения линий сброшено")

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

            # Обновляем текущую скорость мотора (если линия не обнаружена)
            if not self.line_detected:
                self.current_motor_speed = motor_speed

            self._set_servo_angle(clamped_angle)

            print(f"Angle: {clamped_angle:.1f}° | Speed: {self.current_motor_speed:.1f}%")

            # --- Чтение и фильтрация lux с tcs34725 ---
            try:
                lux = self.color_sensor.lux
            except Exception as e:
                lux = 0
            self.lux_buffer.append(lux)
            if len(self.lux_buffer) > self.lux_buffer_size:
                self.lux_buffer.pop(0)
            lux_filt = sum(self.lux_buffer) / len(self.lux_buffer)
            print(f"[TCS] lux={lux:.1f} filt={lux_filt:.1f}")

            # --- Калибровка базового значения ---
            if self.baseline_lux is None and lux_filt < 400:
                self.baseline_lux = lux_filt
                print(f"[TCS] Калибровка: базовое значение lux={lux_filt:.1f}")

            # --- Новый алгоритм обнаружения линии с подсчётом ---
            now = time.time()
            
            # Проверяем, прошло ли достаточно времени после старта
            if not self.line_detection_enabled:
                time_since_start = now - self.start_time
                if time_since_start >= LINE_DETECTION_DELAY:
                    self.line_detection_enabled = True
                    print(f"[TCS] Обнаружение линий включено (прошло {time_since_start:.1f}с)")
            
            # Проверяем, не уехали ли мы с баннера
            if self.banner_detected and self.baseline_lux is not None:
                lux_ratio_to_baseline = lux_filt / self.baseline_lux
                if lux_ratio_to_baseline < 1.5:  # Вернулись к базовому уровню
                    print(f"[TCS] Уехали с баннера, сбрасываем состояние. lux={lux_filt:.1f}")
                    self._reset_line_detection()
            
            if self.baseline_lux is not None and self.last_lux is not None and self.line_detection_enabled:
                # Проверяем скачок вверх (баннер)
                if not self.banner_detected:
                    lux_ratio = lux_filt / self.baseline_lux
                    if lux_ratio > BANNER_JUMP_THRESHOLD:
                        self.jump_counter += 1
                        if self.jump_counter >= STABILITY_ITERATIONS:
                            self.banner_detected = True
                            self.banner_lux = lux_filt
                            print(f"[TCS] Обнаружен баннер! lux={lux_filt:.1f} (в {lux_ratio:.1f} раз больше)")
                            self.jump_counter = 0
                    else:
                        self.jump_counter = 0
                
                # Проверяем скачок вниз (линия на баннере)
                elif self.banner_detected:
                    lux_ratio = self.banner_lux / lux_filt
                    if lux_ratio > LINE_JUMP_THRESHOLD:
                        self.jump_counter += 1
                        if self.jump_counter >= STABILITY_ITERATIONS:
                            # Обнаружена линия
                            self.line_count += 1
                            self.last_line_time = now
                            
                            # Инициализируем время начала, если это первая линия
                            if self.line_start_time is None:
                                self.line_start_time = now
                            
                            print(f"[TCS] Линия #{self.line_count} обнаружена! lux={lux_filt:.1f} (в {lux_ratio:.1f} раз меньше)")
                            
                            # Проверяем временной лимит для первых линий
                            if self.line_count <= 2:
                                time_since_start = now - self.line_start_time
                                if time_since_start > LINE_COUNT_TIMEOUT:
                                    print(f"[TCS] Превышен временной лимит ({LINE_COUNT_TIMEOUT}с) для {self.line_count} линий")
                                    self._reset_line_detection()
                                    self.jump_counter = 0
                                    continue
                            
                            # Изменяем статус в зависимости от количества линий
                            if self.line_count == 1:
                                self.line_status = 'normal'
                                print("[TCS] Первая линия - продолжаем движение")
                            elif self.line_count == 2:
                                self.line_status = 'slow_for_stop'
                                self.current_motor_speed = MOTOR_DRIVE_DC / 2  # Снижаем скорость
                                print("[TCS] Вторая линия - снижаем скорость")
                            elif self.line_count >= 3:
                                self.line_status = 'stop_on_line'
                                self.line_detected = True
                                print("[TCS] Третья линия - ОСТАНОВКА!")
                            
                            self.jump_counter = 0
                    else:
                        self.jump_counter = 0

            self.last_lux = lux_filt

            # Применяем скорость мотора в зависимости от статуса
            if self.line_detected:
                self._set_motor_speed(0)
            else:
                self._set_motor_speed(self.current_motor_speed)

            # Вывод информации о статусе
            if self.line_count > 0:
                print(f"[TCS] Статус: {self.line_status}, линий: {self.line_count}")
            
            # Сброс состояния при нажатии Ctrl+C (для тестирования)
            if self.line_detected:
                print("[TCS] Линия обнаружена! Автомобиль остановлен. Нажмите Ctrl+C для сброса.")

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
