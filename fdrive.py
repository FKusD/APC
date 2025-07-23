##!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Основная программа управления беспилотным автомобилем.
Осуществляет движение по коридору на основе данных с датчиков VL53L5CX.
"""

import mmap
import os
import time
import struct
import signal
import sys
from typing import Dict, Optional, List, Tuple

# Подавление предупреждений от RPi.GPIO, если он недоступен (например, при разработке на ПК)
# try:
import RPi.GPIO as GPIO
import posix_ipc

ON_RASPBERRY = True
# except (ImportError, ModuleNotFoundError):
#     print("WARNING: RPi.GPIO or posix_ipc not found. Running in simulation mode.")
#     ON_RASPBERRY = False


# --- КОНСТАНТЫ И НАСТРОЙКИ ---

# Пины GPIO
SERVO_PIN = 18
MOTOR_PIN = 6

# Параметры ШИМ (PWM)
PWM_FREQUENCY = 7000  # Гц, стандарт для сервоприводов
PWM_FREQUENCY_SERVO = 50
MOTOR_START_DC = 0  # Начальный газ для старта (в процентах)
MOTOR_DRIVE_DC = 0  # Рабочий газ (в процентах)
# MOTOR_START_DC = 35 # Начальный газ для старта (в процентах)
# MOTOR_DRIVE_DC = 20 # Рабочий газ (в процентах)
MOTOR_STOP_DC = 0  # Газ при остановке

# Параметры сервопривода
SERVO_MIN_ANGLE = 80  # Минимальный угол поворота
SERVO_MAX_ANGLE = 135  # Максимальный угол поворота
SERVO_CENTER_ANGLE = 105  # Центральное положение

# Имена сегментов разделяемой памяти
LEFT_SENSOR_SHM = "vl53l5cx_left"
RIGHT_SENSOR_SHM = "vl53l5cx_right"

# Коэффициенты для 8 ПИД-регуляторов (Kp, Ki, Kd)
# Настроены так, чтобы датчики, смотрящие ближе к центру, имели большее влияние.
# Индекс 0 - самая внешняя пара лучей, индекс 7 - самая внутренняя.
PID_COEFFS: List[Tuple[float, float, float]] = [
    # (Kp, Ki, Kd) для ошибок E0 (L0-R7) ... E7 (L7-R0)
    (0.010, 0.00, 0.0),  # E0: внешние лучи
    (0.012, 0.00, 0.0),  # E1
    (0.015, 0.00, 0.0),  # E2
    (0.080, 0.00, 0.0),  # E3
    (0.110, 0.00, 0.0),  # E4
    (0.140, 0.00, 0.0),  # E5
    (0.185, 0.00, 0.0),  # E6
    (0.100, 0.00, 0.0),  # E7: внутренние лучи (наибольшее влияние)
]

# Коэффициент масштабирования для рулевого управления
# Преобразует суммарную ошибку от ПИДов в градусы поворота сервы
STEERING_SCALING_FACTOR = 0.05

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

        if dt <= 1e-9:  # Избегаем деления на ноль
            return 0.0

        # Пропорциональная часть
        P = self.Kp * error

        # Интегральная часть
        self.integral += error * dt
        I = self.Ki * self.integral

        # Дифференциальная часть
        derivative = (error - self.prev_error) / dt
        D = self.Kd * derivative

        # Обновление состояния для следующего шага
        self.prev_error = error
        self.last_time = current_time

        # Возвращаем суммарное управляющее воздействие
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

        if self.data_format != 0:  # Матричное измерение
            matrix_size = self.resolution
            distances_end = 8 + matrix_size * 2
            statuses_end = distances_end + matrix_size

            distances = struct.unpack(f"<{matrix_size}H", data[8:distances_end])
            statuses = struct.unpack(
                f"<{matrix_size}B", data[distances_end:statuses_end]
            )

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

            resolution = struct.unpack("<IBBBB", header_data)[2]
            data_size = 8 + resolution * 3

            mmap_obj.seek(0)
            data = mmap_obj.read(data_size)

            if len(data) == data_size:
                return SensorData(data)
            return None
        except Exception:
            return None
        finally:
            if "sem" in locals() and isinstance(sem, posix_ipc.Semaphore):
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
        print("GPIO and PWM initialized.")

    def _set_servo_angle(self, angle: float):
        """Устанавливает угол поворота сервопривода."""
        duty = angle / 18 + 2
        self.pwm_servo.ChangeDutyCycle(duty)

    def _set_motor_speed(self, duty_cycle: float):
        """Устанавливает скорость мотора."""
        self.pwm_motor.ChangeDutyCycle(duty_cycle)

    def signal_handler(self, signum, frame):
        """Обработчик сигналов для корректного завершения."""
        print(f"\nSignal {signum} received, stopping...")
        self.running = False

    def cleanup(self):
        """Очистка ресурсов."""
        print("Cleaning up resources...")
        if ON_RASPBERRY:
            self._set_motor_speed(MOTOR_STOP_DC)
            self._set_servo_angle(SERVO_CENTER_ANGLE)
            time.sleep(0.5)
            self.pwm_servo.stop()
            self.pwm_motor.stop()
            GPIO.cleanup()
        self.sensor_reader.cleanup()
        print("Cleanup complete. Exiting.")

    def run(self):
        """Основной цикл управления."""
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

        print("Starting car controller...")
        if ON_RASPBERRY:
            self._set_servo_angle(SERVO_CENTER_ANGLE)
            print("Servo centered. Starting motor sequence...")
            self._set_motor_speed(MOTOR_START_DC)
            time.sleep(0.1)
            self._set_motor_speed(MOTOR_DRIVE_DC)
            print(f"Motor running at {MOTOR_DRIVE_DC}% duty cycle.")

        print("Delay 1.5s")
        time.sleep(1.5)
        print("Corridor following started. Press Ctrl+C to stop.")

        # --- Режим отладки: 0.3 сек едет (0.08 старт + 0.22 рабочая), 0.7 сек стоит ---
        debug_state = "start"  # 'start', 'drive', 'stop'
        debug_state_time = time.time()

        while self.running:
            # Чтение данных с датчиков
            left_data = self.sensor_reader.read_sensor_data(LEFT_SENSOR_SHM)
            right_data = self.sensor_reader.read_sensor_data(RIGHT_SENSOR_SHM)

            if (
                not left_data
                or not right_data
                or not left_data.distances
                or not right_data.distances
            ):
                print("Waiting for sensor data...", end="\r")
                time.sleep(0.05)
                continue

            # Извлекаем 4-ю строку (индекс 3) из матрицы 8x8
            # В одномерном списке это будет срез с 24 по 31 индекс
            left_row = left_data.distances[24:32]
            right_row = right_data.distances[24:32]

            if len(left_row) != 8 or len(right_row) != 8:
                print("Invalid data row length, skipping frame.", end="\r")
                time.sleep(0.05)
                continue

            # --- ВЫВОД ЗНАЧЕНИЙ 4-Й СТРОКИ ---
            print(f"LEFT  4th row: {[f'{v:4d}' for v in left_row]}")
            print(f"RIGHT 4th row: {[f'{v:4d}' for v in right_row]}")

            total_correction = 0.0
            pid_debug_info = []
            for i in range(8):
                # Ошибка = расстояние слева - расстояние справа (для симметричных лучей)
                # Если расстояние 0, считаем его максимальным, чтобы избежать ложных срабатываний
                dist_left = left_row[i] if left_row[i] > 0 else 4000
                dist_right = right_row[7 - i] if right_row[7 - i] > 0 else 4000

                error = dist_left - dist_right

                # Рассчитываем коррекцию от ПИД-регулятора
                correction = self.pids[i].compute(setpoint=0, current_value=error)
                total_correction += correction
                pid_debug_info.append((i, error, correction))

            # Подробный вывод ошибок и выходов ПИД-регуляторов
            print("PID details:")
            for idx, err, corr in pid_debug_info:
                print(f"  PID[{idx}]: error={err:5d}, output={corr:8.3f}")

            # Масштабируем и применяем коррекцию к углу сервопривода
            steer_adjustment = total_correction * STEERING_SCALING_FACTOR
            target_angle = (
                SERVO_CENTER_ANGLE + steer_adjustment
            )  # ИНВЕРСИЯ: теперь +, чтобы инвертировать направление

            # Ограничиваем угол в заданных пределах
            clamped_angle = max(SERVO_MIN_ANGLE, min(SERVO_MAX_ANGLE, target_angle))

            if ON_RASPBERRY:
                self._set_servo_angle(clamped_angle)

            # --- Режим отладки: 0.3 сек едет (0.08 старт + 0.22 рабочая), 0.7 сек стоит ---
            now = time.time()
            elapsed = now - debug_state_time
            if debug_state == "start":
                if elapsed >= 0.1:
                    debug_state = "drive"
                    debug_state_time = now
                    if ON_RASPBERRY:
                        self._set_motor_speed(MOTOR_DRIVE_DC)
                        print("[DEBUG] MOTOR DRIVE")
                else:
                    if ON_RASPBERRY:
                        self._set_motor_speed(MOTOR_START_DC)
                        print("[DEBUG] MOTOR START")
            elif debug_state == "drive":
                if elapsed >= 0.22:
                    debug_state = "stop"
                    debug_state_time = now
                    if ON_RASPBERRY:
                        self._set_motor_speed(MOTOR_STOP_DC)
                        print("[DEBUG] MOTOR STOP")
            elif debug_state == "stop":
                if elapsed >= 0.7:
                    debug_state = "start"
                    debug_state_time = now
                    if ON_RASPBERRY:
                        self._set_motor_speed(MOTOR_START_DC)
                        print("[DEBUG] MOTOR START")

            print(
                f"L: {left_row[7]} R: {right_row[0]} | Err: {error:.0f} | Adj: {steer_adjustment:.2f} | Angle: {clamped_angle:.1f}"
            )
            time.sleep(0.02)  # Цикл ~50 Гц


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
