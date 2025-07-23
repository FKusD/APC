#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import test_3pin_servo_2 as servo
from sensor_processor import create_processor, calculate_angles

# Настройка пинов для мотора
MOTOR_LPWM = 12  # GPIO 12 (PWM0)
MOTOR_RPWM = 6  # GPIO 6 (PWM1)
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LPWM, GPIO.OUT)
GPIO.setup(MOTOR_RPWM, GPIO.OUT)

# Настройка ШИМ для мотора (высокая частота для плавности)
motor_lpwm = GPIO.PWM(MOTOR_LPWM, 5000)
motor_rpwm = GPIO.PWM(MOTOR_RPWM, 5000)
motor_lpwm.start(0)
motor_rpwm.start(0)

# Настройки сервопривода
zero_angle = 105
servo.set_angle(zero_angle)
current_angle = zero_angle
pid = servo.PIDController(Kp=0.2, Ki=0.005, Kd=0.002)  # Мягкие коэффициенты

# Инициализация обработчика датчиков
processor = create_processor(["vl53l5cx_left", "vl53l5cx_right"])
sensor_angles = {"vl53l5cx_left": -45, "vl53l5cx_right": 45}

# Константы управления
MOTOR_START_DC = 15  # Начальный газ для старта (в процентах)
MOTOR_DRIVE_DC = 1  # Рабочий газ (в процентах)
# MOTOR_START_DC = 35 # Начальный газ для старта (в процентах)
# MOTOR_DRIVE_DC = 20 # Рабочий газ (в процентах)
MOTOR_STOP_DC = 0  # Г
ANGLE_COEFFICIENT = 0.9  # Коэффициент реакции на угол
BASE_SPEED = 0  # Рабочая скорость (4%)
START_SPEED = 10  # Стартовая скорость (35%)
SMOOTHING_FACTOR = 0.2  # Фактор сглаживания угла


def set_motor_speed(speed):
    """Управление мотором с ограничением скорости"""
    speed = max(-100, min(100, speed))

    if speed > 0:
        motor_lpwm.ChangeDutyCycle(0)
        motor_rpwm.ChangeDutyCycle(speed)
    elif speed < 0:
        motor_lpwm.ChangeDutyCycle(abs(speed))
        motor_rpwm.ChangeDutyCycle(0)
    else:
        motor_lpwm.ChangeDutyCycle(0)
        motor_rpwm.ChangeDutyCycle(0)


def original_smooth_start():
    """Оригинальный плавный старт: 35% -> 20% за 0.5 сек"""
    set_motor_speed(START_SPEED)
    start_time = time.time()
    if START_SPEED == 0:
        return
    while time.time() - start_time < 0.5:
        elapsed = time.time() - start_time
        current_speed = START_SPEED - (15 * (elapsed / 0.5))
        set_motor_speed(current_speed)
        time.sleep(0.01)

    set_motor_speed(BASE_SPEED)  # Финишная скорость


def smooth_steering(target_angle):
    """Плавное изменение угла руления"""
    global current_angle
    current_angle = pid.compute(target_angle, current_angle)
    servo.set_angle(current_angle)


try:
    # Оригинальный плавный старт
    original_smooth_start()
    debug_state = "start"  # 'start', 'drive', 'stop'
    debug_state_time = time.time()
    while True:
        # Получаем данные от датчиков
        angle_diff, distances = calculate_angles(processor, sensor_angles)

        # Применяем коэффициент к разнице углов
        adjusted_diff = angle_diff * ANGLE_COEFFICIENT

        # Расчет целевого угла с плавной коррекцией
        target_angle = zero_angle + adjusted_diff
        smooth_steering(target_angle)

        # Поддержание постоянной скорости
        # set_motor_speed(BASE_SPEED)

        # Отладочный вывод
        for name in processor.sensor_names:
            dist_str = [f"{d:.0f}" if d is not None else "---" for d in distances[name]]
            print(f"{name}: {dist_str}")

        print(f"Разница углов: {angle_diff:.1f}°")
        print(f"Текущий угол сервы: {current_angle:.1f}°")
        print("---")

        now = time.time()
        elapsed = now - debug_state_time
        if debug_state == "start":
            if elapsed >= 0.2:
                debug_state = "drive"
                debug_state_time = now
                set_motor_speed(MOTOR_DRIVE_DC)
                print("[DEBUG] MOTOR DRIVE")
            else:
                set_motor_speed(MOTOR_START_DC)
                print("[DEBUG] MOTOR START")
        elif debug_state == "drive":
            if elapsed >= 0.4:
                debug_state = "stop"
                debug_state_time = now
                set_motor_speed(MOTOR_STOP_DC)
                print("[DEBUG] MOTOR STOP")
        elif debug_state == "stop":
            if elapsed >= 0.5:
                debug_state = "start"
                debug_state_time = now
                set_motor_speed(MOTOR_START_DC)
                print("[DEBUG] MOTOR START")

        # time.sleep(0.01)

except KeyboardInterrupt:
    print("Остановка")
finally:
    set_motor_speed(0)
    motor_lpwm.stop()
    motor_rpwm.stop()
    GPIO.cleanup()
    print("Система остановлена")
