#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys
import tty
import termios

# Настройка пинов для мотора
MOTOR_LPWM = 12  # GPIO 12 (PWM0)
MOTOR_RPWM = 6  # GPIO 6 (PWM1)
GPIO.setmode(GPIO.BCM)
GPIO.setup(MOTOR_LPWM, GPIO.OUT)
GPIO.setup(MOTOR_RPWM, GPIO.OUT)

# Настройка ШИМ для мотора
motor_lpwm = GPIO.PWM(MOTOR_LPWM, 1000)  # Частота 1 kHz
motor_rpwm = GPIO.PWM(MOTOR_RPWM, 1000)
motor_lpwm.start(0)
motor_rpwm.start(0)

# Настройки сервопривода
SERVO_PIN = 18
GPIO.setup(SERVO_PIN, GPIO.OUT)
servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz
servo_pwm.start(0)

# Текущие параметры
current_speed = 0
current_angle = 7.5  # Нейтральное положение сервы (90°)


def set_motor_speed(speed):
    """Управление мотором: speed от -100 до 100"""
    global current_speed
    speed = max(-100, min(100, speed))
    current_speed = speed

    if speed > 0:
        motor_lpwm.ChangeDutyCycle(0)
        motor_rpwm.ChangeDutyCycle(speed)
    elif speed < 0:
        motor_lpwm.ChangeDutyCycle(abs(speed))
        motor_rpwm.ChangeDutyCycle(0)
    else:
        motor_lpwm.ChangeDutyCycle(0)
        motor_rpwm.ChangeDutyCycle(0)


def set_servo_angle(angle):
    """Управление сервоприводом: angle от 0 до 180"""
    global current_angle
    angle = max(80, min(135, angle))
    current_angle = angle
    duty = angle / 18 + 2.5
    servo_pwm.ChangeDutyCycle(duty)


def getch():
    """Чтение одной клавиши без Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def print_status():
    """Вывод текущего состояния"""
    print(
        f"\rСкорость: {current_speed:3}% | Угол: {current_angle:3}°", end="", flush=True
    )


try:
    print("Управление WASD:")
    print("W - вперед")
    print("S - назад")
    print("A - влево")
    print("D - вправо")
    print("Space - стоп")
    print("Q - выход")

    set_servo_angle(90)  # Центрируем серву

    while True:
        key = getch().lower()

        # Управление движением
        if key == "w":
            new_speed = min(100, current_speed + 10)
            set_motor_speed(new_speed)
        elif key == "s":
            new_speed = max(-100, current_speed - 10)
            set_motor_speed(new_speed)
        elif key == " ":
            set_motor_speed(0)

        # Управление поворотом
        elif key == "a":
            new_angle = min(180, current_angle + 10)
            set_servo_angle(new_angle)
        elif key == "d":
            new_angle = max(0, current_angle - 10)
            set_servo_angle(new_angle)

        # Выход
        elif key == "q":
            break

        print_status()

except KeyboardInterrupt:
    pass
finally:
    set_motor_speed(0)
    servo_pwm.stop()
    motor_lpwm.stop()
    motor_rpwm.stop()
    GPIO.cleanup()
    print("\nПрограмма завершена")
