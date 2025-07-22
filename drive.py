#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import test_3pin_servo_2 as servo
import main as sensor

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
zero_angle = 100
servo.set_angle(zero_angle)
current_angle = 100
pid = servo.PIDController(Kp=0.6, Ki=0.02, Kd=0.01)

processor = sensor.SensorProcessor(["vl53l5cx_left", "vl53l5cx_right"])
sensor_angles = {"vl53l5cx_left": -45, "vl53l5cx_right": 45}


def set_motor_speed(speed):
    """Управление мотором: speed от -100 до 100"""
    speed = max(-100, min(100, 0))  # Ограничение скорости

    if speed > 0:
        motor_lpwm.ChangeDutyCycle(0)
        motor_rpwm.ChangeDutyCycle(speed)
    elif speed < 0:
        motor_lpwm.ChangeDutyCycle(abs(speed))
        motor_rpwm.ChangeDutyCycle(0)
    else:
        motor_lpwm.ChangeDutyCycle(0)
        motor_rpwm.ChangeDutyCycle(0)


def smooth_start():
    """Плавный старт с 35% -> 20% за 0.5 секунды"""
    set_motor_speed(35)  # Начальный старт на 35%
    start_time = time.time()

    while time.time() - start_time < 0.5:  # В течение 0.5 секунд
        elapsed = time.time() - start_time
        # Линейное уменьшение от 35% до 20%
        current_speed = 35 - (15 * (elapsed / 0.5))
        set_motor_speed(current_speed)
        time.sleep(0.01)  # Небольшая пауза для плавности

    set_motor_speed(20)  # Финишная скорость 20%


try:
    # Плавный старт
    smooth_start()

    while True:
        angles = []
        sensor_distance_values = {}

        for name in processor.sensor_names:
            data = processor.read_sensor(name)
            if data:
                filtered = processor.filter_data(data)
                angle, raw_distances = processor.calculate_angle(
                    data, sensor_angles[name]
                )
                angles.append(angle)

                if raw_distances and len(raw_distances) > 3:
                    dist_value = raw_distances[3]
                else:
                    dist_value = None

                sensor_distance_values[name] = dist_value

                print(f"{name}: расстояния {raw_distances}")
                print(f"{name}: угол {angle:.1f}°")
            else:
                print(f"{name}: нет данных")
                sensor_distance_values[name] = None

        if len(angles) == 2:
            target = sum(angles) + zero_angle
            print(f"Сумма углов: {target:.1f}°")
            control_angle = pid.compute(target, current_angle)

            # Коррекция по расстоянию
            correction_total = 0
            for name in processor.sensor_names:
                dist_value = sensor_distance_values.get(name)
                if dist_value is not None and 0 < dist_value < 500:
                    correction_magnitude = 30 * (1 - (dist_value - 200) / 300)
                    correction_magnitude = max(0, min(30, correction_magnitude))

                    if name == "vl53l5cx_left":
                        correction_total += correction_magnitude
                    elif name == "vl53l5cx_right":
                        correction_total -= correction_magnitude

            corrected_angle = control_angle + correction_total
            corrected_angle = max(0, min(180, corrected_angle))

            servo.set_angle(corrected_angle)
            current_angle = corrected_angle

            print(f"Коррекция по расстоянию: {correction_total:.1f}°")
            print(f"Итоговый угол: {corrected_angle:.1f}°")

        print("---")
        time.sleep(0.07)

except KeyboardInterrupt:
    print("Остановка")
    set_motor_speed(0)  # Остановка мотора
finally:
    set_motor_speed(0)  # Гарантированная остановка
    motor_lpwm.stop()
    motor_rpwm.stop()
    GPIO.cleanup()
