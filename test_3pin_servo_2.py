import RPi.GPIO as GPIO
from time import time, sleep

SERVO_PIN = 18

GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

pwm = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz
pwm.start(0)


def set_angle(angle):
    duty = angle / 18 + 2
    pwm.ChangeDutyCycle(duty)
    sleep(0.05)


class PIDController:
    def __init__(self, Kp, Ki, Kd, min_angle=80, max_angle=135):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.reset()

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = time()

    def compute(self, setpoint, current_angle):
        error = setpoint - current_angle
        current_time = time()
        dt = current_time - self.last_time
        self.last_time = current_time

        P = self.Kp * error
        self.integral += error * dt
        I = self.Ki * self.integral
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        D = self.Kd * derivative
        self.prev_error = error

        output = current_angle + P + I + D
        return max(min(output, self.max_angle), self.min_angle)


# Тестовая последовательность из 20 значений (в градусах)
TEST_SEQUENCE = [
    110,
    100,
    120,
    90,
    130,  # Основные точки
    110,
    105,
    115,
    95,
    125,  # Средние значения
    110,
    85,
    135,
    110,  # Крайние положения
    100,
    120,
    110,
    80,
    135,
    110,  # Комбинированные переходы
]


def run_test_sequence():
    pid = PIDController(Kp=0.6, Ki=0.02, Kd=0.01, min_angle=80, max_angle=135)

    current_angle = 100  # Начальное положение
    set_angle(current_angle)
    print("Инициализация сервы в 110°")
    sleep(1)

    for i, target in enumerate(TEST_SEQUENCE):
        print(f"\nТест {i + 1}/20: Переход к {target}°")

        # Плавный переход к цели
        for _ in range(30):  # 30 итераций на каждую цель
            control_angle = pid.compute(target, current_angle)
            set_angle(control_angle)
            current_angle = (
                control_angle  # В реальной системе заменить на чтение датчика
            )

            # Вывод текущего состояния
            print(f"Цель: {target}° | Текущий: {current_angle:.1f}°", end="\r")
            sleep(0.05)

        # Краткая пауза между тестами
        sleep(0.3)

    # Возврат в центральное положение
    set_angle(100)
    sleep(0.3)
    print("\n\nТестирование завершено. Серва возвращена в 110°")


if __name__ == "__main__":
    try:
        run_test_sequence()
    except KeyboardInterrupt:
        set_angle(100)
        sleep(0.5)
        pwm.stop()
        GPIO.cleanup()
        print("\nАварийный останов. Серва в 110°")
    finally:
        pwm.stop()
        GPIO.cleanup()
