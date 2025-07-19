from gpiozero import Servo
from time import sleep

servo = Servo(18)  # Использует GPIO18

servo.mid()  # 90 градусов
sleep(1)
