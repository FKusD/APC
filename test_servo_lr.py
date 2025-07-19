from custom_servo import CustomServo
import time

def main():
    servo = CustomServo(
        pwm_pin=18,      # GPIO для ШИМ
        in1_pin=23,      # IN1 MX1508
        in2_pin=24,      # IN2 MX1508
        ads_channel=0    # Канал ADS1115
    )
    
    try:
        servo.set_position(90)  # Двигаем в 90° (не блокирует поток)
        print("Серва движется, основной поток свободен!")
        time.sleep(2)
        
        servo.set_position(75)
        time.sleep(1)
        
    finally:
        servo.cleanup()

if __name__ == "__main__":
    main()