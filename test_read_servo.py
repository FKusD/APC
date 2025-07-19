# test_servo.py
from custom_servo import CustomServo
import time

def main():
    # Инициализация без автозапуска (auto_start=False)
    servo = CustomServo(18, 23, 24, ads_channel=0, auto_start=False)
    
    try:
        print("Тестирование сервы. Только чтение угла.")
        while True:
            angle = servo.get_position()
            print(f"Текущий угол: {angle:.1f}°")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\nЗавершение...")
    finally:
        print("Cleaning up...")
        servo.cleanup()

if __name__ == "__main__":
    main()