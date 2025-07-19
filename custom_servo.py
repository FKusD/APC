# custom_servo.py
import threading
import time
import RPi.GPIO as GPIO
from simple_pid import PID
import Adafruit_ADS1x15

class CustomServo:
    def __init__(self, pwm_pin, in1_pin, in2_pin, ads_channel=0):
        # Настройка GPIO и драйвера MX1508
        self.in1 = in1_pin
        self.in2 = in2_pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        
        # Настройка PWM
        self.pwm_pin = pwm_pin
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, 50)  # 50 Hz
        self.pwm.start(0)
        
        # Настройка АЦП (ADS1115)
        self.adc = Adafruit_ADS1x15.ADS1115()
        self.ads_channel = ads_channel
        
        # Параметры управления
        self.target_pos = 0
        self.current_pos = 0
        self.running = False
        self.lock = threading.Lock()  # Для безопасного доступа к target_pos
        
        # ПИД-регулятор
        self.pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=self.target_pos)
        self.pid.output_limits = (-100, 100)
        
        # Запуск потока управления
        self.thread = threading.Thread(target=self._update_position, daemon=True)
        self.thread.start()
    
    def set_position(self, angle):
        """Устанавливает целевой угол (не блокирует основной поток)."""
        target = int((angle / 180.0) * 32767)  # Преобразуем угол в значение АЦП
        with self.lock:
            self.target_pos = target
            self.pid.setpoint = target
    
    def _update_position(self):
        """Фоновый поток: управляет мотором и читает АЦП."""
        self.running = True
        while self.running:
            # Чтение текущего положения
            self.current_pos = self.adc.read_adc(self.ads_channel, gain=1)
            
            # Вычисление ПИД
            with self.lock:
                output = self.pid(self.current_pos)
            
            # Управление мотором
            if output > 1:  # deadzone для предотвращения дребезга
                GPIO.output(self.in1, GPIO.HIGH)
                GPIO.output(self.in2, GPIO.LOW)
                self.pwm.ChangeDutyCycle(min(100, abs(output)))
            elif output < -1:
                GPIO.output(self.in1, GPIO.LOW)
                GPIO.output(self.in2, GPIO.HIGH)
                self.pwm.ChangeDutyCycle(min(100, abs(output)))
            else:  # Остановка, если ошибка мала
                GPIO.output(self.in1, GPIO.LOW)
                GPIO.output(self.in2, GPIO.LOW)
                self.pwm.ChangeDutyCycle(0)
            
            time.sleep(0.01)  # Частота обновления (можно уменьшить)
    
    def stop(self):
        """Остановка потока и мотора."""
        self.running = False
        self.thread.join()
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
    
    def cleanup(self):
        self.stop()
        self.pwm.stop()
        GPIO.cleanup()