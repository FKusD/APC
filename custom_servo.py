# custom_servo.py
import threading
import time
import RPi.GPIO as GPIO
from simple_pid import PID
import Adafruit_ADS1x15

class CustomServo:
    def __init__(self, pwm_pin, in1_pin, in2_pin, ads_channel=0, auto_start=True):
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
        self.lock = threading.Lock()
        self.auto_start = auto_start  # Флаг автоматического запуска
        
        # ПИД-регулятор
        self.pid = PID(Kp=1.0, Ki=0.1, Kd=0.05, setpoint=self.target_pos)
        self.pid.output_limits = (-100, 100)
        
        # Запуск потока управления (если auto_start=True)
        if self.auto_start:
            self.start()
    
    def start(self):
        """Запускает поток управления сервой."""
        if not self.running:
            self.thread = threading.Thread(target=self._update_position, daemon=True)
            self.running = True
            self.thread.start()
    
    def set_position(self, angle):
        """Устанавливает целевой угол (0-180°)."""
        if not self.running:
            print("Предупреждение: серва не запущена. Используйте start().")
            return
        
        target = int((angle / 180.0) * 32767)  # Преобразуем в значение АЦП
        with self.lock:
            self.target_pos = target
            self.pid.setpoint = target
    
    def get_position(self):
        """Возвращает текущий угол сервы (0-180°)."""
        return (self.current_pos / 32767) * 180  # Преобразуем АЦП в угол
    
    def _update_position(self):
        """Фоновый поток: управляет мотором и читает АЦП."""
        while self.running:
            self.current_pos = self.adc.read_adc(self.ads_channel, gain=1)
            
            with self.lock:
                output = self.pid(self.current_pos)
            
            if output > 1:
                GPIO.output(self.in1, GPIO.HIGH)
                GPIO.output(self.in2, GPIO.LOW)
                self.pwm.ChangeDutyCycle(min(100, abs(output)))
            elif output < -1:
                GPIO.output(self.in1, GPIO.LOW)
                GPIO.output(self.in2, GPIO.HIGH)
                self.pwm.ChangeDutyCycle(min(100, abs(output)))
            else:
                GPIO.output(self.in1, GPIO.LOW)
                GPIO.output(self.in2, GPIO.LOW)
                self.pwm.ChangeDutyCycle(0)
            
            time.sleep(0.01)
    
    def stop(self):
        """Останавливает серву."""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join()
        GPIO.output(self.in1, GPIO.LOW)
        GPIO.output(self.in2, GPIO.LOW)
        self.pwm.ChangeDutyCycle(0)
    
    def cleanup(self):
        """Очистка ресурсов."""
        self.stop()
        self.pwm.stop()
        GPIO.cleanup()