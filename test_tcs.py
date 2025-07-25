import time
import adafruit_tcs34725
import board
from adafruit_bitbangio import I2C

# Просто передаём board.D9 и board.D10!
i2c = I2C(board.D9, board.D10)  # SCL, SDA

sensor = adafruit_tcs34725.TCS34725(i2c)

while True:
    print("lux= ", sensor.lux)
    print("temp= ", sensor.color_temperature)
    print("rgb= ", sensor.color_rgb_bytes)
    time.sleep(0.5)