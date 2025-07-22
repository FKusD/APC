#!/usr/bin/env python3
# Версия 3

"""
Чтение данных VL53L5CX с фильтрацией (50-20000 мм),
расчетом угла по всем 8 точкам и суммой углов
"""

import mmap
import os
import time
import struct
import numpy as np
import math


class SensorProcessor:
    def __init__(self, sensor_names):
        self.sensor_names = sensor_names
        self.min_dist = 50
        self.max_dist = 20000
        # Углы лучей в градусах (8 точек от -22.5 до 22.5)
        self.angles = np.linspace(-22.5, 22.5, 8)

    def read_sensor(self, name):
        try:
            with open(f"/dev/shm/{name}", "rb") as f:
                data = f.read(200)
                distances = struct.unpack("<64H", data[8:136])
                return distances[24:32]  # 4-я строка (все 8 точек)
        except:
            return None

    def filter_data(self, distances):
        return [d if self.min_dist <= d <= self.max_dist else 0 for d in distances]

    def calculate_angle(self, data, sensor_angle):
        # Создаем список расстояний для средней строки (строка 4)
        raw_distances = []
        for j in range(8):
            idx = 4 * 8 + j
            if data.statuses[idx] == 255 or not (50 <= data.distances[idx] <= 4000):
                raw_distances.append(3333)
            else:
                raw_distances.append(data.distances[idx])

        # Собираем валидные точки для строк 3, 4, 5
        points = []
        for i in [3, 4, 5]:
            for j in range(8):
                idx = i * 8 + j
                status = data.statuses[idx]
                dist = data.distances[idx]

                # Пропускаем невалидные точки
                if status == 255 or not (50 <= dist <= 4000):
                    continue

                # Рассчитываем углы и координаты
                alpha = (j - 3.5) * (45.0 / 8.0)
                absolute_angle = sensor_angle + alpha
                rad = math.radians(absolute_angle)
                x = dist * math.cos(rad)
                y = -dist * math.sin(rad)  # учтена система координат (Y+ влево)
                points.append((x, y))

        # Возвращаем 90° при недостатке точек
        if len(points) < 2:
            return 90.0, raw_distances

        # Линейная регрессия y = a*x + b
        n = len(points)
        sum_x = sum(x for x, y in points)
        sum_y = sum(y for x, y in points)
        sum_xy = sum(x * y for x, y in points)
        sum_x2 = sum(x**2 for x, y in points)

        denominator = n * sum_x2 - sum_x**2
        if abs(denominator) < 1e-6:
            a = 1e6  # вертикальная стена
        else:
            a = (n * sum_xy - sum_x * sum_y) / denominator

        # Рассчет угла в градусах
        angle_rad = math.atan(a)
        angle_deg = math.degrees(angle_rad)
        return angle_deg, raw_distances


def main():
    processor = SensorProcessor(["vl53l5cx_left", "vl53l5cx_right"])
    sensor_angles = {"vl53l5cx_left": -45, "vl53l5cx_right": 45}

    try:
        while True:
            angles = []
            for name in processor.sensor_names:
                data = processor.read_sensor(name)
                if data:
                    filtered = processor.filter_data(data)
                    angle, raw_distances = processor.calculate_angle(
                        data, sensor_angles[name]
                    )
                    angles.append(angle)
                    print(f"{name}: расстояния {raw_distances}")
                    print(f"{name}: угол {angle:.1f}°")
                else:
                    print(f"{name}: нет данных")

            if len(angles) == 2:
                print(f"Сумма углов: {sum(angles):.1f}°")
            print("---")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Остановка")


if __name__ == "__main__":
    main()
