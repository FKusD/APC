#!/usr/bin/env python3
# Версия 6 - Финальная оптимизация

"""
Обработчик данных VL53L5CX для интеграции с drive.py:
- Только расчет углов (без коррекции по расстоянию)
- Разница углов: left_angle - right_angle
- Минимум 4 точки для расчета
- None для невалидных данных
- Нулевой угол по умолчанию
"""

import mmap
import os
import time
import struct
import numpy as np
import math
from typing import Optional, Tuple, List


class SensorProcessor:
    def __init__(self, sensor_names: List[str]):
        self.sensor_names = sensor_names
        self.min_dist = 50  # Минимальное валидное расстояние (мм)
        self.max_dist = 20000  # Максимальное валидное расстояние (мм)

        # Углы лучей в градусах для 8 точек в строке (от -22.5 до 22.5)
        self.beam_angles = np.linspace(-22.5, 22.5, 8)

        # Используем только среднюю строку (4) для упрощения
        self.used_row = 4
        self.min_points = 4  # Минимальное количество точек для расчета

    def read_sensor(self, name: str) -> Optional[dict]:
        """Читает данные датчика из shared memory"""
        try:
            with open(f"/dev/shm/{name}", "rb") as f:
                data = f.read(200)  # Читаем весь блок данных

                # Распаковываем расстояния (64 uint16) и статусы (64 uint8)
                distances = struct.unpack("<64H", data[8:136])
                statuses = struct.unpack("<64B", data[136:200])

                return {
                    "distances": distances,
                    "statuses": statuses,
                    "timestamp": time.time(),
                }
        except Exception as e:
            print(f"Ошибка чтения {name}: {str(e)}")
            return None

    def filter_point(self, distance: int, status: int) -> bool:
        """Проверяет валидность точки"""
        return status != 255 and self.min_dist <= distance <= self.max_dist

    def calculate_angle(
        self, sensor_data: dict, sensor_angle: float
    ) -> Tuple[float, list]:
        """
        Расчет угла наклона стены

        Args:
            sensor_data: Словарь с 'distances' и 'statuses'
            sensor_angle: Угол установки датчика (градусы)

        Returns:
            tuple: (угол в градусах, список расстояний для средней строки)
        """
        if (
            not sensor_data
            or "distances" not in sensor_data
            or "statuses" not in sensor_data
        ):
            return 0.0, [None] * 8  # Нулевой угол по умолчанию

        distances = sensor_data["distances"]
        statuses = sensor_data["statuses"]

        # 1. Получаем расстояния для средней строки
        middle_row_distances = []
        valid_points = []

        for col in range(8):
            idx = self.used_row * 8 + col
            if idx < len(distances):
                if self.filter_point(distances[idx], statuses[idx]):
                    middle_row_distances.append(distances[idx])
                    valid_points.append((self.beam_angles[col], distances[idx]))
                else:
                    middle_row_distances.append(None)
            else:
                middle_row_distances.append(None)

        # 2. Если валидных точек меньше минимального - возвращаем 0°
        if len(valid_points) < self.min_points:
            return 0.0, middle_row_distances

        # 3. Преобразуем точки в координаты (x, y)
        points = []
        for angle, dist in valid_points:
            abs_angle = math.radians(sensor_angle + angle)
            x = dist * math.cos(abs_angle)  # X - вперед
            y = -dist * math.sin(abs_angle)  # Y - влево
            points.append((x, y))

        # 4. Линейная регрессия (y = a*x + b)
        n = len(points)
        sum_x = sum(x for x, y in points)
        sum_y = sum(y for x, y in points)
        sum_xy = sum(x * y for x, y in points)
        sum_x2 = sum(x**2 for x, y in points)

        denominator = n * sum_x2 - sum_x**2
        if abs(denominator) < 1e-6:
            angle_deg = 0.0  # Нулевой угол при вертикальной стене
        else:
            a = (n * sum_xy - sum_x * sum_y) / denominator
            angle_rad = math.atan(a)
            angle_deg = math.degrees(angle_rad)

        return angle_deg, middle_row_distances


# Методы для использования в drive.py
def create_processor(sensor_names):
    return SensorProcessor(sensor_names)


def calculate_angles(processor, sensor_angles):
    """Основная функция для вызова из drive.py"""
    angles = {}
    distances = {}

    for name in processor.sensor_names:
        raw_data = processor.read_sensor(name)

        if raw_data:
            angle, dist = processor.calculate_angle(raw_data, sensor_angles[name])
            print(angle)
            angles[name] = angle
            distances[name] = dist
        else:
            angles[name] = 0.0
            distances[name] = [None] * 8

    # Вычисляем разницу углов (left - right)
    angle_diff = -angles.get(processor.sensor_names[0], 0.0) - angles.get(
        processor.sensor_names[1], 0.0
    )
    print(
        angles.get(processor.sensor_names[0], 0.0),
        "  ",
        angles.get(processor.sensor_names[1], 0.0),
    )

    return angle_diff, distances
