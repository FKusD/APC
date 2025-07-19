#!/usr/bin/env python3
"""
Программа для чтения данных датчиков из shared memory
Запуск: python3 read_sensors.py
"""

import mmap
import os
import time
import struct
import signal
import sys
from typing import Dict, Optional
import posix_ipc


# Структура данных датчика (должна соответствовать C структуре)
class SensorData:
    def __init__(self, data: bytes):
        # Распаковываем заголовок: timestamp, sensor_type, resolution, data_format, reserved
        header = struct.unpack("<IBBBB", data[:8])
        self.timestamp = header[0]
        self.sensor_type = header[1]
        self.resolution = header[2]
        self.data_format = header[3]

        # Определяем размер данных в зависимости от формата
        if self.data_format == 0:  # Одиночное измерение
            # 8 байт заголовка + 8 байт данных (distance + status + reserved)
            single_data = struct.unpack("<HBBBBBB", data[8:16])
            self.distance_mm = single_data[0]
            self.status = single_data[1]
            self.matrix_data = None
        else:  # Матричное измерение
            # 8 байт заголовка + 64*2 + 64 = 200 байт данных
            matrix_size = self.resolution
            distances = struct.unpack(f"<{matrix_size}H", data[8 : 8 + matrix_size * 2])
            statuses = struct.unpack(
                f"<{matrix_size}B", data[8 + matrix_size * 2 : 8 + matrix_size * 3]
            )
            self.distances = list(distances)
            self.statuses = list(statuses)
            self.distance_mm = (
                distances[0] if distances else 0
            )  # Для обратной совместимости
            self.status = statuses[0] if statuses else 0

    def __str__(self):
        sensor_names = {0: "VL53L1X", 1: "VL53L5CX", 2: "TCS34725"}
        sensor_name = sensor_names.get(self.sensor_type, f"Unknown({self.sensor_type})")

        # Форматируем время
        time_str = time.strftime("%H:%M:%S", time.localtime(self.timestamp))

        if self.data_format == 0:  # Одиночное измерение
            return f"[{time_str}] {sensor_name}: Distance={self.distance_mm}mm, Status={self.status}"
        else:  # Матричное измерение
            # Полный вывод матрицы (NxN)
            n = int(self.resolution**0.5)
            if n * n != self.resolution:
                # Если не квадратная матрица, выводим одной строкой
                matrix_str = f"Matrix {self.resolution} zones: "
                for i in range(self.resolution):
                    matrix_str += f"Zone{i}={self.distances[i]}mm({self.statuses[i]}) "
                return f"[{time_str}] {sensor_name}: {matrix_str}"
            else:
                # Квадратная матрица (например, 8x8)
                matrix_str = f"Matrix {n}x{n} zones:\n"
                for row in range(n):
                    row_str = ""
                    for col in range(n):
                        idx = row * n + col
                        row_str += f"{self.distances[idx]:4d}({self.statuses[idx]}) "
                    matrix_str += row_str.rstrip() + "\n"
                return f"[{time_str}] {sensor_name}:\n{matrix_str.rstrip()}"


class SensorReader:
    def __init__(self):
        self.running = True
        self.shm_handles: Dict[str, tuple] = {}  # {name: (fd, mmap_obj)}

        # Обработчик сигналов для корректного завершения
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)

    def signal_handler(self, signum, frame):
        """Обработчик сигналов для корректного завершения"""
        print(f"\nПолучен сигнал {signum}, завершение программы...")
        self.running = False

    def open_semaphore(self, shm_name: str):
        sem_name = f"/sem_{shm_name}"
        try:
            sem = posix_ipc.Semaphore(sem_name)
            return sem
        except Exception as e:
            print(f"Ошибка открытия семафора {sem_name}: {e}")
            return None

    def open_shared_memory(self, shm_name: str) -> Optional[tuple]:
        """Открывает shared memory сегмент"""
        try:
            # Открываем shared memory для чтения
            fd = os.open(f"/dev/shm/{shm_name}", os.O_RDONLY)

            # Получаем размер файла
            stat = os.fstat(fd)
            size = stat.st_size

            # Отображаем в память
            mmap_obj = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ)

            print(f"Открыт shared memory: {shm_name} (размер: {size} байт)")
            sem = self.open_semaphore(shm_name)
            if not sem:
                os.close(fd)
                return None
            return (fd, mmap_obj, sem)

        except FileNotFoundError:
            print(f"Shared memory не найден: {shm_name}")
            return None
        except Exception as e:
            print(f"Ошибка открытия {shm_name}: {e}")
            return None

    def read_sensor_data(self, shm_name: str) -> Optional[SensorData]:
        """Читает данные датчика из shared memory"""
        if shm_name not in self.shm_handles:
            handle = self.open_shared_memory(shm_name)
            if handle is None:
                return None
            self.shm_handles[shm_name] = handle

        fd, mmap_obj, sem = self.shm_handles[shm_name]

        try:
            sem.acquire(timeout=1)  # Ждём максимум 1 сек
            mmap_obj.seek(0)
            # Читаем заголовок для определения размера данных
            header_data = mmap_obj.read(8)
            if len(header_data) < 8:
                sem.release()
                return None

            # Распаковываем заголовок
            header = struct.unpack("<IBBBB", header_data)
            resolution = header[2]
            data_format = header[3]

            # Определяем размер данных
            if data_format == 0:  # Одиночное измерение
                data_size = 16  # 8 байт заголовка + 8 байт данных
            else:  # Матричное измерение
                data_size = (
                    8 + resolution * 3
                )  # 8 байт заголовка + resolution*2 (distances) + resolution (statuses)
                if resolution == 0:
                    print(f"{shm_name}: Некорректное разрешение (0), пропуск чтения")
                    sem.release()
                    return None

            # Читаем полные данные
            mmap_obj.seek(0)  # Возвращаемся в начало
            data = mmap_obj.read(data_size)
            if len(data) != data_size:
                print(
                    f"{shm_name}: Недостаточно данных для распаковки (ожидалось {data_size}, получено {len(data)})"
                )
                sem.release()
                return None
            if len(data) == data_size:
                print(f"{shm_name}: RAW HEADER {data[:16].hex()}")
                sem.release()
                return SensorData(data)
            else:
                sem.release()
                return None
        except Exception as e:
            print(f"Ошибка чтения {shm_name}: {e}")
            sem.release()
            return None

    def close_shared_memory(self, shm_name: str):
        """Закрывает shared memory сегмент"""
        if shm_name in self.shm_handles:
            fd, mmap_obj, sem = self.shm_handles[shm_name]
            mmap_obj.close()
            os.close(fd)
            sem.close()
            del self.shm_handles[shm_name]
            print(f"Закрыт shared memory: {shm_name}")

    def cleanup(self):
        """Очистка всех ресурсов"""
        for shm_name in list(self.shm_handles.keys()):
            self.close_shared_memory(shm_name)

    def run(self, sensor_names: list, update_interval: float = 0.1):
        """Основной цикл чтения данных"""
        print("Программа чтения данных датчиков запущена")
        print("Нажмите Ctrl+C для остановки")
        print("-" * 60)

        try:
            while self.running:
                for shm_name in sensor_names:
                    data = self.read_sensor_data(shm_name)
                    if data:
                        print(f"{shm_name}: {data}")
                    else:
                        print(f"{shm_name}: Нет данных")

                print("-" * 60)
                time.sleep(update_interval)

        except KeyboardInterrupt:
            print("\nПолучен сигнал прерывания")
        finally:
            self.cleanup()
            print("Программа завершена")


def main():
    """Главная функция"""
    # Список имен shared memory сегментов (должны соответствовать конфигурации)
    # Можно изменить на нужные имена из sensors_config.txt
    sensor_names = ["vl53l1x_left", "vl53l1x_right", "vl53l5cx_left", "vl53l5cx_right"]

    # Интервал обновления (в секундах)
    update_interval = 0.1

    # Создаем и запускаем читатель
    reader = SensorReader()
    reader.run(sensor_names, update_interval)


if __name__ == "__main__":
    main()

