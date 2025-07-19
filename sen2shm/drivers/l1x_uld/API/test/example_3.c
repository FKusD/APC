#include "VL53L1X_api.h"
#include "vl53l1_platform.h"
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>

int main() {
  uint8_t dev = 0x29 << 1; // стандартный адрес VL53L1X (8-битный)
  int status = 0;
  uint8_t sensorState = 0;
  uint16_t distance = 0;
  uint8_t dataReady = 0;
  uint8_t rangeStatus = 0;
  int i;

  // Проверка загрузки чипа
  while (sensorState == 0) {
    status = VL53L1X_BootState(dev, &sensorState);
    VL53L1_WaitMs(dev, 2);
  }
  printf("Chip booted\n");
  printf("Waiting 1 second\n");
  VL53L1_WaitMs(dev, 1000);
  printf("Wait end\n");

  // Инициализация сенсора
  status = VL53L1X_SensorInit(dev);
  printf("SensorInit status: %d\n", status);
  if (status) {
    printf("Sensor init failed: %d\n", status);
    return -1;
  }

  // Смена I2C-адреса на 0x30 (8-битный)
  uint8_t new_addr = 0x30 << 1;
  status = VL53L1X_SetI2CAddress(dev, new_addr);
  printf("SetI2CAddress status: %d\n", status);
  if (status) {
    printf("SetI2CAddress failed: %d\n", status);
    return -1;
  }
  dev = new_addr; // Используем новый адрес для всех последующих вызовов

  // Настройка ROI (например, 4x4)
  status = VL53L1X_SetROI(dev, 4, 4);
  printf("SetROI status: %d\n", status);
  if (status) {
    printf("SetROI failed: %d\n", status);
    return -1;
  }

  // Остальные параметры
  status = VL53L1X_SetDistanceMode(dev, 2); // 1=short, 2=long
  // status = VL53L1X_SetTimingBudgetInMs(dev, 50);
  // status = VL53L1X_SetInterMeasurementInMs(dev, 50);
  printf("VL53L1X Example 2: ROI 4x4, 10 measurements\n");
  status = VL53L1X_StartRanging(dev);
  if (status) {
    printf("StartRanging failed: %d\n", status);
    return -1;
  }
  for (i = 0; i < 20; i++) {
    dataReady = 0;
    while (dataReady == 0) {
      status = VL53L1X_CheckForDataReady(dev, &dataReady);
      VL53L1_WaitMs(dev, 5);
    }
    status = VL53L1X_GetRangeStatus(dev, &rangeStatus);
    status = VL53L1X_GetDistance(dev, &distance);
    printf("Measurement %d: Distance = %u mm, RangeStatus = %u\n", i + 1,
           distance, rangeStatus);
    VL53L1X_ClearInterrupt(dev);
  }
  VL53L1X_StopRanging(dev);
  printf("Done.\n");
  return 0;
}
