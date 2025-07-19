#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include "vl53l1_platform.h"
#include "VL53L1X_api.h"

int main() {
    uint16_t dev = 0x52;
    int status = 0;
    uint8_t sensorState = 0;
    uint16_t distance = 0;
    uint8_t dataReady = 0;
    int i;

    // Проверка загрузки чипа
    while(sensorState == 0) {
        status = VL53L1X_BootState(dev, &sensorState);
        VL53L1_WaitMs(dev, 5);
    }
    printf("SensorInit status: %d\n", status);
    printf("Chip booted\n");

    // Инициализация сенсора
    status = VL53L1X_SensorInit(dev);
    if (status) {
        printf("Sensor init failed: %d\n", status);
        return -1;
    }
    status = VL53L1X_SetDistanceMode(dev, 2); // 1=short, 2=long
    status = VL53L1X_SetTimingBudgetInMs(dev, 100);
    status = VL53L1X_SetInterMeasurementInMs(dev, 100);
    printf("VL53L1X Example: 100 measurements\n");
    status = VL53L1X_StartRanging(dev);
    if (status) {
        printf("StartRanging failed: %d\n", status);
        return -1;
    }
    for (i = 0; i < 100; i++) {
        dataReady = 0;
        while (dataReady == 0) {
            status = VL53L1X_CheckForDataReady(dev, &dataReady);
            VL53L1_WaitMs(dev, 2);
        }
        uint8_t rangeStatus = 0;
        status = VL53L1X_GetRangeStatus(dev, &dataReady);
        status = VL53L1X_GetDistance(dev, &distance);
        printf("Measurement %d: Distance = %u mm, rangeStatus = %u\n", i+1, distance, rangeStatus);
        VL53L1X_ClearInterrupt(dev);
    }
    VL53L1X_StopRanging(dev);
    printf("Done.\n");
    return 0;
} 
