/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include "vl53l1_platform.h"

#define I2C_DEV_PATH "/dev/i2c-1"
static int i2c_fd = -1;
static uint16_t current_addr = 0;

static int i2c_init(uint16_t dev_addr) {
    if (i2c_fd >= 0 && current_addr == dev_addr) return 0;
    if (i2c_fd >= 0) close(i2c_fd);
    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C device");
        return -1;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, dev_addr >> 1) < 0) {
        perror("Failed to set I2C address");
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }
    current_addr = dev_addr;
    return 0;
}

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    if (i2c_init(dev) < 0) return -1;
    uint8_t buf[count + 2];
    buf[0] = (index >> 8) & 0xFF;
    buf[1] = index & 0xFF;
    memcpy(&buf[2], pdata, count);
    ssize_t ret = write(i2c_fd, buf, count + 2);
    return (ret == (ssize_t)(count + 2)) ? 0 : -1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    if (i2c_init(dev) < 0) return -1;
    uint8_t reg[2] = { (index >> 8) & 0xFF, index & 0xFF };
    if (write(i2c_fd, reg, 2) != 2) return -1;
    ssize_t ret = read(i2c_fd, pdata, count);
    return (ret == (ssize_t)count) ? 0 : -1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {
    return VL53L1_WriteMulti(dev, index, &data, 1);
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t buf[2] = { (data >> 8) & 0xFF, data & 0xFF };
    return VL53L1_WriteMulti(dev, index, buf, 2);
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t buf[4] = { (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF };
    return VL53L1_WriteMulti(dev, index, buf, 4);
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t buf[2];
    int8_t status = VL53L1_ReadMulti(dev, index, buf, 2);
    if (status == 0) *data = (buf[0] << 8) | buf[1];
    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t buf[4];
    int8_t status = VL53L1_ReadMulti(dev, index, buf, 4);
    if (status == 0) *data = (buf[0]<<24) | (buf[1]<<16) | (buf[2]<<8) | buf[3];
    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    (void)dev;
    struct timespec ts;
    ts.tv_sec = wait_ms / 1000;
    ts.tv_nsec = (wait_ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
    return 0;
}

