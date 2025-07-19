#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <ioctl.h>
#include <wiringPi.h>
#include <vl53l5cx_api.h>

typedef enum { 
    SENSOR_VL53L1X,
    SENSOR_VL53L5CX,
    SENSOR_TCS34725
} SensorType;

typedef struct {
    SensorType type;
    int xshut_pin;
    uint8_t i2c_addr;
    char filename[256];
} SensorConfig;

void read_sensor_data(SensorConfig *config, uint8_t *data) {
    switch (config->type) {
        case SENSOR_VL53L1X:
            vl53l1x_read_data(config->i2c_addr, data);
            break;
        case SENSOR_VL53L5CX:
            vl53l5cx_read_data(config->i2c_addr, data);
            break;
        case SENSOR_TCS34725:
            other_sensor_read_data(config->i2c_addr, data);
            break;
    }
}

int read_config(const char *config_path, SensorConfig *configs, int *count) {
    FILE *file = fopen(config_path, "r");
    if (!file) return -1;

    char type_str[32];
    *count = 0;

    while (fscanf(file, "%31s %d %hhx %255s", 
                 type_str, 
                 &configs[*count].xshut_pin, 
                 &configs[*count].i2c_addr, 
                 configs[*count].filename) == 4) {

        // Преобразуем строку в SensorType
        if (strcmp(type_str, "vl53l1x") == 0) {
            configs[*count].type = SENSOR_VL53L1X;
        } else if (strcmp(type_str, "vl53l5cx") == 0) {
            configs[*count].type = SENSOR_VL53L5CX;
        } else {
            configs[*count].type = SENSOR_OTHER;
        }

        (*count)++;
    }
    fclose(file);
    return 0;
}

int int main(int argc, char *argv[])
{
    SensorConfig configs[6];
    int sensor_count = 0;

    // read config file
    if (read_config("./sensors_config.txt", configs, &sensor_count) != 0) {
        fprintf(stderr, "Error: cant read config");
        return EXIT_FAILURE;
    }

    // sensors init
    //TODO this

    //main loop
    while (1) {
        for (int i = 0; i < 6; i++) {
            if (read_sensor_data(configs[i].i2c_addr, sensor_data) == 0) {
                write_to_file(configs[i].filename, sensor_data, 4);
            }
        }
    }

    return EXIT_SUCCESS;
}
