#include <VL53L1X_api.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include <vl53l5cx_api.h>
#include <wiringPi.h>
#include <semaphore.h>


typedef enum { SENSOR_VL53L1X, SENSOR_VL53L5CX, SENSOR_TCS34725 } SensorType;

typedef struct {
  SensorType type;
  int xshut_pin;
  uint8_t i2c_addr;
  char shm_name[256]; // Имя shared memory сегмента
  int initialized;    // Флаг инициализации

  // Указатель на конфигурацию датчика (используется только для VL53L5CX)
  void *sensor_config;

  // Shared memory дескрипторы
  int shm_fd;
  void *shm_ptr;
  sem_t *sem; // дескриптор семафора
} SensorConfig;

// Структура данных датчика в shared memory
typedef struct {
  uint32_t timestamp;  // Временная метка
  uint8_t sensor_type; // Тип датчика (0=VL53L1X, 1=VL53L5CX, 2=TCS34725)
  uint8_t resolution;  // Разрешение (1 для одиночного, 16 для 4x4, 64 для 8x8)
  uint8_t data_format; // Формат данных (0=одиночное, 1=матрица)
  uint8_t reserved;    // Зарезервировано

  // Объединение для разных форматов данных
  union {
    // Одиночное измерение (VL53L1X, TCS34725)
    struct {
      uint16_t distance_mm; // Расстояние в мм
      uint8_t status;       // Статус измерения
      uint8_t reserved[5];  // Зарезервировано
    } single;

    // Матричное измерение (VL53L5CX)
    struct {
      uint16_t distances[64]; // Массив расстояний (максимум 8x8)
      uint8_t statuses[64];   // Массив статусов
    } matrix;
  } data;
} SensorData;

// Глобальные переменные для I2C
static int i2c_fd = -1;
// static uint16_t current_addr = 0;

// Глобальная переменная для отслеживания состояния программы
static volatile int running = 1;

// Обработчик сигналов для корректного завершения
void signal_handler(int sig) {
  printf("\nПолучен сигнал %d, завершение программы...\n", sig);
  running = 0;
}

// Функция для проверки наличия устройства на I2C адресе
int check_i2c_device(uint8_t addr) {
  if (i2c_fd < 0) {
    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
      perror("Failed to open I2C device");
      return -1;
    }
  }

  if (ioctl(i2c_fd, I2C_SLAVE, addr) < 0) {
    return -1;
  }

  // Попытка чтения байта для проверки наличия устройства
  uint8_t test_byte;
  if (read(i2c_fd, &test_byte, 1) < 0) {
    return -1;
  }

  return 0;
}

// Функция для создания shared memory сегмента
int create_shared_memory(SensorConfig *config) {
  // Определяем размер сегмента в зависимости от типа датчика
  size_t shm_size;
  if (config->type == SENSOR_VL53L5CX) {
    shm_size = 8 + 64 * 3; // 8 байт заголовка + 64*2 (distances) + 64 (statuses)
  } else {
    shm_size = sizeof(SensorData); // Для одиночных датчиков
  }

  // Создаем shared memory сегмент
  config->shm_fd =
      shm_open(config->shm_name, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
  if (config->shm_fd == -1) {
    perror("shm_open failed");
    return -1;
  }

  // Устанавливаем размер сегмента
  if (ftruncate(config->shm_fd, shm_size) == -1) {
    perror("ftruncate failed");
    close(config->shm_fd);
    return -1;
  }

  // Отображаем shared memory в адресное пространство процесса
  config->shm_ptr = mmap(NULL, shm_size, PROT_READ | PROT_WRITE, MAP_SHARED,
                         config->shm_fd, 0);
  if (config->shm_ptr == MAP_FAILED) {
    perror("mmap failed");
    close(config->shm_fd);
    return -1;
  }

  // Инициализируем данные нулями
  memset(config->shm_ptr, 0, shm_size);

  printf("Shared memory создан: %s (размер: %zu байт)\n", config->shm_name,
         shm_size);

  // Создаем именованный семафор
  char sem_name[256];
  snprintf(sem_name, sizeof(sem_name), "/sem_%.250s", config->shm_name);
  config->sem = sem_open(sem_name, O_CREAT, 0666, 1); // 1 — начальное значение (разрешено читать)
  if (config->sem == SEM_FAILED) {
      perror("sem_open failed");
      munmap(config->shm_ptr, shm_size);
      close(config->shm_fd);
      return -1;
  }

  return 0;
}

// Функция для записи одиночных данных в shared memory
int write_single_to_shm(SensorConfig *config, uint16_t distance,
                        uint8_t status) {
  if (!config->shm_ptr) {
    printf("Error: Shared memory не инициализирован для %s\n",
           config->shm_name);
    return -1;
  }

  sem_wait(config->sem);

  SensorData *data = (SensorData *)config->shm_ptr;

  // Обновляем данные
  data->timestamp = (uint32_t)time(NULL);
  data->sensor_type = config->type;
  data->resolution = 1;  // Одиночное измерение
  data->data_format = 0; // Формат одиночного измерения
  data->data.single.distance_mm = distance;
  data->data.single.status = status;

  sem_post(config->sem);
  return 0;
}

// Функция для записи матричных данных в shared memory
int write_matrix_to_shm(SensorConfig *config, uint16_t *distances,
                        uint8_t *statuses, uint8_t resolution) {
  if (!config->shm_ptr) {
    printf("Error: Shared memory не инициализирован для %s\n",
           config->shm_name);
    return -1;
  }

  sem_wait(config->sem);

  SensorData *data = (SensorData *)config->shm_ptr;

  // Обновляем данные
  data->timestamp = (uint32_t)time(NULL);
  data->sensor_type = config->type;
  data->resolution = resolution; // 16 для 4x4, 64 для 8x8
  data->data_format = 1;         // Формат матричного измерения

  // Копируем данные матрицы
  for (int i = 0; i < resolution; i++) {
    data->data.matrix.distances[i] = distances[i];
    data->data.matrix.statuses[i] = statuses[i];
  }

  sem_post(config->sem);
  return 0;
}

// Функция для закрытия shared memory
void close_shared_memory(SensorConfig *config) {
  if (config->shm_ptr && config->shm_ptr != MAP_FAILED) {
    size_t shm_size = 8 + 64 * 3; // Максимальный размер для матрицы 8x8
    munmap(config->shm_ptr, shm_size);
    config->shm_ptr = NULL;
  }

  if (config->shm_fd >= 0) {
    close(config->shm_fd);
    config->shm_fd = -1;
  }

  // Удаляем shared memory сегмент
  shm_unlink(config->shm_name);
  printf("Shared memory закрыт: %s\n", config->shm_name);

  // Закрываем и удаляем семафор
  char sem_name[256];
  snprintf(sem_name, sizeof(sem_name), "/sem_%.250s", config->shm_name);
  if (config->sem) {
      sem_close(config->sem);
      sem_unlink(sem_name);
      config->sem = NULL;
  }
}

// Функция для инициализации VL53L1X
int init_vl53l1x_sensor(uint8_t addr) {
  uint16_t dev = addr;
  uint8_t sensorState = 0;
  int status = 0;

  // Проверка загрузки чипа
  while (sensorState == 0) {
    status = VL53L1X_BootState(dev, &sensorState);
    if (status != 0) {
      printf("VL53L1X boot state check failed: %d\n", status);
      return -1;
    }
    VL53L1_WaitMs(dev, 2);
  }
  printf("VL53L1X chip booted at address 0x%02X\n", addr);
  VL53L1_WaitMs(dev, 100);

  // Инициализация сенсора
  status = VL53L1X_SensorInit(dev);
  if (status != 0) {
    printf("VL53L1X sensor init failed: %d\n", status);
    return -1;
  }

  // Настройка параметров
  status = VL53L1X_SetDistanceMode(dev, 2); // Long mode
  status = VL53L1X_SetTimingBudgetInMs(dev, 100);
  status = VL53L1X_SetInterMeasurementInMs(dev, 100);

  printf("VL53L1X initialized successfully at address 0x%02X\n", addr);
  return 0;
}

// Функция для инициализации VL53L5CX
int init_vl53l5cx_sensor(uint8_t addr, SensorConfig *sensor_config) {
  uint8_t isAlive, status;

  // Выделяем память для конфигурации VL53L5CX
  VL53L5CX_Configuration *config = malloc(sizeof(VL53L5CX_Configuration));
  if (!config) {
    printf("Failed to allocate memory for VL53L5CX configuration\n");
    return -1;
  }

  // Инициализируем конфигурацию нулями
  memset(config, 0, sizeof(VL53L5CX_Configuration));

  // Настройка платформы
  config->platform.address = addr;
  config->platform.fd = open("/dev/i2c-1", O_RDONLY);
  printf("config->platform.address: %d\n", config->platform.address);

  // Проверка наличия датчика
  status = vl53l5cx_is_alive(config, &isAlive);
  if (!isAlive || status) {
    printf("VL53L5CX not detected at address 0x%02X\n", addr);
    free(config);
    return -1;
  }

  // Инициализация датчика
  status = vl53l5cx_init(config);
  if (status) {
    printf("VL53L5CX ULD Loading failed: %d\n", status);
    free(config);
    return -1;
  }

  status = vl53l5cx_set_resolution(config, 64);
  if (status) {
    printf("VL53L5CX resolution set failed: %d\n", status);
    free(config);
    return -1;
  }

  // Сохраняем указатель на конфигурацию
  sensor_config->sensor_config = config;

  printf("VL53L5CX initialized successfully at address 0x%02X\n", addr);
  return 0;
}

int init_gpio(SensorConfig *configs, int sensor_count) {
  if (wiringPiSetupGpio() == -1) {
    printf("Error: wiringPi init\n");
    return -1;
  }

  // Проверяем корректность конфигурации
  for (int i = 0; i < sensor_count; i++) {
    // Проверяем, что I2C адрес в допустимом диапазоне (0x08-0x77)
    if (configs[i].i2c_addr < 0x08 || configs[i].i2c_addr > 0x77) {
      printf("Error: Invalid I2C address 0x%02X for sensor %d\n",
             configs[i].i2c_addr, i);
      return -1;
    }

    // Проверяем, что GPIO пин в допустимом диапазоне
    if (configs[i].xshut_pin < 0 || configs[i].xshut_pin > 40) {
      printf("Error: Invalid GPIO pin %d for sensor %d\n", configs[i].xshut_pin,
             i);
      return -1;
    }
  }

  // Проверяем уникальность I2C адресов
  for (int i = 0; i < sensor_count; i++) {
    for (int j = i + 1; j < sensor_count; j++) {
      if (configs[i].i2c_addr == configs[j].i2c_addr) {
        printf("Error: Duplicate I2C address 0x%02X for sensors %d and %d\n",
               configs[i].i2c_addr, i, j);
        return -1;
      }
    }
  }

  // Сначала все пины XSHUT устанавливаем в LOW (выключаем все датчики)
  for (int i = 0; i < sensor_count; i++) {
    pinMode(configs[i].xshut_pin, OUTPUT);
    digitalWrite(configs[i].xshut_pin, LOW);
    configs[i].initialized = 0;
    configs[i].sensor_config = NULL; // Инициализируем указатель на конфигурацию
    configs[i].shm_fd = -1;
    configs[i].shm_ptr = NULL;
    configs[i].sem = NULL; // Инициализируем семафор
  }

  // Ждем немного для стабилизации
  delay(100);

  // Теперь включаем датчики по одному и проверяем их
  for (int i = 0; i < sensor_count; i++) {
    printf("Checking sensor %d (pin %d, addr 0x%02X)...\n", i,
           configs[i].xshut_pin, configs[i].i2c_addr);

    // Включаем текущий датчик
    digitalWrite(configs[i].xshut_pin, HIGH);
    delay(100); // Ждем загрузки датчика

    // Проверяем стандартный адрес 0x29 (0x52 в 7-bit формате)
    if (check_i2c_device(0x29) == 0) {
      printf("Found sensor at default address 0x29\n");

      // Инициализируем датчик в зависимости от типа
      int init_status = -1;
      switch (configs[i].type) {
      case SENSOR_VL53L1X:
        init_status = init_vl53l1x_sensor(0x29 << 1);
        if (init_status == 0) {
          // Меняем адрес на нужный
          if (configs[i].i2c_addr != 0x29) {
            init_status = VL53L1X_SetI2CAddress(0x29 << 1, configs[i].i2c_addr << 1);
            if (init_status != 0) {
              printf("Failed to change VL53L1X address\n");
              break; // или break, если хотите прервать обработку этого датчика
            }
            printf("VL53L1X address changed to 0x%02X\n", configs[i].i2c_addr);
          }
          // Всегда пытаемся создать shm:
          if (create_shared_memory(&configs[i]) == 0) {
            configs[i].initialized = 1;
          } else {
            printf("Failed to create shared memory for sensor %d\n", i);
          }
        }
        break;

      case SENSOR_VL53L5CX:
        printf("init_vl53l5cx_sensor(0x%02X, &configs[%d])\n",
               configs[i].i2c_addr, i);
        init_status = init_vl53l5cx_sensor(0x29 << 1, &configs[i]);
        if (init_status == 0) {
          // Меняем адрес на нужный
          if (configs[i].i2c_addr != 0x29) {
            VL53L5CX_Configuration *config =
                (VL53L5CX_Configuration *)configs[i].sensor_config;
            init_status =
                vl53l5cx_set_i2c_address(config, configs[i].i2c_addr << 1);
            if (init_status != 0) {
              printf("Failed to change VL53L5CX address\n");
              break; // или break, если хотите прервать обработку этого датчика
            }
            printf("VL53L5CX address changed to 0x%02X\n", configs[i].i2c_addr);
          }
          // Всегда пытаемся создать shm:
          if (create_shared_memory(&configs[i]) == 0) {
            configs[i].initialized = 1;
          } else {
            printf("Failed to create shared memory for sensor %d\n", i);
          }
        }
        break;

      case SENSOR_TCS34725:
        // TODO: Реализовать для TCS34725
        printf("TCS34725 initialization not implemented yet\n");
        break;
      }
    } else {
      // Проверяем, может датчик уже на нужном адресе
      if (check_i2c_device(configs[i].i2c_addr) == 0) {
        printf("Sensor already at target address 0x%02X\n",
               configs[i].i2c_addr);

        // Проверяем, что датчик работает
        int init_status = -1;
        switch (configs[i].type) {
        case SENSOR_VL53L1X:
          init_status = init_vl53l1x_sensor(configs[i].i2c_addr << 1);
          break;
        case SENSOR_VL53L5CX:
          init_status = init_vl53l5cx_sensor(configs[i].i2c_addr << 1, &configs[i]);
          break;
        case SENSOR_TCS34725:
          // TODO: Реализовать для TCS34725
          break;
        }

        if (init_status == 0) {
          // Создаем shared memory для датчика
          if (create_shared_memory(&configs[i]) == 0) {
            configs[i].initialized = 1;
          } else {
            printf("Failed to create shared memory for sensor %d\n", i);
          }
        }
      } else {
        printf("No sensor found for configuration %d\n", i);
      }
    }

    // Выключаем датчик перед проверкой следующего
    // digitalWrite(configs[i].xshut_pin, LOW);
    // delay(50);
  }

  // Включаем все инициализированные датчики
  // for (int i = 0; i < sensor_count; i++) {
  //   if (configs[i].initialized) {
  //     digitalWrite(configs[i].xshut_pin, HIGH);
  //     printf("Sensor %d enabled (pin %d, addr 0x%02X)\n", i,
  //            configs[i].xshut_pin, configs[i].i2c_addr);
  //   }
  // }

  return 0;
}

// Функция для остановки всех датчиков
void stop_all_sensors(SensorConfig *configs, int sensor_count) {
  printf("Остановка всех датчиков...\n");

  for (int i = 0; i < sensor_count; i++) {
    if (configs[i].initialized) {
      switch (configs[i].type) {
      case SENSOR_VL53L1X:
        VL53L1X_StopRanging(configs[i].i2c_addr);
        printf("VL53L1X остановлен (адрес 0x%02X)\n", configs[i].i2c_addr);
        break;
      case SENSOR_VL53L5CX: {
        VL53L5CX_Configuration *config =
            (VL53L5CX_Configuration *)configs[i].sensor_config;
        if (config) {
          vl53l5cx_stop_ranging(config);
          printf("VL53L5CX остановлен (адрес 0x%02X)\n", configs[i].i2c_addr);
          // Освобождаем память
          free(config);
          configs[i].sensor_config = NULL;
        }
        break;
      }
      case SENSOR_TCS34725:
        // TODO: Остановка для TCS34725
        break;
      }

      // Закрываем shared memory для датчика
      close_shared_memory(&configs[i]);

      // Выключаем питание датчика
      digitalWrite(configs[i].xshut_pin, LOW);
    }
  }

  // Закрываем I2C файловый дескриптор
  if (i2c_fd >= 0) {
    close(i2c_fd);
    i2c_fd = -1;
  }
}

int read_sensor_data(SensorConfig *config, uint8_t *data) {
  switch (config->type) {
  case SENSOR_VL53L1X: {
    uint8_t dev = config->i2c_addr << 1;
    uint8_t dataReady = 0;
    uint16_t distance = 0;
    uint8_t rangeStatus = 0;

    // Проверяем готовность данных
    if (VL53L1X_CheckForDataReady(dev, &dataReady) != 0) {
      printf("VL53L1X_CheckForDataReady error\n");
      return -1;
    }

    if (dataReady) {
      // Получаем статус и расстояние
      if (VL53L1X_GetRangeStatus(dev, &rangeStatus) != 0) {
        printf("VL53L1X_GetRangeStatus error\n");
        return -1;
      }

      if (VL53L1X_GetDistance(dev, &distance) != 0) {
        printf("VL53L1X_GetDistance error\n");
        return -1;
      }

      // Очищаем прерывание
      VL53L1X_ClearInterrupt(dev);

      // Записываем данные в буфер (4 байта: 2 байта расстояния + 2 байта
      // статуса)
      data[0] = (distance >> 8) & 0xFF;
      data[1] = distance & 0xFF;
      data[2] = 0;                  // Старший байт статуса всегда 0 для VL53L1X
      data[3] = rangeStatus & 0xFF; // Младший байт статуса

      return 0;
    }
    return -1;
  }

  case SENSOR_VL53L5CX: {
    VL53L5CX_ResultsData results;
    uint8_t isReady = 0;
    VL53L5CX_Configuration *vl53l5cx_config =
        (VL53L5CX_Configuration *)config->sensor_config;

    if (!vl53l5cx_config) {
      printf("Error: VL53L5CX configuration not found\n");
      return -1;
    }

    // Проверяем готовность данных
    if (vl53l5cx_check_data_ready(vl53l5cx_config, &isReady) != 0) {
      return -1;
    }

    if (isReady) {
      // Получаем данные
      if (vl53l5cx_get_ranging_data(vl53l5cx_config, &results) != 0) {
        return -1;
      }

      // Получаем текущее разрешение
      uint8_t resolution;
      if (vl53l5cx_get_resolution(vl53l5cx_config, &resolution) != 0) {
        printf("Warning: vl53l5cx_get_resolution error, пропуск записи\n");
        return -1;
      }
      if (resolution == 0) {
        printf("Warning: resolution==0, пропуск записи в shared memory\n");
        return -1;
      }
      printf("VL53L5CX: resolution = %d\n", resolution);

      // Подготавливаем массивы для матричных данных
      uint16_t distances[64];
      uint8_t statuses[64];

      // Копируем данные из всех зон
      for (int i = 0; i < resolution; i++) {
        distances[i] = results.distance_mm[i];
        statuses[i] = results.target_status[i];
      }

      // Записываем матричные данные в shared memory
      if (write_matrix_to_shm(config, distances, statuses, resolution) == 0) {
        // Для обратной совместимости также записываем в буфер данные первой
        // зоны
        data[0] = (distances[0] >> 8) & 0xFF;
        data[1] = distances[0] & 0xFF;
        data[2] = 0;
        data[3] = statuses[0] & 0xFF;
        return 0;
      }
    }
    return -1;
  }

  case SENSOR_TCS34725:
    // TODO: Реализовать чтение данных TCS34725
    return -1;
  }
  return -1;
}

// delete this
// Функция для записи данных в файл
int write_to_file(const char *filename, uint8_t *data, int size) {
  FILE *file = fopen(filename, "wb");
  if (!file) {
    perror("Failed to open file for writing");
    return -1;
  }

  size_t written = fwrite(data, 1, size, file);
  fclose(file);

  if (written != size) {
    perror("Failed to write data to file");
    return -1;
  }

  return 0;
}

int read_config(const char *config_path, SensorConfig *configs, int *count) {
  FILE *file = fopen(config_path, "r");
  if (!file) {
    perror("Failed to open config file");
    return -1;
  }

  char line[512];
  char type_str[32];
  *count = 0;

  while (fgets(line, sizeof(line), file) && *count < 6) {
    // Пропускаем пустые строки и комментарии
    char *trimmed = line;
    while (*trimmed == ' ' || *trimmed == '\t')
      trimmed++; // Пропускаем пробелы в начале

    if (*trimmed == '\n' || *trimmed == '\0' || *trimmed == '#') {
      continue; // Пропускаем пустые строки и комментарии
    }

    // Убираем символ новой строки в конце
    char *newline = strchr(trimmed, '\n');
    if (newline)
      *newline = '\0';

    // Убираем комментарии в конце строки
    char *comment = strchr(trimmed, '#');
    if (comment)
      *comment = '\0';

    // Убираем пробелы в конце
    while (strlen(trimmed) > 0 && (trimmed[strlen(trimmed) - 1] == ' ' ||
                                   trimmed[strlen(trimmed) - 1] == '\t')) {
      trimmed[strlen(trimmed) - 1] = '\0';
    }

    if (strlen(trimmed) == 0)
      continue; // Пропускаем пустые строки после обработки

    // Парсим строку
    if (sscanf(trimmed, "%31s %d %hhx %255s", type_str,
               &configs[*count].xshut_pin, &configs[*count].i2c_addr,
               configs[*count].shm_name) == 4) {

      // Преобразуем строку в SensorType
      if (strcmp(type_str, "l1x") == 0) {
        configs[*count].type = SENSOR_VL53L1X;
      } else if (strcmp(type_str, "l5cx") == 0) {
        configs[*count].type = SENSOR_VL53L5CX;
      } else if (strcmp(type_str, "tcs") == 0) {
        configs[*count].type = SENSOR_TCS34725;
      } else {
        printf("Warning: Unknown sensor type '%s' in line %d, skipping\n",
               type_str, *count + 1);
        continue;
      }

      printf("Loaded config: %s pin=%d addr=0x%02X file=%s\n", type_str,
             configs[*count].xshut_pin, configs[*count].i2c_addr,
             configs[*count].shm_name);
      (*count)++;
    } else {
      printf("Warning: Invalid config line: %s\n", trimmed);
    }
  }

  fclose(file);
  printf("Total sensors configured: %d\n", *count);
  return 0;
}

int main(int argc, char *argv[]) {
  SensorConfig configs[6];
  int sensor_count = 0;
  uint8_t sensor_data[4];

  // Устанавливаем обработчик сигналов для корректного завершения
  signal(SIGINT, signal_handler);  // Ctrl+C
  signal(SIGTERM, signal_handler); // kill

  printf("Программа запущена. Нажмите Ctrl+C для остановки.\n");

  // read config file
  if (read_config("./sensors_config.txt", configs, &sensor_count) != 0) {
    fprintf(stderr, "Error: cant read config\n");
    return EXIT_FAILURE;
  }

  // sensors init
  if (init_gpio(configs, sensor_count) != 0) {
    fprintf(stderr, "Error: sensors initialization failed\n");
    return EXIT_FAILURE;
  }

  // Запускаем измерение для всех инициализированных датчиков
  for (int i = 0; i < sensor_count; i++) {
    if (configs[i].initialized) {
      switch (configs[i].type) {
      case SENSOR_VL53L1X:
        VL53L1X_StartRanging(configs[i].i2c_addr << 1);
        break;
      case SENSOR_VL53L5CX: {
        VL53L5CX_Configuration *config =
            (VL53L5CX_Configuration *)configs[i].sensor_config;
        if (config) {
          vl53l5cx_start_ranging(config);
        }
        break;
      }
      case SENSOR_TCS34725:
        // TODO: Запуск для TCS34725
        break;
      }
    }
  }

  // main loop
  while (running) {
    for (int i = 0; i < sensor_count; i++) {
      if (configs[i].initialized) {
        if (read_sensor_data(&configs[i], sensor_data) == 0) {
          // Для VL53L5CX данные уже записаны в shared memory в read_sensor_data
          if (configs[i].type == SENSOR_VL53L5CX) {
            printf("Sensor %d: Matrix data written to shared memory\n", i);
          } else {
            // Для других датчиков записываем одиночные данные
            uint16_t distance = (sensor_data[0] << 8) | sensor_data[1];
            uint8_t status = sensor_data[3]; // Берем младший байт статуса

            if (write_single_to_shm(&configs[i], distance, status) == 0) {
              printf("Sensor %d: Distance = %d mm, Status = %d\n", i, distance,
                     status);
            } else {
              printf("Error writing to shared memory for sensor %d\n", i);
            }
          }
        }
        else {
          printf("Error reading sensor data for sensor %d\n", i);
        }
      }
    }
    delay(50); // Пауза между циклами
  }

  // Корректное завершение
  stop_all_sensors(configs, sensor_count);
  printf("Программа завершена.\n");

  return EXIT_SUCCESS;
}