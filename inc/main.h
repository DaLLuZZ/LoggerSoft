#ifndef MAIN_H_
#define MAIN_H_

#define LOGGER_ID 29
//#define USE_BME280_SPI
#define USE_BME280_I2C
//#define USE_W25Q_EXT_FLASH

#include <stdint.h>
#include "SX1278.h"

#ifdef USE_BME280_SPI
#include "bme280.h"
#endif
#ifdef USE_BME280_I2C
#include "bmp280.h"
#endif

#ifdef USE_W25Q_EXT_FLASH
#define FLASH_CS_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_12
#include "z_flash_W25QXXX.h"
#endif

#define SX_DIO0_GPIO_Port GPIOA
#define SX_DIO0_Pin GPIO_PIN_10

#define SX_NSS_GPIO_Port GPIOA
#define SX_NSS_Pin GPIO_PIN_4

#define SX_RESET_GPIO_Port GPIOA
#define SX_RESET_Pin GPIO_PIN_9

#ifdef USE_BME280_SPI
#define BME280_CS_GPIO_Port GPIOA
#define BME_CS_Pin GPIO_PIN_8

extern BME280_t bme;
extern BME280_Driver_t bme_drv;
extern struct spi_bus_data bme_spi;
extern BME280_Config_t bme_config;
extern BME280_Data_t bme_data;
#endif

#ifdef USE_BME280_I2C
extern BMP280_HandleTypedef bmp280;
#endif

extern SX1278_t SX1278;
extern SX1278_hw_t SX1278_hw;

#endif /* MAIN_H_ */
