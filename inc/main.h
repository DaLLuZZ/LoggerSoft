#ifndef MAIN_H_
#define MAIN_H_

#define LOGGER_ID 29
//#define USE_BME280_SPI
#define USE_BME280_I2C

#ifdef USE_BME280_SPI
#include "bme280.h"
#endif
#ifdef USE_BME280_I2C
#include "bmp280.h"
#endif

#include <stdint.h>
#include "z_flash_W25QXXX.h"
#include "SX1278.h"

#define FLASH_CS_GPIO_Port GPIOB
#define FLASH_CS_Pin GPIO_PIN_12

#define BME280_CS_GPIO_Port GPIOA
#define BME_CS_Pin GPIO_PIN_8

#define SX_DIO0_GPIO_Port GPIOA
#define SX_DIO0_Pin GPIO_PIN_10

#define SX_NSS_GPIO_Port GPIOA
#define SX_NSS_Pin GPIO_PIN_4

#define SX_RESET_GPIO_Port GPIOA
#define SX_RESET_Pin GPIO_PIN_9

#ifdef USE_BME280_SPI
extern BME280_t bme;
extern BME280_Driver_t bme_drv;
extern struct spi_bus_data bme_spi;
extern BME280_Config_t bme_config;
extern BME280_Data_t bme_data;
#endif

extern SX1278_t SX1278;
extern SX1278_hw_t SX1278_hw;

#endif /* MAIN_H_ */
