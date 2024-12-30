#ifndef MAIN_H_
#define MAIN_H_

/*
 * MAGIC_SIGNATURE - used for validation in auto firmware flasher (stored in 0x800FFFC, last 32bit of internal flash)
 * LOGGER_ID - value stored in 0x800FFF8
 * SLEEP_MINUTES - integer value in [1...60] including both bounds
 */
#define MAGIC_SIGNATURE 0xDEADBEEF
#define LOGGER_ID 0xFFFFFFFF
#define SLEEP_MINUTES 15
#define PACKET_DUPLICATION_COUNT 1

//#define USE_EXTERNAL_LXTAL

//#define USE_BME280_SPI
#define USE_BME280_I2C
//#define USE_W25Q_EXT_FLASH
#define USE_RA_01_SENDER
#define USE_MCU_DEEPSLEEP_MODE
#define DEINIT_ALL_PERIPH_DURING_MCU_SLEEP
#define USE_TEST_PACKET_SPAMMING

#include <stdint.h>
#include <stdio.h>

#ifdef USE_BME280_SPI
#include "bme280.h"
#endif // USE_BME280_SPI
#ifdef USE_BME280_I2C
#include "bmp280.h"
#endif // USE_BME280_I2C

#ifdef USE_W25Q_EXT_FLASH

#include "z_flash_W25QXXX.h"

#define FLASH_CS_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_12

#endif // USE_W25Q_EXT_FLASH

#ifdef USE_RA_01_SENDER

#include "SX1278.h"

#define SX_DIO0_GPIO_Port GPIOA
#define SX_DIO0_Pin GPIO_PIN_10

#define SX_NSS_GPIO_Port GPIOB
#define SX_NSS_Pin GPIO_PIN_12

#define SX_RESET_GPIO_Port GPIOA
#define SX_RESET_Pin GPIO_PIN_9

extern SX1278_t SX1278;
extern SX1278_hw_t SX1278_hw;

#endif // USE_RA_01_SENDER

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

#endif /* MAIN_H_ */
