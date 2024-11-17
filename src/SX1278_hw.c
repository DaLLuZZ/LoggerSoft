/**
 * Author Wojciech Domski <Wojciech.Domski@gmail.com>
 * www: www.Domski.pl
 *
 * Hardware layer for SX1278 LoRa module
 */

#include "SX1278_hw.h"
#include <string.h>

#include "gd32e23x_hal_gpio.h"
#include "gd32e23x_hal_spi.h"

void SX1278_hw_init(SX1278_hw_t * hw) {
	SX1278_hw_SetNSS(hw, 1);
	hal_gpio_bit_write(hw->reset.port, hw->reset.pin, SET);
}

void SX1278_hw_SetNSS(SX1278_hw_t * hw, int value) {
	hal_gpio_bit_write(hw->nss.port, hw->nss.pin,
			(value == 1) ? SET : RESET);
}

void SX1278_hw_Reset(SX1278_hw_t * hw) {
	SX1278_hw_SetNSS(hw, 1);
	hal_gpio_bit_write(hw->reset.port, hw->reset.pin, RESET);

	SX1278_hw_DelayMs(1); // at least 100 us

	hal_gpio_bit_write(hw->reset.port, hw->reset.pin, SET);

	SX1278_hw_DelayMs(10); // delay should be 5 ms minimum according to chip manual
}

void SX1278_hw_SPICommand(SX1278_hw_t * hw, uint8_t cmd) {
	SX1278_hw_SetNSS(hw, 0);
	hal_spi_transmit_poll(hw->spi, &cmd, 1, 1000);
	while (hw->spi->state != HAL_SPI_STATE_READY) {
	}
}

uint8_t SX1278_hw_SPIReadByte(SX1278_hw_t * hw) {
	uint8_t txByte = 0x00;
	uint8_t rxByte = 0x00;

	SX1278_hw_SetNSS(hw, 0);
	hal_spi_transmit_receive_poll(hw->spi, &txByte, &rxByte, 1, 1000);
	while (hw->spi->state != HAL_SPI_STATE_READY) {
	}
	return rxByte;
}

void SX1278_hw_DelayMs(uint32_t msec) {
	hal_basetick_delay_ms(msec);
}

int SX1278_hw_GetDIO0(SX1278_hw_t * hw) {
	return (hal_gpio_output_bit_get(hw->dio0.port, hw->dio0.pin) == SET);
}

