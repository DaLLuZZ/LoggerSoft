/*
    \file  main.c
*/
/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/
#include "gd32e23x_hal.h"
#include "gd32e23x_hal_init.h"
/* user code [global 0] begin */





#include "main.h"


BME280_t bme;
BME280_Driver_t bme_drv;
struct spi_bus_data bme_spi;
BME280_Config_t bme_config;
BME280_Data_t bme_data;

SX1278_t SX1278;
SX1278_hw_t SX1278_hw;


hal_uart_user_cb uart_trns_cmpd(hal_uart_dev_struct *uart)
{

}

hal_uart_user_cb uart_recv_byte(hal_uart_dev_struct *uart)
{
	hal_uart_transmit_interrupt(&uart1_info, "p\r\n", 3, uart_trns_cmpd);
}





/* user code [global 0] end */

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* user code [local 0] begin */

    /* user code [local 0] end */

    msd_system_init();
    msd_clock_init();
    /* user code [local 1] begin */

    /* user code [local 1] end */
    msd_gpio_init();
    msd_adc_init();
    msd_crc_init();
    msd_dbg_init();
    msd_rtc_init();
    msd_spi0_init();
    msd_spi1_init();
    msd_timer0_init();
    msd_timer2_init();
    msd_usart1_init();

    /* user code [local 2] begin */

    char buffer[256];
    uint32_t message_length;
    uint32_t freevalue;

    // Start SPI

    hal_spi_start(&spi0_info);
    hal_spi_start(&spi1_info);
    hal_uart_start(&uart1_info);
    hal_adc_start(&adc_info);


   /*
    * TRY ADC
    */

    message_length = sprintf(buffer, "Trying ADC...\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    hal_adc_regular_conversion_poll(&adc_info, 1000);
    freevalue = hal_adc_regular_value_get(&adc_info);
    message_length = sprintf(buffer, "ADC raw value: %d; Voltage: %.2f Volts\r\n", freevalue, freevalue * 0.00080586);
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    /*
     * INIT BME
     */
    message_length = sprintf(buffer, "Init BME...\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    // set spi data
    bme_spi.spi_handle = &spi0_info;
    bme_spi.NCS_gpio = BME280_CS_GPIO_Port;
    bme_spi.NCS_pin = BME_CS_Pin;

    /* fill the driver */
    bme_drv.read = bme280_read_platform_spec;
    bme_drv.write = bme280_write_platform_spec;
    bme_drv.delay = bme280_delay_platform_spec;
    bme_drv.env_spec_data = &bme_spi;

    int8_t res = BME280_Init(&bme, &bme_drv);

    if (BME280_OK != res)
    {
        message_length = sprintf(buffer, "Init BME failed...\r\n");
        hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);
        //do something when error occurred
    }

    // BME options
    bme_config.oversampling_h = BME280_OVERSAMPLING_X4;
    bme_config.oversampling_p = BME280_OVERSAMPLING_X2;
    bme_config.oversampling_t = BME280_OVERSAMPLING_X2;
    bme_config.filter = BME280_FILTER_2;
    bme_config.spi3w_enable = 0;
    bme_config.t_stby = BME280_STBY_500MS;
    bme_config.mode = BME280_NORMALMODE;

    message_length = sprintf(buffer, "Configuring BME...\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    /* set all sensor's options */
    BME280_ConfigureAll(&bme, &bme_config);
    if (BME280_OK != res)
    {
        message_length = sprintf(buffer, "Configuring BME failed...\r\n");
        hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);
        // do something when error occurred
    }

    message_length = sprintf(buffer, "Reading BME...\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    // perform single read operation
	res = BME280_ReadAllLast(&bme, &bme_data);
	if (BME280_OK != res)
	{
        message_length = sprintf(buffer, "Reading BME failed...\r\n");
        hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);
		//do something when error occurred
	}

    message_length = sprintf(buffer, "H: %d.%d   T: %d.%d   P: %d.%d...\r\n", bme_data.humidity_int, bme_data.humidity_fract, bme_data.temp_int, bme_data.temp_fract, bme_data.pressure_int, bme_data.pressure_fract);
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);


	/*
	 * INIT EXTERNAL FLASH
	 */

	if (Flash_TestAvailability() == 0)
	{
	    message_length = sprintf(buffer, "Something wrong with W25Q...\r\n");
	    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);
		//do something when error occurred
	}


	/*
	 * INIT LORA MODULE RA-01
	 */

	//initialize LoRa module
	SX1278_hw.dio0.port = SX_DIO0_GPIO_Port;
	SX1278_hw.dio0.pin = SX_DIO0_Pin;
	SX1278_hw.nss.port = SX_NSS_GPIO_Port;
	SX1278_hw.nss.pin = SX_NSS_Pin;
	SX1278_hw.reset.port = SX_RESET_GPIO_Port;
	SX1278_hw.reset.pin = SX_RESET_Pin;
	SX1278_hw.spi = &spi0_info;
	SX1278.hw = &SX1278_hw;

	SX1278_init(&SX1278, 440000000, SX1278_POWER_20DBM, SX1278_LORA_SF_12,
			SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_8, SX1278_LORA_CRC_EN, 32);

while (1)
{
	// Try to send message
	message_length = sprintf(buffer, "bu! ispugalsya? ne boisya!");
	//message_length = sprintf(buffer, "AAAAA SPASITE NAS POZHALUYSTA Ku-ku, we have %d.%d degrees here!\r\n\r\n", bme_data.temp_int, bme_data.temp_fract);
	uint32_t ret1 = SX1278_LoRaEntryTx(&SX1278, message_length + 1, 2000);
	uint32_t ret2 = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) buffer, message_length + 1, 20000);

	hal_basetick_delay_ms(100);

    message_length = sprintf(buffer, "Entry: %d; Send: %d\r\n", ret1, ret2);
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);
}

//hal_basetick_delay_ms(500000);

    // Have a timeout

    message_length = sprintf(buffer, "Delay 10 sec, then sleep mode of ra-01\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    hal_basetick_delay_ms(10000);

	// Try to make ra-01 sleep
	SX1278_sleep(&SX1278);

    message_length = sprintf(buffer, "Ra-01 sleeping rn... Delay 15 sec, then sleep of mcu\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    // try to receive uart
    hal_uart_receive_interrupt(&uart1_info, buffer, 1, uart_recv_byte);

    hal_basetick_delay_ms(15000);

    message_length = sprintf(buffer, "good night!\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    // rtc alarm config
    // rtc alarm enable OR hal_rtc_alarm_enable_interrupt
    // rtc_normal_2_bcd

    // try to sleep
    hal_basetick_suspend();
    hal_pmu_to_deepsleepmode(HAL_PMU_LDO_LOWPOWER, HAL_WFI_CMD);
    //hal_basetick_resume();

    message_length = sprintf(buffer, "good morning!\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    // try to receive
    hal_uart_receive_interrupt(&uart1_info, buffer, 1, uart_recv_byte);


    /* user code [local 2] end */

    while(1){
        /* user code [local 3] begin */

        /* user code [local 3] end */
    }
}
/* user code [global 1] begin */








/* user code [global 1] end */	
