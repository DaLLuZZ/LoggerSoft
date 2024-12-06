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

struct txPack {
	uint32_t device_id;
	uint32_t msg_id;
	float humidity;
	float temperature;
	float pressure;
	float voltage;
};


hal_uart_user_cb uart_trns_cmpd(hal_uart_dev_struct *uart)
{

}

hal_uart_user_cb uart_recv_byte(hal_uart_dev_struct *uart)
{
	hal_uart_transmit_interrupt(&uart1_info, "p\r\n", 3, uart_trns_cmpd);
}


float combineToFloat(int32_t integerPart, int32_t fractionalPart)
{
    int32_t fractionalDigits = 0;
    int32_t temp = fractionalPart;

    while (temp > 0) {
        temp /= 10;
        fractionalDigits++;
    }

    float result = integerPart + fractionalPart / pow(10, fractionalDigits);

    return result;
}

hal_rtc_irq_handle_cb emptyFunc(void* ptr)
{

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
    float dataToSend[4];
    uint32_t message_length;
    uint16_t freevalue;

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

	struct txPack pack;
	pack.device_id = 27;
	pack.msg_id = 1;
	pack.humidity = 0.0f;
	pack.temperature = 0.0f;
	pack.pressure = 0.0f;
	pack.voltage = 0.0f;

	//initialize LoRa module
	SX1278_hw.dio0.port = SX_DIO0_GPIO_Port;
	SX1278_hw.dio0.pin = SX_DIO0_Pin;
	SX1278_hw.nss.port = SX_NSS_GPIO_Port;
	SX1278_hw.nss.pin = SX_NSS_Pin;
	SX1278_hw.reset.port = SX_RESET_GPIO_Port;
	SX1278_hw.reset.pin = SX_RESET_Pin;
	SX1278_hw.spi = &spi1_info;
	SX1278.hw = &SX1278_hw;

	SX1278_init(&SX1278, 440000000, SX1278_POWER_20DBM, SX1278_LORA_SF_12,
			SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_8, SX1278_LORA_CRC_EN, 32);

	// Try to send message
	//message_length = sprintf(buffer, "bu! ispugalsya? ne boisya!");
	//uint32_t ret1 = SX1278_LoRaEntryTx(&SX1278, message_length + 1, 2000);
	//uint32_t ret2 = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) buffer, message_length + 1, 20000);

	pack.humidity = combineToFloat(bme_data.humidity_int, bme_data.humidity_fract);
	pack.temperature = combineToFloat(bme_data.temp_int, bme_data.temp_fract);
	pack.pressure = combineToFloat(bme_data.pressure_int, bme_data.pressure_fract);
	pack.voltage = (2 * freevalue) * 0.000814f; // 2 mul because of 1/1 R-div

	uint32_t ret1 = SX1278_LoRaEntryTx(&SX1278, sizeof(pack), 1000);
	uint32_t ret2 = SX1278_LoRaTxPacket(&SX1278, (uint8_t*)(&pack), sizeof(pack), 6000);

	pack.msg_id = pack.msg_id + 1;
	hal_basetick_delay_ms(100);

    // Have a timeout

    message_length = sprintf(buffer, "Delay 10 sec, then sleep mode of ra-01\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    hal_basetick_delay_ms(10000);

    // INIT ALARM FOR RTC

    hal_rtc_alarm_struct rtc_alarm_time;
    hal_rtc_struct_init(HAL_RTC_ALARM_STRUCT, &rtc_alarm_time);
    rtc_alarm_time.rtc_alarm_mask = HAL_RTC_ALARM_DATE_MASK | HAL_RTC_ALARM_HOUR_MASK | HAL_RTC_ALARM_MINUTE_MASK;
    rtc_alarm_time.rtc_weekday_or_date = RTC_ALARM_DATE_SELECTED;
    rtc_alarm_time.rtc_alarm_second = 0x50;
    hal_rtc_alarm_config(&rtc_alarm_time);

    // HERE STARTS REGULAR CODE

    while (1)
    {

		// init all periph
		msd_gpio_init();
		msd_adc_init();
		msd_crc_init();
		msd_spi0_init();
		msd_spi1_init();
		msd_timer0_init();
		msd_timer2_init();
		msd_usart1_init();

		// Wake up ADC, get value and sleep
		hal_adc_start(&adc_info);
		hal_adc_regular_conversion_poll(&adc_info, 1000);
		freevalue = hal_adc_regular_value_get(&adc_info);
		hal_adc_stop(&adc_info);

		// Wake up BME's SPI, wake up bme, read values, sleep bme, stop BME's SPI
		hal_spi_start(bme_spi.spi_handle);
		bme_config.mode = BME280_NORMALMODE;
		BME280_ConfigureAll(&bme, &bme_config);
		BME280_ReadAllLast(&bme, &bme_data);
		bme_config.mode = BME280_SLEEPMODE;
		BME280_ConfigureAll(&bme, &bme_config);
		hal_spi_stop(bme_spi.spi_handle);

		// Fill pack with values before sending
		pack.humidity = combineToFloat(bme_data.humidity_int, bme_data.humidity_fract);
		pack.temperature = combineToFloat(bme_data.temp_int, bme_data.temp_fract);
		pack.pressure = combineToFloat(bme_data.pressure_int, bme_data.pressure_fract);
		pack.voltage = (2 * freevalue) * 0.000814f; // 2 mul because of 1/1 R-div

		// START Ra-01 SPI, wake up Ra-01, send packet, sleep Ra-01, stop Ra-01 SPI
		hal_spi_start(SX1278_hw.spi);
		SX1278_standby(&SX1278);
		SX1278_LoRaEntryTx(&SX1278, sizeof(pack), 1000);
		SX1278_LoRaTxPacket(&SX1278, (uint8_t*)(&pack), sizeof(pack), 6000);
		SX1278_sleep(&SX1278);
		hal_spi_stop(SX1278_hw.spi);
		pack.msg_id += 1; // Increment msg_id for next message

		// deinit all periph
		msd_gpio_deinit();
		msd_adc_deinit();
		msd_crc_deinit();
		msd_spi0_deinit();
		msd_spi1_deinit();
		msd_timer0_deinit();
		msd_timer2_deinit();
		msd_usart1_deinit();

		rtc_parameter_struct rtc_initpara_struct;
		// rtc alarm config and sleep until alarm, then wake up and disable alarm
		// IMPORTANT NOTE: state of alarm register must change between 2 alarms
		hal_rtc_alarm_disable();
		hal_nvic_periph_irq_disable(RTC_IRQn);
		rtc_interrupt_disable(RTC_INT_ALARM);
		rtc_alarm_time.rtc_alarm_mask = HAL_RTC_ALARM_DATE_MASK | HAL_RTC_ALARM_HOUR_MASK | HAL_RTC_ALARM_MINUTE_MASK;
		rtc_alarm_time.rtc_alarm_second = 0x50;
		hal_rtc_alarm_config(&rtc_alarm_time);
		rtc_alarm_subsecond_config(RTC_MASKSSC_0_14, 0);
		rtc_flag_clear(RTC_FLAG_ALARM0);
		hal_rtc_alarm_enable_interrupt(emptyFunc);
		rtc_interrupt_enable(RTC_INT_ALARM);
		hal_nvic_periph_irq_enable(RTC_IRQn, 2);
		hal_rtc_alarm_enable();

		hal_basetick_suspend();
		hal_pmu_to_deepsleepmode(HAL_PMU_LDO_LOWPOWER, HAL_WFI_CMD);
		hal_basetick_resume();

		hal_rtc_alarm_disable();
		hal_nvic_periph_irq_disable(RTC_IRQn);
		rtc_interrupt_disable(RTC_INT_ALARM);
		rtc_flag_clear(RTC_FLAG_ALARM0);
		rtc_current_time_get(&rtc_initpara_struct);

		hal_basetick_delay_ms(10000);

    }

    // HERE ENDS REGULAR CODE

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
