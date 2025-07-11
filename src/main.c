#include "gd32e23x_hal.h"
#include "gd32e23x_hal_init.h"

#include "main.h"

const uint32_t logger_id __attribute__((section(".logger_id"), used)) = LOGGER_ID;
const uint32_t magic_signature __attribute__((section(".magic_signature"), used)) = MAGIC_SIGNATURE;

#ifdef USE_W25Q_EXT_FLASH

uint32_t flash_write_address = 0xFFFFFFFF;
uint8_t flash_failed = 0;

#endif

#ifdef USE_BME280_SPI

BME280_t bme;
BME280_Driver_t bme_drv;
struct spi_bus_data bme_spi;
BME280_Config_t bme_config;
BME280_Data_t bme_data;

#endif

#ifdef USE_BME280_I2C

BMP280_HandleTypedef bmp280;

#endif

#ifdef USE_RA_01_SENDER

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

#endif // USE_RA_01_SENDER


void uart_trns_cmpd(hal_uart_dev_struct *uart)
{

}

void uart_recv_byte(hal_uart_dev_struct *uart)
{
	hal_uart_transmit_interrupt(&uart1_info, "p\r\n", 3, uart_trns_cmpd);
}

void emptyFunc()
{

}

int main(void)
{
    msd_system_init();
    msd_clock_init();

    msd_gpio_init();
    msd_adc_init();
    msd_i2c1_init();
    msd_rtc_init();
    msd_spi0_init();
    msd_spi1_init();
    msd_timer0_init();
    msd_timer2_init();
    msd_usart1_init();

    char buffer[256];
    float dataToSend[4];
    uint32_t message_length;
    uint16_t adc_raw_value;

    // Start SPI, i2c, uart and adc

    hal_spi_start(&spi0_info);
    hal_spi_start(&spi1_info);
    hal_i2c_start(&i2c1_info);
    hal_uart_start(&uart1_info);
    hal_adc_start(&adc_info);

   /*
    * TRY ADC
    */

    hal_adc_regular_conversion_poll(&adc_info, 1000);
    adc_raw_value = hal_adc_regular_value_get(&adc_info);

#ifdef USE_BME280_SPI

    /*
     * INIT BME SPI
     */

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

    /* set all sensor's options */
    BME280_ConfigureAll(&bme, &bme_config);
    if (BME280_OK != res)
    {
        // do something when error occurred
    }

    // perform single read operation
	res = BME280_ReadAllLast(&bme, &bme_data);
	if (BME280_OK != res)
	{
		//do something when error occurred
	}

    message_length = sprintf(buffer, "H: %d.%d   T: %d.%d   P: %d.%d...\r\n", bme_data.humidity_int, bme_data.humidity_fract, bme_data.temp_int, bme_data.temp_fract, bme_data.pressure_int, bme_data.pressure_fract);
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

#endif // USE_BME280_SPI

#ifdef USE_BME280_I2C

    float pressure, temperature, humidity;

    /*
     * INIT BME I2C
     */
	bmp280_init_default_params(&bmp280.params);
	bmp280.params.oversampling_humidity = BMP280_ULTRA_HIGH_RES;
	bmp280.params.oversampling_temperature = BMP280_ULTRA_HIGH_RES;
	bmp280.params.oversampling_pressure = BMP280_ULTRA_LOW_POWER;
	bmp280.addr = BMP280_I2C_ADDRESS_0;
	bmp280.i2c = &i2c1_info;

	if (!bmp280_init(&bmp280, &bmp280.params))
	{
		// do smth when error occurred
	}

	message_length =  sprintf(buffer, "found %s (%x)\r\n", bmp280.id == BME280_CHIP_ID ? "BME280" : "BMP280", bmp280.id);

	hal_basetick_delay_ms(300);

	// perform single read operation
	if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity))
	{
		message_length = sprintf(buffer, "Temperature/pressure reading failed\r\n");
		hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);
	}

#endif // USE_BME280_I2C

#ifdef USE_W25Q_EXT_FLASH

	/*
	 * INIT EXTERNAL FLASH
	 */

	Flash_TestAvailability();
	Flash_Reset();
	Flash_Init();

	/*
	// Temp commented out due to wrong 1st byte reading
	if (Flash_Init() == 0)
	{
	    message_length = sprintf(buffer, "Something wrong with W25Q...\r\n");
	    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);
		//do something when error occurred
	}
	*/

	struct txPack mem_pack;
	for (uint32_t i = 0; i < 4 * 1024 * 1024; i += sizeof(mem_pack))
	{
		Flash_Read(i, (uint8_t*)&mem_pack.device_id, sizeof(mem_pack));
		for (uint8_t j = 4; j < sizeof(mem_pack); j++)
		{
			if (((uint8_t*)(&mem_pack.device_id))[j] != 0xFF)
			{
				break;
			}

			if (j == sizeof(mem_pack) - 1)
			{
				flash_write_address = i;
			}
		}

		if (flash_write_address != 0xFFFFFFFF)
		{
			break;
		}
		else if (i >= 4 * 1024 * 1024 - sizeof(mem_pack))
		{
			flash_failed = 1;
		}
	}

#endif // USE_W25Q_EXT_FLASH

#ifdef USE_RA_01_SENDER
	/*
	 * INIT LORA MODULE RA-01
	 */

	struct txPack pack;
	pack.device_id = *((uint32_t*)0x0800FFF8);
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

	SX1278_init(&SX1278, 433000000, SX1278_POWER_20DBM, SX1278_LORA_SF_12,
			SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_8, SX1278_LORA_CRC_EN, 24);

#ifdef USE_BME280_SPI

	pack.humidity = combineToFloat(bme_data.humidity_int, bme_data.humidity_fract);
	pack.temperature = combineToFloat(bme_data.temp_int, bme_data.temp_fract);
	pack.pressure = combineToFloat(bme_data.pressure_int, bme_data.pressure_fract);

#endif // USE_BME280_SPI

#ifdef USE_BME280_I2C

	pack.humidity = humidity;
	pack.temperature = temperature;
	pack.pressure = pressure;

#endif // USE_BME280_I2C

	pack.voltage = (2 * adc_raw_value) * 0.000814f; // 2 mul because of 1/1 R-div

	uint32_t ret1 = SX1278_LoRaEntryTx(&SX1278, sizeof(pack), 50);
	uint32_t ret2 = SX1278_LoRaTxPacket(&SX1278, (uint8_t*)(&pack), sizeof(pack), 2500);

#ifdef USE_W25Q_EXT_FLASH

	Flash_Write(flash_write_address, (uint8_t*)&pack.device_id, sizeof(pack));
	flash_write_address += sizeof(pack);

#endif // USE_W25Q_EXT_FLASH

	pack.msg_id = pack.msg_id + 1;

#endif // USE_RA_01_SENDER

    hal_basetick_delay_ms(5000);

#ifdef USE_TEST_PACKET_SPAMMING

    if (flash_failed)
    {
    	pack.voltage = -3.3f;
    }

    for (int i = 0; i < 36; i++)
    {
		uint32_t ret1 = SX1278_LoRaEntryTx(&SX1278, sizeof(pack), 50);
		uint32_t ret2 = SX1278_LoRaTxPacket(&SX1278, (uint8_t*)(&pack), sizeof(pack), 2500);

#ifdef USE_W25Q_EXT_FLASH

		Flash_Write(flash_write_address, (uint8_t*)&pack.device_id, sizeof(pack));
		flash_write_address += sizeof(pack);

#endif // USE_W25Q_EXT_FLASH

		pack.msg_id = pack.msg_id + 1;
		hal_basetick_delay_ms(12500);
    }

#endif // USE_TEST_PACKET_SPAMMING

#ifdef USE_MCU_DEEPSLEEP_MODE

    // INIT ALARM FOR RTC

    hal_rtc_alarm_struct rtc_alarm_time;
    hal_rtc_struct_init(HAL_RTC_ALARM_STRUCT, &rtc_alarm_time);
    rtc_alarm_time.rtc_alarm_mask = HAL_RTC_ALARM_DATE_MASK | HAL_RTC_ALARM_HOUR_MASK | HAL_RTC_ALARM_MINUTE_MASK;
    rtc_alarm_time.rtc_weekday_or_date = RTC_ALARM_DATE_SELECTED;
    rtc_alarm_time.rtc_alarm_second = 0x50;
    hal_rtc_alarm_config(&rtc_alarm_time);

#endif // USE_MCU_DEEPSLEEP_MODE

    while (1)
    {

#ifdef DEINIT_ALL_PERIPH_DURING_MCU_SLEEP

		// reinit all periph
		msd_gpio_init();
		msd_adc_init();
		msd_spi0_init();
		msd_spi1_init();
		msd_timer0_init();
		msd_timer2_init();
		msd_usart1_init();
		msd_i2c1_init();

#endif // DEINIT_ALL_PERIPH_DURING_MCU_SLEEP

		// Wake up ADC, get value and sleep
		hal_adc_start(&adc_info);
		hal_adc_regular_conversion_poll(&adc_info, 1000);
		adc_raw_value = hal_adc_regular_value_get(&adc_info);
		hal_adc_stop(&adc_info);

#ifdef USE_BME280_SPI

		// Wake up BME's SPI, wake up bme, read values, sleep bme, stop BME's SPI
		hal_spi_start(bme_spi.spi_handle);
		bme_config.mode = BME280_NORMALMODE;
		BME280_ConfigureAll(&bme, &bme_config);
		BME280_ReadAllLast(&bme, &bme_data);
		bme_config.mode = BME280_SLEEPMODE;
		BME280_ConfigureAll(&bme, &bme_config);
		hal_spi_stop(bme_spi.spi_handle);

#endif

#ifdef USE_BME280_I2C

		// Wake up BME's I2C, wake up bme, read values, sleep bme, stop BME's I2C
		hal_i2c_start(bmp280.i2c);
		bmp280_wakeup(&bmp280);
		bmp280_read_float(&bmp280, &temperature, &pressure, &humidity);
		bmp280_sleep(&bmp280);
		hal_i2c_stop(bmp280.i2c);

#endif

#ifdef USE_RA_01_SENDER

		// Fill pack with values before sending
#ifdef USE_BME280_SPI
		pack.humidity = combineToFloat(bme_data.humidity_int, bme_data.humidity_fract);
		pack.temperature = combineToFloat(bme_data.temp_int, bme_data.temp_fract);
		pack.pressure = combineToFloat(bme_data.pressure_int, bme_data.pressure_fract);
#endif // USE_BME280_SPI
#ifdef USE_BME280_I2C
		pack.humidity = humidity;
		pack.temperature = temperature;
		pack.pressure = pressure;
#endif // USE_BME280_I2C
		pack.voltage = (2 * adc_raw_value) * 0.000814f; // 2 mul because of 1/1 R-div

		// START Ra-01 SPI, wake up Ra-01, send packet, sleep Ra-01, stop Ra-01 SPI
		hal_spi_start(SX1278_hw.spi);
		SX1278_standby(&SX1278);
		SX1278_LoRaEntryTx(&SX1278, sizeof(pack), 50);
		SX1278_LoRaTxPacket(&SX1278, (uint8_t*)(&pack), sizeof(pack), 2500);
		SX1278_sleep(&SX1278);
		hal_spi_stop(SX1278_hw.spi);

#ifdef USE_W25Q_EXT_FLASH

		// START SPI, power up flash ic, save packet data to flash, power down flash ic, stop SPI
		hal_spi_start(&FLASH_SPI);
		Flash_PowerUp();
		Flash_Write(flash_write_address, (uint8_t*)&pack.device_id, sizeof(pack));
		flash_write_address += sizeof(pack);
		Flash_PowerDown();
		hal_spi_stop(&FLASH_SPI);

#endif // USE_W25Q_EXT_FLASH

		pack.msg_id += 1; // Increment msg_id for next message

#endif // USE_RA_01_SENDER

#ifdef DEINIT_ALL_PERIPH_DURING_MCU_SLEEP

		// deinit all periph
		msd_gpio_deinit();
		msd_adc_deinit();
		msd_spi0_deinit();
		msd_spi1_deinit();
		msd_timer0_deinit();
		msd_timer2_deinit();
		msd_usart1_deinit();
		msd_i2c1_deinit();

#endif // DEINIT_ALL_PERIPH_DURING_MCU_SLEEP

#ifdef USE_MCU_DEEPSLEEP_MODE

		rtc_parameter_struct rtc_initpara_struct;
		uint8_t temp;
		// rtc alarm config and sleep until alarm, then wake up and disable alarm
		// IMPORTANT NOTE: state of alarm register must change between 2 alarms
		hal_rtc_alarm_disable();
		hal_nvic_periph_irq_disable(RTC_IRQn);
		rtc_interrupt_disable(RTC_INT_ALARM);
		rtc_alarm_time.rtc_alarm_mask = HAL_RTC_ALARM_DATE_MASK | HAL_RTC_ALARM_HOUR_MASK;
		rtc_current_time_get(&rtc_initpara_struct);
		temp = rtc_bcd_2_normal(rtc_initpara_struct.rtc_minute) + SLEEP_MINUTES;
		if (temp >= 60)
		{
			temp -= 60;
		}
		rtc_alarm_time.rtc_alarm_minute = rtc_normal_2_bcd(temp);
		rtc_alarm_time.rtc_alarm_second = 0x00;
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

#endif // USE_MCU_DEEPSLEEP_MODE

#ifndef USE_MCU_DEEPSLEEP_MODE

		hal_basetick_delay_ms(SLEEP_MINUTES * 1000 * 60);

#endif // USE_MCU_DEEPSLEEP_MODE

    }

    // transmit message uart
    message_length = sprintf(buffer, "Hello there!\r\n");
    hal_uart_transmit_poll(&uart1_info, buffer, message_length, 1000);

    // try to receive uart
    hal_uart_receive_interrupt(&uart1_info, buffer, 1, uart_recv_byte);

}
