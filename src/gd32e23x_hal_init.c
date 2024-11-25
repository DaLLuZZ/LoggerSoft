/*
    \file  gd32e23x_hal_init.c
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
#include "gd32e23x_hal_init.h"
/* user code [global 0] begin */

/* user code [global 0] end */
hal_adc_dev_struct adc_info;
hal_spi_dev_struct spi0_info;
hal_spi_dev_struct spi1_info;
hal_timer_dev_struct timer0_info;
hal_timer_dev_struct timer2_info;
hal_uart_dev_struct uart1_info;

void msd_system_init(void)
{
    /* user code [system_init local 0] begin */
    /* user code [system_init local 0] end */
    hal_fmc_prefetch_enable();
    hal_rcu_periph_clk_enable(RCU_CFGCMP);
    hal_nvic_periph_irq_enable(NonMaskableInt_IRQn, 0);
    hal_nvic_periph_irq_enable(HardFault_IRQn, 0);
    hal_nvic_periph_irq_enable(SVCall_IRQn, 0);
    hal_nvic_periph_irq_enable(PendSV_IRQn, 0);
    hal_nvic_periph_irq_enable(SysTick_IRQn, 0);
    hal_basetick_init(HAL_BASETICK_SOURCE_SYSTICK);

    /* user code [system_init local 1] begin */
    /* user code [system_init local 1] end */
}

void msd_clock_init(void)
{
    /* user code [clock_init local 0] begin */
    /* user code [clock_init local 0] end */
    hal_rcu_clk_struct rcu_clk_parameter;
    hal_rcu_osci_struct rcu_osci_parameter;
    hal_rcu_periphclk_struct rcu_periphclk_parameter;

    hal_rcu_struct_init(HAL_RCU_CLK_STRUCT, &rcu_clk_parameter);
    hal_rcu_struct_init(HAL_RCU_OSCI_STRUCT, &rcu_osci_parameter);
    hal_rcu_struct_init(HAL_RCU_PERIPHCLK_STRUCT, &rcu_periphclk_parameter);

    rcu_osci_parameter.irc8m.need_configure = ENABLE;
    rcu_osci_parameter.irc8m.state = RCU_OSC_ON;
    rcu_osci_parameter.irc8m.adjust_value = 0;
    rcu_osci_parameter.irc40k.need_configure = ENABLE;
    rcu_osci_parameter.irc40k.state = RCU_OSC_ON;
    if(HAL_ERR_NONE != hal_rcu_osci_config(&rcu_osci_parameter)){
        while(1);
    }

    rcu_clk_parameter.clock_type = RCU_CLKTYPE_SYSCLK | RCU_CLKTYPE_AHBCLK | RCU_CLKTYPE_APB1CLK | RCU_CLKTYPE_APB2CLK;
    rcu_clk_parameter.sysclk_source = RCU_SYSCLK_SRC_IRC8M;
    rcu_clk_parameter.ahbclk_divider = RCU_SYSCLK_AHBDIV1;
    rcu_clk_parameter.apb1clk_divider = RCU_AHBCLK_APB1DIV1;
    rcu_clk_parameter.apb2clk_divider = RCU_AHBCLK_APB2DIV1;
    if(HAL_ERR_NONE != hal_rcu_clock_config(&rcu_clk_parameter, WS_WSCNT_2)){
        while(1);
    }

    rcu_periphclk_parameter.periph_clock_type = RCU_PERIPH_CLKTYPE_RTC;
    rcu_periphclk_parameter.rtc_clock_source = RCU_RTC_CLKSRC_IRC40K;
    if(HAL_ERR_NONE != hal_rcu_periph_clock_config(&rcu_periphclk_parameter)){
        while(1);
    }

    rcu_periphclk_parameter.periph_clock_type = RCU_PERIPH_CLKTYPE_ADC;
    rcu_periphclk_parameter.adc_clock_source = RCU_ADCCK_APB2_DIV2;
    if(HAL_ERR_NONE != hal_rcu_periph_clock_config(&rcu_periphclk_parameter)){
        while(1);
    }

    /* user code [clock_init local 1] begin */
    /* user code [clock_init local 1] end */
}

void msd_gpio_init(void)
{
    /* user code [gpio_init local 0] begin */
    /* user code [gpio_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;

    hal_rcu_periph_clk_enable(RCU_GPIOC);
    hal_rcu_periph_clk_enable(RCU_GPIOF);
    hal_rcu_periph_clk_enable(RCU_GPIOB);
    hal_rcu_periph_clk_enable(RCU_GPIOA);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOF, GPIO_PIN_7, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_3, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_2, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOF, GPIO_PIN_6, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_5, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_4, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_7, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_6, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_9, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_8, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_0, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_1, &gpio_init_parameter);

    hal_gpio_bit_set(GPIOA, GPIO_PIN_4);
    gpio_init_parameter.mode = HAL_GPIO_MODE_OUTPUT_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_4, &gpio_init_parameter);

    hal_gpio_bit_set(GPIOA, GPIO_PIN_8);
    gpio_init_parameter.mode = HAL_GPIO_MODE_OUTPUT_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_8, &gpio_init_parameter);

    hal_gpio_bit_set(GPIOA, GPIO_PIN_9);
    gpio_init_parameter.mode = HAL_GPIO_MODE_OUTPUT_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_9, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOC, GPIO_PIN_13, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_15, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOC, GPIO_PIN_15, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOC, GPIO_PIN_14, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_INPUT;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_11, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_10, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_INPUT;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_10, &gpio_init_parameter);

    hal_gpio_bit_set(GPIOB, GPIO_PIN_12);
    gpio_init_parameter.mode = HAL_GPIO_MODE_OUTPUT_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_12, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_11, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_12, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOF, GPIO_PIN_1, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOF, GPIO_PIN_0, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_2MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_0, &gpio_init_parameter);

    /* user code [gpio_init local 1] begin */
    /* user code [gpio_init local 1] end */
}

void msd_gpio_deinit(void)
{
    /* user code [gpio_deinit local 0] begin */
    /* user code [gpio_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_GPIOC);
    hal_rcu_periph_clk_disable(RCU_GPIOF);
    hal_rcu_periph_clk_disable(RCU_GPIOB);
    hal_rcu_periph_clk_disable(RCU_GPIOA);
    hal_gpio_deinit(GPIOF, GPIO_PIN_7);
    hal_gpio_deinit(GPIOB, GPIO_PIN_3);
    hal_gpio_deinit(GPIOB, GPIO_PIN_2);
    hal_gpio_deinit(GPIOF, GPIO_PIN_6);
    hal_gpio_deinit(GPIOB, GPIO_PIN_5);
    hal_gpio_deinit(GPIOB, GPIO_PIN_4);
    hal_gpio_deinit(GPIOB, GPIO_PIN_7);
    hal_gpio_deinit(GPIOB, GPIO_PIN_6);
    hal_gpio_deinit(GPIOB, GPIO_PIN_9);
    hal_gpio_deinit(GPIOB, GPIO_PIN_8);
    hal_gpio_deinit(GPIOA, GPIO_PIN_0);
    hal_gpio_deinit(GPIOA, GPIO_PIN_1);
    hal_gpio_deinit(GPIOA, GPIO_PIN_4);
    hal_gpio_deinit(GPIOA, GPIO_PIN_8);
    hal_gpio_deinit(GPIOA, GPIO_PIN_9);
    hal_gpio_deinit(GPIOC, GPIO_PIN_13);
    hal_gpio_deinit(GPIOA, GPIO_PIN_15);
    hal_gpio_deinit(GPIOC, GPIO_PIN_15);
    hal_gpio_deinit(GPIOC, GPIO_PIN_14);
    hal_gpio_deinit(GPIOA, GPIO_PIN_11);
    hal_gpio_deinit(GPIOB, GPIO_PIN_10);
    hal_gpio_deinit(GPIOA, GPIO_PIN_10);
    hal_gpio_deinit(GPIOB, GPIO_PIN_12);
    hal_gpio_deinit(GPIOB, GPIO_PIN_11);
    hal_gpio_deinit(GPIOA, GPIO_PIN_12);
    hal_gpio_deinit(GPIOF, GPIO_PIN_1);
    hal_gpio_deinit(GPIOF, GPIO_PIN_0);
    hal_gpio_deinit(GPIOB, GPIO_PIN_0);
    /* user code [gpio_deinit local 1] begin */
    /* user code [gpio_deinit local 1] end */
}

void msd_adc_init(void)
{
    /* user code [adc_init local 0] begin */
    /* user code [adc_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;
    hal_adc_init_struct adc_init_parameter;
    hal_adc_regularch_init_struct adc_reginit_parameter;
    hal_adc_regularch_config_struct adc_regchannel_parameter;

    hal_rcu_periph_clk_enable(RCU_ADC);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_ANALOG;
    gpio_init_parameter.pull = HAL_GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_1, &gpio_init_parameter);

    hal_adc_struct_init(HAL_ADC_DEV_STRUCT, &adc_info);
    hal_adc_struct_init(HAL_ADC_INIT_STRUCT, &adc_init_parameter);
    hal_adc_struct_init(HAL_ADC_REGULARCH_INIT_STRUCT, &adc_reginit_parameter);
    hal_adc_struct_init(HAL_ADC_REGULARCH_CONFIG_STRUCT, &adc_regchannel_parameter);

    adc_init_parameter.resolution_select = ADC_RESOLUTION_12B;
    adc_init_parameter.data_alignment = ADC_DATAALIGN_RIGHT;
    adc_init_parameter.scan_mode = DISABLE;
    adc_init_parameter.oversample_config.oversample_mode = DISABLE;
    hal_adc_init(&adc_info, &adc_init_parameter);

    adc_reginit_parameter.length = 1;
    adc_reginit_parameter.exttrigger_select = ADC_EXTTRIG_REGULAR_NONE;
    adc_reginit_parameter.continuous_mode = DISABLE;
    adc_reginit_parameter.discontinuous_mode = ENABLE;
    adc_reginit_parameter.discontinuous_channel_length = 1;
    hal_adc_regular_channel_init(&adc_info, &adc_reginit_parameter);

    adc_regchannel_parameter.regular_channel = ADC_CHANNEL_9;
    adc_regchannel_parameter.regular_sequence = ADC_REGULAR_SEQUENCE_0;
    adc_regchannel_parameter.sample_time = ADC_SAMPLETIME_239POINT5;
    hal_adc_regular_channel_config(&adc_info, &adc_regchannel_parameter);

    hal_adc_calibration(&adc_info);
    /* user code [adc_init local 1] begin */
    /* user code [adc_init local 1] end */
}

void msd_adc_deinit(void)
{
    /* user code [adc_deinit local 0] begin */
    /* user code [adc_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_ADC);
    hal_gpio_deinit(GPIOB, GPIO_PIN_1);
    hal_adc_deinit(&adc_info);
    /* user code [adc_deinit local 1] begin */
    /* user code [adc_deinit local 1] end */
}

void msd_crc_init(void)
{
    /* user code [crc_init local 0] begin */
    /* user code [crc_init local 0] end */
    hal_crc_init_struct crc_init_parameter;

    hal_rcu_periph_clk_enable(RCU_CRC);
    hal_crc_struct_init(&crc_init_parameter);

    crc_init_parameter.output_reverse = DISABLE;
    crc_init_parameter.input_reverse = CRC_INPUT_REVERSE_NOT;
    crc_init_parameter.polynomial_size = CRC_POLYNOMIAL_SIZE_32BIT;
    crc_init_parameter.polynomial = 0x04C11DB7;
    crc_init_parameter.initdata = 0xFFFFFFFF;
    hal_crc_init(&crc_init_parameter);

    /* user code [crc_init local 1] begin */
    /* user code [crc_init local 1] end */
}

void msd_crc_deinit(void)
{
    /* user code [crc_deinit local 0] begin */
    /* user code [crc_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_CRC);
    hal_crc_deinit();
    /* user code [crc_deinit local 1] begin */
    /* user code [crc_deinit local 1] end */
}

void msd_dbg_init(void)
{
    /* user code [dbg_init local 0] begin */
    /* user code [dbg_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;

    hal_rcu_periph_clk_enable(RCU_PMU);
    hal_gpio_struct_init(&gpio_init_parameter);

    /* user code [dbg_init local 1] begin */
    /* user code [dbg_init local 1] end */
}

void msd_rtc_init(void)
{
    /* user code [rtc_init local 0] begin */
    /* user code [rtc_init local 0] end */
    hal_rtc_init_struct rtc_init_parameter;

    hal_rcu_periph_clk_enable(RCU_RTC);
    hal_rtc_register_sync_wait();
    hal_rtc_struct_init(HAL_RTC_INIT_STRUCT, &rtc_init_parameter);

    hal_rtc_deinit();
    rtc_init_parameter.rtc_factor_asyn = 0x7F;
    rtc_init_parameter.rtc_factor_syn = 0xFF;
    rtc_init_parameter.rtc_display_format = HAL_RTC_24HOUR;
    hal_rtc_init(&rtc_init_parameter);

    /* user code [rtc_init local 1] begin */
    /* user code [rtc_init local 1] end */
}

void msd_rtc_deinit(void)
{
    /* user code [rtc_deinit local 0] begin */
    /* user code [rtc_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_RTC);
    hal_rtc_deinit();
    /* user code [rtc_deinit local 1] begin */
    /* user code [rtc_deinit local 1] end */
}

void msd_spi0_init(void)
{
    /* user code [spi0_init local 0] begin */
    /* user code [spi0_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;
    hal_spi_init_struct spi0_init_parameter;

    hal_rcu_periph_clk_enable(RCU_SPI0);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_6, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_5, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOA, GPIO_PIN_7, &gpio_init_parameter);

    hal_spi_struct_init(HAL_SPI_INIT_STRUCT, &spi0_init_parameter);
    hal_spi_struct_init(HAL_SPI_DEV_STRUCT, &spi0_info);

    spi0_init_parameter.device_mode = SPI_MASTER;
    spi0_init_parameter.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi0_init_parameter.frame_size = SPI_FRAMESIZE_8BIT;
    spi0_init_parameter.endian = SPI_ENDIAN_MSB;
    spi0_init_parameter.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi0_init_parameter.prescale = SPI_PSC_16;
    spi0_init_parameter.crc_calculation = SPI_CRC_DISABLE;
    spi0_init_parameter.crc_poly = 7;
    spi0_init_parameter.nss = SPI_NSS_SOFT;
    spi0_init_parameter.ti_mode = SPI_TIMODE_DISABLE;
    spi0_init_parameter.nssp_mode = SPI_NSSP_DISABLE;
    hal_spi_init(&spi0_info, SPI0, &spi0_init_parameter);

    /* user code [spi0_init local 1] begin */
    /* user code [spi0_init local 1] end */
}

void msd_spi0_deinit(void)
{
    /* user code [spi0_deinit local 0] begin */
    /* user code [spi0_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_SPI0);
    hal_gpio_deinit(GPIOA, GPIO_PIN_6);
    hal_gpio_deinit(GPIOA, GPIO_PIN_5);
    hal_gpio_deinit(GPIOA, GPIO_PIN_7);
    hal_spi_deinit(&spi0_info);
    /* user code [spi0_deinit local 1] begin */
    /* user code [spi0_deinit local 1] end */
}

void msd_spi1_init(void)
{
    /* user code [spi1_init local 0] begin */
    /* user code [spi1_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;
    hal_spi_init_struct spi1_init_parameter;

    hal_rcu_periph_clk_enable(RCU_SPI1);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_14, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_13, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_UP;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_0;
    hal_gpio_init(GPIOB, GPIO_PIN_15, &gpio_init_parameter);

    hal_spi_struct_init(HAL_SPI_INIT_STRUCT, &spi1_init_parameter);
    hal_spi_struct_init(HAL_SPI_DEV_STRUCT, &spi1_info);

    spi1_init_parameter.device_mode = SPI_MASTER;
    spi1_init_parameter.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi1_init_parameter.frame_size = SPI_FRAMESIZE_8BIT;
    spi1_init_parameter.endian = SPI_ENDIAN_MSB;
    spi1_init_parameter.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi1_init_parameter.prescale = SPI_PSC_16;
    spi1_init_parameter.crc_calculation = SPI_CRC_DISABLE;
    spi1_init_parameter.crc_poly = 7;
    spi1_init_parameter.crc_length = SPI_CRC_8BIT;
    spi1_init_parameter.nss = SPI_NSS_SOFT;
    spi1_init_parameter.ti_mode = SPI_TIMODE_DISABLE;
    spi1_init_parameter.nssp_mode = SPI_NSSP_DISABLE;
    hal_spi_init(&spi1_info, SPI1, &spi1_init_parameter);

    /* user code [spi1_init local 1] begin */
    /* user code [spi1_init local 1] end */
}

void msd_spi1_deinit(void)
{
    /* user code [spi1_deinit local 0] begin */
    /* user code [spi1_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_SPI1);
    hal_gpio_deinit(GPIOB, GPIO_PIN_14);
    hal_gpio_deinit(GPIOB, GPIO_PIN_13);
    hal_gpio_deinit(GPIOB, GPIO_PIN_15);
    hal_spi_deinit(&spi1_info);
    /* user code [spi1_deinit local 1] begin */
    /* user code [spi1_deinit local 1] end */
}

void msd_timer0_init(void)
{
    /* user code [timer0_init local 0] begin */
    /* user code [timer0_init local 0] end */
    hal_timer_basic_struct timer0_basic_parameter;
    hal_timer_clocksource_struct timer0_clocksource_parameter;

    hal_rcu_periph_clk_enable(RCU_TIMER0);
    hal_timer_struct_init(HAL_TIMER_DEV_STRUCT, &timer0_info);
    hal_timer_struct_init(HAL_TIMER_BASIC_STRUCT, &timer0_basic_parameter);

    timer0_basic_parameter.prescaler = 19;
    timer0_basic_parameter.alignedmode = TIMER_COUNTER_EDGE;
    timer0_basic_parameter.counterdirection = TIMER_COUNTER_UP;
    timer0_basic_parameter.period = 199;
    timer0_basic_parameter.clockdivision = TIMER_CKDIV_DIV1;
    timer0_basic_parameter.repetitioncounter = 0;
    timer0_basic_parameter.autoreload_shadow = AUTO_RELOAD_SHADOW_DISABLE;
    timer0_basic_parameter.trgo_selection = TIMRE_TRGO_SRC_RESET;
    timer0_basic_parameter.master_slave_mode = TIMER_MASTER_SLAVE_MODE_DISABLE;
    hal_timer_basic_init(&timer0_info, TIMER0, &timer0_basic_parameter);
    hal_timer_struct_init(HAL_TIMER_CLOCKSOURCE_STRUCT, &timer0_clocksource_parameter);

    timer0_clocksource_parameter.clock_source = TIMER_CLOCK_SOURCE_CK_TIMER;
    hal_timer_clock_source_config(&timer0_info, &timer0_clocksource_parameter);

    hal_nvic_periph_irq_enable(TIMER0_BRK_UP_TRG_COM_IRQn, 2);
    /* user code [timer0_init local 1] begin */
    /* user code [timer0_init local 1] end */
}

void msd_timer0_deinit(void)
{
    /* user code [timer0_deinit local 0] begin */
    /* user code [timer0_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_TIMER0);
    hal_timer_deinit(&timer0_info);
    /* user code [timer0_deinit local 1] begin */
    /* user code [timer0_deinit local 1] end */
}

void msd_timer2_init(void)
{
    /* user code [timer2_init local 0] begin */
    /* user code [timer2_init local 0] end */
    hal_timer_basic_struct timer2_basic_parameter;
    hal_timer_clocksource_struct timer2_clocksource_parameter;

    hal_rcu_periph_clk_enable(RCU_TIMER2);
    hal_timer_struct_init(HAL_TIMER_DEV_STRUCT, &timer2_info);
    hal_timer_struct_init(HAL_TIMER_BASIC_STRUCT, &timer2_basic_parameter);

    timer2_basic_parameter.prescaler = 0;
    timer2_basic_parameter.alignedmode = TIMER_COUNTER_EDGE;
    timer2_basic_parameter.counterdirection = TIMER_COUNTER_UP;
    timer2_basic_parameter.period = 0;
    timer2_basic_parameter.clockdivision = TIMER_CKDIV_DIV1;
    timer2_basic_parameter.autoreload_shadow = AUTO_RELOAD_SHADOW_DISABLE;
    timer2_basic_parameter.trgo_selection = TIMRE_TRGO_SRC_RESET;
    timer2_basic_parameter.master_slave_mode = TIMER_MASTER_SLAVE_MODE_DISABLE;
    hal_timer_basic_init(&timer2_info, TIMER2, &timer2_basic_parameter);
    hal_timer_struct_init(HAL_TIMER_CLOCKSOURCE_STRUCT, &timer2_clocksource_parameter);

    timer2_clocksource_parameter.clock_source = TIMER_CLOCK_SOURCE_CK_TIMER;
    hal_timer_clock_source_config(&timer2_info, &timer2_clocksource_parameter);

    /* user code [timer2_init local 1] begin */
    /* user code [timer2_init local 1] end */
}

void msd_timer2_deinit(void)
{
    /* user code [timer2_deinit local 0] begin */
    /* user code [timer2_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_TIMER2);
    hal_timer_deinit(&timer2_info);
    /* user code [timer2_deinit local 1] begin */
    /* user code [timer2_deinit local 1] end */
}

void msd_usart1_init(void)
{
    /* user code [usart1_init local 0] begin */
    /* user code [usart1_init local 0] end */
    hal_gpio_init_struct gpio_init_parameter;
    hal_uart_init_struct uart1_init_parameter;

    hal_rcu_periph_clk_enable(RCU_USART1);
    hal_gpio_struct_init(&gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_1;
    hal_gpio_init(GPIOA, GPIO_PIN_2, &gpio_init_parameter);

    gpio_init_parameter.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_parameter.pull = HAL_GPIO_PULL_NONE;
    gpio_init_parameter.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_parameter.af = HAL_GPIO_AF_1;
    hal_gpio_init(GPIOA, GPIO_PIN_3, &gpio_init_parameter);

    hal_uart_struct_init(HAL_UART_DEV_STRUCT, &uart1_info);
    hal_uart_struct_init(HAL_UART_INIT_STRUCT, &uart1_init_parameter);

    uart1_init_parameter.baudrate = 115200;
    uart1_init_parameter.parity = UART_PARITY_NONE;
    uart1_init_parameter.word_length = UART_WORD_LENGTH_8BIT;
    uart1_init_parameter.direction = UART_DIRECTION_RX_TX;
    uart1_init_parameter.stop_bit = UART_STOP_BIT_1;
    uart1_init_parameter.over_sample = UART_OVER_SAMPLE_16;
    uart1_init_parameter.hardware_flow = UART_HARDWARE_FLOW_NONE;
    uart1_init_parameter.work_mode = UART_WORK_MODE_ASYN;
    uart1_init_parameter.sample_method = UART_THREE_SAMPLE_BIT;
    hal_uart_init(&uart1_info, USART1, &uart1_init_parameter);

    hal_nvic_periph_irq_enable(USART1_IRQn, 1);
    /* user code [usart1_init local 1] begin */
    /* user code [usart1_init local 1] end */
}

void msd_usart1_deinit(void)
{
    /* user code [usart1_deinit local 0] begin */
    /* user code [usart1_deinit local 0] end */
    hal_rcu_periph_clk_disable(RCU_USART1);
    hal_gpio_deinit(GPIOA, GPIO_PIN_2);
    hal_gpio_deinit(GPIOA, GPIO_PIN_3);
    hal_uart_deinit(&uart1_info);
    /* user code [usart1_deinit local 1] begin */
    /* user code [usart1_deinit local 1] end */
}

/* user code [global 1] begin */

/* user code [global 1] end */
