/*
    \file  gd32e23x_hal_init.h
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
#ifndef  GD32E23X_HAL_INIT_H
#define  GD32E23X_HAL_INIT_H
#include "gd32e23x_hal.h"
/* user code [global 0] begin */

/* user code [global 0] end */
extern hal_adc_dev_struct adc_info;
extern hal_spi_dev_struct spi0_info;
extern hal_spi_dev_struct spi1_info;
extern hal_timer_dev_struct timer0_info;
extern hal_timer_dev_struct timer2_info;
extern hal_uart_dev_struct uart1_info;

void msd_system_init(void);
void msd_clock_init(void);
void msd_gpio_init(void);
void msd_gpio_deinit(void);
void msd_adc_init(void);
void msd_adc_deinit(void);
void msd_crc_init(void);
void msd_crc_deinit(void);
void msd_dbg_init(void);
void msd_rtc_init(void);
void msd_rtc_deinit(void);
void msd_spi0_init(void);
void msd_spi0_deinit(void);
void msd_spi1_init(void);
void msd_spi1_deinit(void);
void msd_timer0_init(void);
void msd_timer0_deinit(void);
void msd_timer2_init(void);
void msd_timer2_deinit(void);
void msd_usart1_init(void);
void msd_usart1_deinit(void);

/* user code [global 1] begin */

/* user code [global 1] end */
#endif  
