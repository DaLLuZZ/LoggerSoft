/*!
    \file    gd32e23x_hal_libopt.h
    \brief   library optional for gd32e23x
    
    \version 2019-03-14, V1.0.0, firmware for GD32E23x
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

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

#ifndef GD32E23X_HAL_LIBOPT_H
#define GD32E23X_HAL_LIBOPT_H

/* if set, flash operation (write and eraser) will reserve original data located
   in out of targeted scope */
#define FLASH_OPER_RESERVE_ORIGINAL_DATA        1
/* if set, the parameters check will be implemented in function */
#define HAL_PARAMETER_CHECK                     0
/* if set, print debug message according to level of marco 'HAL_DEBUG_PRINTF_LEVEL'
   and halt code according to level of marco 'HAL_DEBUG_HALT_LEVEL' */
#define HAL_DEBUG                               0

#if (1 == HAL_DEBUG)
#define HAL_DEBUG_PRINTF                        printf
#define HAL_DEBUG_PRINTF_LEVEL                  HAL_DEBUG_LVL_ALL
#define HAL_DEBUG_HALT_LEVEL                    HAL_DEBUG_LVL_NONE

#define HAL_DEBUG_UART                          USART0
#define HAL_DEBUG_EXTRA_DO
#endif /* 1 == HAL_DEBUG */

#include "gd32e23x_hal_adc.h"
#include "gd32e23x_hal_rcu.h"
#include "gd32e23x_hal_dma.h"
#include "gd32e23x_hal_basetick.h"
#include "gd32e23x_hal_uart.h"
#include "gd32e23x_hal_fwdgt.h"
#include "gd32e23x_hal_nvic.h"
#include "gd32e23x_hal_gpio.h"
#include "gd32e23x_hal_exti.h"
#include "gd32e23x_hal_syscfg.h"
#include "gd32e23x_hal_fmc.h"
#include "gd32e23x_hal_pmu.h"
#include "gd32e23x_hal_irda.h"
#include "gd32e23x_hal_syscfg.h"
#include "gd32e23x_hal_usrt.h"
#include "gd32e23x_hal_rtc.h"
#include "gd32e23x_hal_i2c.h"
#include "gd32e23x_hal_smbus.h"
#include "gd32e23x_hal_spi.h"
#include "gd32e23x_hal_i2s.h"
#include "gd32e23x_hal_timer.h"
#include "gd32e23x_hal_pwmout.h"

#endif /* GD32E23X_HAL_LIBOPT_H */
