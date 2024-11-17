/*!
    \file    gd32e23x_hal_basetick.h
    \brief   definitions for the basetick

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

#ifndef GD32E23X_HAL_BASETICK_H
#define GD32E23X_HAL_BASETICK_H
#include "gd32e23x_hal.h"

/* the source of basetick definitions */
typedef enum{
    HAL_BASETICK_SOURCE_TIMER0 = 0,
    HAL_BASETICK_SOURCE_TIMER2,
    HAL_BASETICK_SOURCE_TIMER5,
    HAL_BASETICK_SOURCE_TIMER13,
    HAL_BASETICK_SOURCE_TIMER14,
    HAL_BASETICK_SOURCE_TIMER15,
    HAL_BASETICK_SOURCE_TIMER16,
    HAL_BASETICK_SOURCE_SYSTICK,
}hal_basetick_source_enum;

/* global variable declaration */
extern hal_basetick_source_enum g_basetick_source;

/* the callback of basetick interrupt declaration */
typedef void (*hal_basetick_irq_handle_cb)(void);

/* function declarations */
/* initialization functions */
/* initialize basetick timer */
void hal_basetick_init(hal_basetick_source_enum source);

/* timeout and delay fucntions */
/* get the basetick count */
uint32_t hal_basetick_count_get(void);
/* check whether the delay is finished */
FlagStatus hal_basetick_timeout_check(uint32_t time_start, uint32_t delay);
/* set the basetick delay */
void hal_basetick_delay_ms(uint32_t time_ms);

/* control fucntions */
/* suspend the basetick timer */
void hal_basetick_suspend(void);
/* resume the basetick timer */
void hal_basetick_resume(void);

/* interrupt functions */
/* basetick interrupt handler content function, 
which is merely used in SysTick_Handler or TIMERx_UP_IRQHandler */
void hal_basetick_irq(void);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_basetick_irq_handle_set(hal_basetick_irq_handle_cb irq_handler);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_basetick_irq_handle_reset(void);

#endif /* GD32E23X_HAL_BASETICK_H */
