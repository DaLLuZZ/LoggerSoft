/*!
    \file    gd32e23x_hal_syscfg.h
    \brief   definitions for the SYSCFG

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

#ifndef GD32E23X_HAL_SYSCFG_H
#define GD32E23X_HAL_SYSCFG_H
#include "gd32e23x_hal.h"

/* function declarations */
/* deinit syscfg module */
void hal_syscfg_deinit(void);

/* enable the DMA channels remapping */
int32_t hal_syscfg_dma_remap_enable(uint32_t syscfg_dma_remap);
/* disable the DMA channels remapping */
int32_t hal_syscfg_dma_remap_disable(uint32_t syscfg_dma_remap);

#if defined(GD32E230)
/* enable PB9 high current capability */
void hal_syscfg_high_current_enable(void);
/* disable PB9 high current capability */
void hal_syscfg_high_current_disable(void);
#endif /* GD32E230 */

/* configure the GPIO pin as EXTI Line */
int32_t hal_syscfg_exti_config(uint32_t gpio_periph, uint32_t pin);
/* connect TIMER0/14/15/16 break input to the selected parameter */
int32_t hal_syscfg_lock_config(uint32_t syscfg_lock);

/* set the IRQ_LATENCY value */
void hal_irq_latency_set(uint8_t irq_latency);

/* check if the SRAM parity check error flag in SYSCFG_CFG2 is set or not */
FlagStatus hal_syscfg_sram_pcef_flag_get(void);
/* clear the SRAM parity check error flag in SYSCFG_CFG2 by writing 1 */
void hal_syscfg_sram_pcef_flag_clear(void);

#endif /* GD32E23X_HAL_SYSCFG_H */
