/*
    \file  gd32e23x_hal_it.c
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
#include "gd32e23x_hal_it.h"
#include "gd32e23x_hal.h"
#include "gd32e23x_hal_init.h"

void NMI_Handler(void)
{
    /* user code [NonMaskableInt_IRQn local 0] begin */
    /* user code [NonMaskableInt_IRQn local 0] end */
    /* user code [NonMaskableInt_IRQn local 1] begin */
    /* user code [NonMaskableInt_IRQn local 1] end */
}

void HardFault_Handler(void)
{
    /* user code [HardFault_IRQn local 0] begin */
    /* user code [HardFault_IRQn local 0] end */
    while(1)
    {
    }

    /* user code [HardFault_IRQn local 1] begin */
    /* user code [HardFault_IRQn local 1] end */
}

void SVC_Handler(void)
{
    /* user code [SVCall_IRQn local 0] begin */
    /* user code [SVCall_IRQn local 0] end */
    /* user code [SVCall_IRQn local 1] begin */
    /* user code [SVCall_IRQn local 1] end */
}

void RTC_IRQHandler(void)
{

}


void PendSV_Handler(void)
{
    /* user code [PendSV_IRQn local 0] begin */
    /* user code [PendSV_IRQn local 0] end */
    /* user code [PendSV_IRQn local 1] begin */
    /* user code [PendSV_IRQn local 1] end */
}

void SysTick_Handler(void)
{
    /* user code [SysTick_IRQn local 0] begin */
    /* user code [SysTick_IRQn local 0] end */
    hal_basetick_irq();
    /* user code [SysTick_IRQn local 1] begin */
    /* user code [SysTick_IRQn local 1] end */
}

void TIMER0_BRK_UP_TRG_COM_IRQHandler(void)
{
    /* user code [TIMER0_BRK_UP_TRG_COM_IRQn local 0] begin */
    /* user code [TIMER0_BRK_UP_TRG_COM_IRQn local 0] end */
    hal_timer_irq(&timer0_info);
    /* user code [TIMER0_BRK_UP_TRG_COM_IRQn local 1] begin */
    /* user code [TIMER0_BRK_UP_TRG_COM_IRQn local 1] end */
}

void USART1_IRQHandler(void)
{
    /* user code [USART1_IRQn local 0] begin */
    /* user code [USART1_IRQn local 0] end */
    hal_uart_irq(&uart1_info);
    /* user code [USART1_IRQn local 1] begin */
    hal_uart_transmit_poll(&uart1_info, "USART1_IRQHandler\r\n", 19, 1000);
    /* user code [USART1_IRQn local 1] end */
}
