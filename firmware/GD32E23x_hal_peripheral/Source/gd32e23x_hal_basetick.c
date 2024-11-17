/*!
    \file    gd32e23x_hal_basetick.c
    \brief   basetick driver 

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

#include "gd32e23x_hal.h"

/* global variable, basetick timer source */
hal_basetick_source_enum g_basetick_source = HAL_BASETICK_SOURCE_SYSTICK;

/* the clock source of the basetick timer */
static const rcu_periph_enum _BASETICK_SOURCE_CLK[] = {RCU_TIMER0, RCU_TIMER2, \
                                                       RCU_TIMER5, RCU_TIMER13, \
                                                       RCU_TIMER14, RCU_TIMER15, \
                                                       RCU_TIMER16};
/* the peripheral of the basetick timer */
static const uint32_t _BASETICK_SOURCE_PERIPH[] = {TIMER0, TIMER2, TIMER5, \
                                                   TIMER13, TIMER14, TIMER15, \
                                                   TIMER16};
/* the interrupt number of the correspond timer */
static const uint8_t _BASETICK_SOURCE_IRQN[] = {13, 16, 17, 19, 20, 21, 22};

/* the callback of basetick interrupt definition */
static hal_basetick_irq_handle_cb _basetick_irq_handle = NULL;

/* the internal basetick counter */
static __IO uint32_t g_basetick_cnt;
static __IO uint32_t g_basetick_count = 0U;

/* static function declaration */
static void _hal_systick_init(uint32_t count_freq, uint32_t prio);
static void _hal_basetick_timer_init(hal_basetick_source_enum source, uint32_t count_freq, uint8_t prio);

/*!
    \brief      initialize basetick timer
    \param[in]  source: select the source timer
      \arg        HAL_BASETICK_SOURCE_TIMERx: x=0,1,2,5,13,14,15,16, use the TIMERx as the source
      \arg        HAL_BASETICK_SOURCE_SYSTICK: use the systick as the source
    \param[in]  count_freq: the frequence of basetick interrupt
    \param[in]  prio: the priority of basetick interrupt
    \param[out] none
    \retval     none
*/
void hal_basetick_init(hal_basetick_source_enum source)
{
    g_basetick_source = source;
    if(HAL_BASETICK_SOURCE_SYSTICK == source){
        _hal_systick_init(HAL_BASETICK_RATE_HZ, 0U);
    }else{
        _hal_basetick_timer_init(source, HAL_BASETICK_RATE_HZ, 0U);
    }
}

/*!
    \brief      get the basetick count
    \param[in]  none
    \param[out] none
    \retval     the basetick count
*/
uint32_t hal_basetick_count_get(void)
{
    return (g_basetick_count);
}

/*!
    \brief      check whether the delay is finished
    \param[in]  time_start: the starting time point of the delay
    \param[in]  delay: the delay interval
    \param[out] none
    \retval     the basetick count
*/
FlagStatus hal_basetick_timeout_check(uint32_t time_start, uint32_t delay)
{
    if(g_basetick_count - time_start > delay){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      set the basetick delay
    \param[in]  time_ms: the timeout interval
    \param[out] none
    \retval     none
*/
void hal_basetick_delay_ms(uint32_t time_ms)
{
    uint32_t delay;
    uint32_t time_start = g_basetick_count;

    delay = (time_ms * (uint32_t)((float)HAL_BASETICK_RATE_HZ / (float)1000U));

    while(g_basetick_count - time_start < delay){
        
    }
}

/*!
    \brief      suspend the basetick timer
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_basetick_suspend(void)
{
    if(HAL_BASETICK_SOURCE_SYSTICK == g_basetick_source){
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
    }else{
        timer_disable(_BASETICK_SOURCE_PERIPH[g_basetick_source]);
    }
}

/*!
    \brief      resume the basetick timer
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_basetick_resume(void)
{
    if(HAL_BASETICK_SOURCE_SYSTICK == g_basetick_source){
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    }else{
        timer_enable(_BASETICK_SOURCE_PERIPH[g_basetick_source]);
    }
}

/*!
    \brief      basetick interrupt handler content function, which is merely \
                used in SysTick_Handler or TIMERx_UP_IRQHandler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_basetick_irq(void)
{
    if(g_basetick_cnt != 0U){
        g_basetick_cnt--;
    }
    g_basetick_count++;

    switch(g_basetick_source){
        case HAL_BASETICK_SOURCE_TIMER0:
            timer_interrupt_flag_clear(TIMER0, TIMER_INT_FLAG_UP);
            break;

        case HAL_BASETICK_SOURCE_TIMER2:
            timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
            break;

        case HAL_BASETICK_SOURCE_TIMER5:
            timer_interrupt_flag_clear(TIMER5, TIMER_INT_FLAG_UP);
            break;

        case HAL_BASETICK_SOURCE_TIMER13:
            timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);
            break;

        case HAL_BASETICK_SOURCE_TIMER14:
            timer_interrupt_flag_clear(TIMER14, TIMER_INT_FLAG_UP);
            break;

        case HAL_BASETICK_SOURCE_TIMER15:
            timer_interrupt_flag_clear(TIMER15, TIMER_INT_FLAG_UP);
            break;

        case HAL_BASETICK_SOURCE_TIMER16:
            timer_interrupt_flag_clear(TIMER16, TIMER_INT_FLAG_UP);
            break;

        default:
            break;
    }
    
    if(NULL != _basetick_irq_handle){
        _basetick_irq_handle();
    }
}

/*!
    \brief      set user-defined interrupt callback function, which will be \
                registered and called when corresponding interrupt be triggered
    \param[in]  irq_handler: 
    \param[out] none
    \retval     none
*/
void hal_basetick_irq_handle_set(hal_basetick_irq_handle_cb irq_handler)
{
    if(NULL == irq_handler){
        HAL_DEBUGE("callback function is invalid");
    }

    _basetick_irq_handle = irq_handler;
}

/*!
    \brief      reset all user-defined interrupt callback function, which will \
                be registered and called when corresponding interrupt be triggered
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_basetick_irq_handle_reset(void)
{
    _basetick_irq_handle = NULL;
}

/*!
    \brief      initlize systick when use it as the source
    \param[in]  count_freq: the frequence of basetick interrupt
    \param[in]  prio: the priority of basetick interrupt
    \param[out] none
    \retval     none
*/
static void _hal_systick_init(uint32_t count_freq, uint32_t prio)
{

#if (1 == HAL_PARAMETER_CHECK)
    uint32_t lowest_prio;
    lowest_prio = (0x01 << __NVIC_PRIO_BITS) - 1U;
    if(prio > lowest_prio){
        HAL_DEBUGE("parameter [prio] value is greater than configurable priority");
        prio = lowest_prio;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure the systick handler priority */
    NVIC_SetPriority(SysTick_IRQn, prio);
    
    if (SysTick_Config(SystemCoreClock / count_freq)){
        /* capture error */
        while (1){
        }
    }
}

/*!
    \brief      initlize timer when use it as the source
    \param[in]  source: select the source timer
      \arg        HAL_BASETICK_SOURCE_TIMERx: x=0,1,2,5,13,14,15,16
    \param[in]  count_freq: the frequence of basetick interrupt
    \param[in]  prio: the priority of basetick interrupt
    \param[out] none
    \retval     none
*/
static void _hal_basetick_timer_init(hal_basetick_source_enum source, uint32_t count_freq, uint8_t prio)
{
    timer_parameter_struct timer_initpara;
    uint16_t timer_prescaler;

#if (1 == HAL_PARAMETER_CHECK)
    uint8_t lowest_prio;
    /* check the parameter */
    lowest_prio = (0x01 << __NVIC_PRIO_BITS) - 1U;
    if(prio > lowest_prio){
        HAL_DEBUGE("parameter [prio] value is greater than configurable priority");
        prio = lowest_prio;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* enable the interrupt */
    nvic_irq_enable(_BASETICK_SOURCE_IRQN[source], prio);

    /* enable the clock of timer */
    rcu_periph_clock_enable(_BASETICK_SOURCE_CLK[source]);

    /* get the APBx clock */
    if((source==HAL_BASETICK_SOURCE_TIMER2)||(source==HAL_BASETICK_SOURCE_TIMER5)||\
       (source==HAL_BASETICK_SOURCE_TIMER13)){
        timer_prescaler = (uint16_t)(hal_rcu_periph_clkfreq_get(RCU_PERIPH_CLKTYPE_APB1TIMER)/1000000U) - 1U;
    }else{
        timer_prescaler = (uint16_t)(hal_rcu_periph_clkfreq_get(RCU_PERIPH_CLKTYPE_APB2TIMER)/1000000U) - 1U;
    }
    
    timer_deinit(_BASETICK_SOURCE_PERIPH[source]);
    /* initialize the using timer */
    timer_initpara.prescaler         = timer_prescaler;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 1000000U/count_freq;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0U;
    timer_init(_BASETICK_SOURCE_PERIPH[source], &timer_initpara);
    
    timer_interrupt_enable(_BASETICK_SOURCE_PERIPH[source], TIMER_INT_UP);
    timer_enable(_BASETICK_SOURCE_PERIPH[source]);
}
