 /*!
    \file    gd32e23x_hal_syscfg.c
    \brief   SYSCFG driver

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

#define EXTI_PORT_OFFSET                             ((uint32_t)10U)
#define EXTI_PIN_OFFSET                              ((uint32_t)1U)

#define _GPIO_PIN_VALUE_MASK                         ((uint32_t)0xFFFF0000U)
#define _GPIOC_PIN_VALUE_MASK                        ((uint32_t)0xFFFF1FFFU)
#define _GPIOF_PIN_VALUE_MASK                        ((uint32_t)0xFFFFFF3CU)

#define _SYSCFG_SRAM_PCEF_FLAG                       ((uint32_t)0x00000100U)

/*!
    \brief      reset the SYSCFG registers
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_syscfg_deinit(void)
{
    rcu_periph_reset_enable(RCU_CFGCMPRST);
    rcu_periph_reset_disable(RCU_CFGCMPRST);
}

/*!
    \brief      enable the DMA channels remapping
    \param[in]  syscfg_dma_remap: specify the DMA channels to remap
                one or more parameters can be selected which are shown as below:
      \arg        SYSCFG_DMA_REMAP_TIMER16: remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0)
      \arg        SYSCFG_DMA_REMAP_TIMER15: remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2)
      \arg        SYSCFG_DMA_REMAP_USART0RX: remap USART0 Rx DMA request to channel4(default channel2)
      \arg        SYSCFG_DMA_REMAP_USART0TX: remap USART0 Tx DMA request to channel3(default channel1)
      \arg        SYSCFG_DMA_REMAP_ADC: remap ADC DMA requests from channel0 to channel1
      \arg        SYSCFG_PA11_REMAP_PA12: remap PA11 PA12
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_syscfg_dma_remap_enable(uint32_t syscfg_dma_remap)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check syscfg_dma_remap parameter */
    if((SYSCFG_DMA_REMAP_TIMER16 != syscfg_dma_remap) && (SYSCFG_DMA_REMAP_TIMER15 != syscfg_dma_remap)\
        && (SYSCFG_DMA_REMAP_USART0RX != syscfg_dma_remap) && (SYSCFG_DMA_REMAP_USART0TX != syscfg_dma_remap)\
        && (SYSCFG_DMA_REMAP_ADC != syscfg_dma_remap)&& (SYSCFG_PA11_REMAP_PA12 != syscfg_dma_remap)){
        HAL_DEBUGE("parameter [syscfg_dma_remap] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    SYSCFG_CFG0 |= syscfg_dma_remap;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      disable the DMA channels remapping
    \param[in]  syscfg_dma_remap: specify the DMA channels to remap
                one or more parameters can be selected which are shown as below:
      \arg        SYSCFG_DMA_REMAP_TIMER16: remap TIMER16 channel0 and UP DMA requests to channel1(defaut channel0)
      \arg        SYSCFG_DMA_REMAP_TIMER15: remap TIMER15 channel2 and UP DMA requests to channel3(defaut channel2)
      \arg        SYSCFG_DMA_REMAP_USART0RX: remap USART0 Rx DMA request to channel4(default channel2)
      \arg        SYSCFG_DMA_REMAP_USART0TX: remap USART0 Tx DMA request to channel3(default channel1)
      \arg        SYSCFG_DMA_REMAP_ADC: remap ADC DMA requests from channel0 to channel1
      \arg        SYSCFG_PA11_REMAP_PA12: remap PA11 PA12
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_syscfg_dma_remap_disable(uint32_t syscfg_dma_remap)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check syscfg_dma_remap parameter */
    if((SYSCFG_DMA_REMAP_TIMER16 != syscfg_dma_remap) && (SYSCFG_DMA_REMAP_TIMER15 != syscfg_dma_remap)\
        && (SYSCFG_DMA_REMAP_USART0RX != syscfg_dma_remap) && (SYSCFG_DMA_REMAP_USART0TX != syscfg_dma_remap)\
        && (SYSCFG_DMA_REMAP_ADC != syscfg_dma_remap)&& (SYSCFG_PA11_REMAP_PA12 != syscfg_dma_remap)){
        HAL_DEBUGE("parameter [syscfg_dma_remap] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    SYSCFG_CFG0 &= ~syscfg_dma_remap;
    
    return HAL_ERR_NONE;
}

#if defined(GD32E230)
/*!
    \brief      enable PB9 high current capability
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_syscfg_high_current_enable(void)
{
    SYSCFG_CFG0 |= SYSCFG_HIGH_CURRENT_ENABLE;
}

/*!
    \brief      disable PB9 high current capability
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_syscfg_high_current_disable(void)
{
    SYSCFG_CFG0 &= SYSCFG_HIGH_CURRENT_DISABLE;
}
#endif /* GD32E230 */

/*!
    \brief      configure the GPIO pin as EXTI Line
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
    \param[in]  pin: GPIO pin
                only one parameter can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15)
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_syscfg_exti_config(uint32_t gpio_periph, uint32_t pin)
{
    uint32_t exti_port;
    uint8_t exti_pin;
    uint32_t i;

#if (1 == HAL_PARAMETER_CHECK)
    /* check gpio_periph value */
    if((GPIOA != gpio_periph) && (GPIOB != gpio_periph) && (GPIOC != gpio_periph) && (GPIOF != gpio_periph)){
        HAL_DEBUGE("parameter [gpio_periph] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check if pin is PA0 ~ PA15/PB0 ~ PB15 or not */
    if((GPIOA == gpio_periph) || (GPIOB == gpio_periph)){
#if (1 == HAL_PARAMETER_CHECK)
        if((0U != (pin & _GPIO_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
#endif /* 1 == HAL_PARAMETER_CHECK */
        
        /* check if only one GPIO pin is set or not and get EXTI source pin value */
        for(exti_pin = 0U; exti_pin < 16U; exti_pin++)
        {
            if((EXTI_PIN_OFFSET << exti_pin) & pin){
                    for(i = (exti_pin + 1); i < 16U; i++)
                    {
                        if(0U != ((EXTI_PIN_OFFSET << i) & pin)){
                            HAL_DEBUGE("parameter [pin] value is invalid");
                            return HAL_ERR_VAL;
                        }
                    }
                    break;
               }
        }
    }
        
    /* check if pin is PC13 ~ PC15 or not */
    if((GPIOC == gpio_periph)){
#if (1 == HAL_PARAMETER_CHECK)
        if((0U != (pin & _GPIOC_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
#endif /* 1 == HAL_PARAMETER_CHECK */
        
        /* check if only one GPIO pin is set or not and get EXTI source pin value */
        for(exti_pin = 13U; exti_pin < 16U; exti_pin++)
        {
            if((EXTI_PIN_OFFSET << exti_pin) & pin){
                for(i = (exti_pin +1); i < 16U; i++)
                {
                    if(0U != ((EXTI_PIN_OFFSET << i) & pin)){
                        HAL_DEBUGE("parameter [pin] value is invalid");
                        return HAL_ERR_VAL;
                    }
                }
                break;
            }
        }
    }

    /* check if pin is PF0 ~ PF1/PF6 ~ PF7 or not */
    if((GPIOF == gpio_periph)){
#if (1 == HAL_PARAMETER_CHECK)
        if((0U != (pin & _GPIOF_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
#endif /* 1 == HAL_PARAMETER_CHECK */
        
        /* check if only one GPIO pin is set or not and get EXTI source pin value */
        for(exti_pin = 0U; exti_pin < 2U; exti_pin++)
        {
            if((EXTI_PIN_OFFSET << exti_pin) & pin){
                for(i = (exti_pin +1); i < 16U; i++)
                {
                    if(0U != ((EXTI_PIN_OFFSET << i) & pin)){
                        HAL_DEBUGE("parameter [pin] value is invalid");
                        return HAL_ERR_VAL;
                    }
                }
                break;
            }
        }

        for(exti_pin = 6U; exti_pin < 8U; exti_pin++)
        {
            if((EXTI_PIN_OFFSET << exti_pin) & pin){
                for(i = (exti_pin +1); i < 16U; i++)
                {
                    if(0U != ((EXTI_PIN_OFFSET << i) & pin)){
                        HAL_DEBUGE("parameter [pin] value is invalid");
                        return HAL_ERR_VAL;
                    }
                }
                break;
            }
        }
    }
    
    /* get EXTI GPIO port */
    exti_port = gpio_periph - GPIO_BASE;  
    exti_port = (exti_port >> EXTI_PORT_OFFSET );

    /* configure the GPIO pin as EXTI Line */
    syscfg_exti_line_config((uint8_t)exti_port, exti_pin);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      connect TIMER0/14/15/16 break input to the selected parameter
    \param[in]  syscfg_lock: Specify the parameter to be connected
                only one parameter can be selected which are shown as below:
      \arg        SYSCFG_LOCK_LOCKUP: Cortex-M23 lockup output connected to the break input
      \arg        SYSCFG_LOCK_SRAM_PARITY_ERROR: SRAM_PARITY check error connected to the break input
      \arg        SYSCFG_LOCK_LVD: LVD interrupt connected to the break input
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_syscfg_lock_config(uint32_t syscfg_lock)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check syscfg_lock parameter */
    if((SYSCFG_LOCK_LOCKUP != syscfg_lock) && (SYSCFG_LOCK_SRAM_PARITY_ERROR != syscfg_lock)\
        && (SYSCFG_LOCK_LVD != syscfg_lock)){
        HAL_DEBUGE("parameter [syscfg_lock] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    SYSCFG_CFG2 |= syscfg_lock;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      set the wait state counter value
    \param[in]  irq_latency: IRQ_LATENCY value (0x00 - 0xFF)
    \param[out] none
    \retval     none
*/
void hal_irq_latency_set(uint8_t irq_latency)
{
    uint32_t reg;
    
    reg = SYSCFG_CPU_IRQ_LAT &(~(uint32_t)SYSCFG_CPU_IRQ_LAT_IRQ_LATENCY);
    reg |= (uint32_t)(IRQ_LATENCY(irq_latency));
    
    SYSCFG_CPU_IRQ_LAT = (uint32_t)reg;
}

/*!
    \brief      check if the SRAM parity check error flag in SYSCFG_CFG2 is set or not
    \param[in]  none
    \param[out] none
    \retval     the syscfg_sarm_pcef_flag state returned (SET or RESET)
  */
FlagStatus hal_syscfg_sram_pcef_flag_get(void)
{
    if((SYSCFG_CFG2 & _SYSCFG_SRAM_PCEF_FLAG) != (uint32_t)RESET){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear the SRAM parity check error flag in SYSCFG_CFG2 by writing 1
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_syscfg_sram_pcef_flag_clear(void)
{
    SYSCFG_CFG2 |= (uint32_t)_SYSCFG_SRAM_PCEF_FLAG;
}
