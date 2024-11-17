/*!
    \file    gd32e23x_hal_exti.c
    \brief   EXTI driver
    
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

#define _EXTI_GPIO_MAX_NUM              (16U)
#define _EXTI_RISING                    (0x00000001U)
#define _EXTI_FALLING                   (0x00000002U)
#define _EXTI_EVENT                     (0x00000004U)
#define _EXTI_INTERRUPT                 (0x00000008U)

#define _EXTI_GPIO_PORT_GET(x)          ((x >> 4) & (uint8_t)0x0F)
#define _EXTI_GPIO_PIN_GET(x)           (x & (uint8_t)0x0F)

#define _GPIO_PIN_VALUE_MASK              ((uint32_t)0xFFFF0000U)
#define _GPIOC_PIN_VALUE_MASK             ((uint32_t)0xFFFF1FFFU)
#define _GPIOF_PIN_VALUE_MASK             ((uint32_t)0xFFFFFF3CU)

volatile uint32_t _exti_gpio_used = 0;
static hal_gpio_irq_handle_cb _gpio_irq_handle = NULL;
static volatile uint8_t _exti_gpio_info[_EXTI_GPIO_MAX_NUM];

static void _exti_type_config(uint32_t pin, hal_exti_type_enum exti_type);
static void _exti_gpio_info_set(uint32_t gpio_periph, uint32_t pin);

/*!
    \brief      deinitialize the EXTI GPIO
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_exti_gpio_deinit(uint32_t gpio_periph, uint32_t pin)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check gpio_periph value */
    if((GPIOA != gpio_periph) && (GPIOB != gpio_periph) && (GPIOC != gpio_periph) && (GPIOF != gpio_periph)){
        HAL_DEBUGE("parameter [gpio_periph] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check if pin is PA0 ~ PA15/PB0 ~ PB15 or not*/
    if((GPIOA == gpio_periph) || (GPIOB == gpio_periph)){
        if((0U != (pin & _GPIO_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
    }
    
    /* check if pin is PC13 ~ PC15 or not*/
    if((GPIOC == gpio_periph)){
        if((0U != (pin & _GPIOC_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
    }
    
    /* check if pin is PF0 ~ PF1/PF6 ~ PF7 or not*/
    if((GPIOF == gpio_periph)){
        if((0U != (pin & _GPIOF_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    hal_gpio_deinit(gpio_periph, pin);
    _exti_gpio_used &= ~pin;
    /* reset the EXTI gpio pin */
    EXTI_INTEN &= (uint32_t)~pin;
    EXTI_EVEN  &= (uint32_t)~pin;
    EXTI_RTEN  &= (uint32_t)~pin;
    EXTI_FTEN  &= (uint32_t)~pin;
    EXTI_SWIEV &= (uint32_t)~pin;

    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize the EXTI internal line
    \param[in]  line: the argument could be selected from enumeration <hal_exti_internal_line_enum>
    \param[out] none
    \retval     none
*/
void hal_exti_internal_deinit(hal_exti_internal_line_enum line)
{
    EXTI_INTEN &= (uint32_t)~line;
    EXTI_EVEN  &= (uint32_t)~line;
    EXTI_RTEN  &= (uint32_t)~line;
    EXTI_FTEN  &= (uint32_t)~line;
    EXTI_SWIEV &= (uint32_t)~line;
}

/*!
    \brief      initialize the configuration of EXTI gpio 
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[in]  pull: gpio pin with pull-up or pull-down resistor
                only one parameter can be selected which is shown as below:
      \arg        HAL_GPIO_PULL_NONE: floating mode, no pull-up and pull-down resistors
      \arg        HAL_GPIO_PUll_PULLUP: with pull-up resistor
      \arg        HAL_GPIO_PUll_PULLDOWN: with pull-down resistor
    \param[in]  exti_type: the argument could be selected from enumeration <hal_exti_type_enum>
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_ALREADY_DONE, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_exti_gpio_init(uint32_t gpio_periph, uint32_t pin, uint32_t pull, hal_exti_type_enum exti_type)
{
    hal_gpio_init_struct gpio_init;

#if (1 == HAL_PARAMETER_CHECK)
    /* check gpio_periph value */
    if((GPIOA != gpio_periph) && (GPIOB != gpio_periph) && (GPIOC != gpio_periph) && (GPIOF != gpio_periph)){
        HAL_DEBUGE("parameter [gpio_periph] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check if pin is PA0 ~ PA15/PB0 ~ PB15 or not*/
    if((GPIOA == gpio_periph) || (GPIOB == gpio_periph)){
        if((0U != (pin & _GPIO_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
    }
    
    /* check if pin is PC13 ~ PC15 or not*/
    if((GPIOC == gpio_periph)){
        if((0U != (pin & _GPIOC_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
    }
    
    /* check if pin is PF0 ~ PF1/PF6 ~ PF7 or not*/
    if((GPIOF == gpio_periph)){
        if((0U != (pin & _GPIOF_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;
        }
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the pin is use or not */
    if(0 != (_exti_gpio_used & pin)){
        HAL_DEBUGE("exti gpio init fail, this pin is already used");
        return HAL_ERR_ALREADY_DONE;
    }else{
        _exti_gpio_used |= pin;
        _exti_gpio_info_set(gpio_periph, pin);
    }
    
    hal_gpio_struct_init(&gpio_init);
    gpio_init.mode = HAL_GPIO_MODE_INPUT;
    gpio_init.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init.pull = pull;
    
    hal_gpio_init(gpio_periph, pin, &gpio_init);
    hal_syscfg_exti_config(gpio_periph, pin);
    _exti_type_config(pin, exti_type);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the configuration of EXTI internal 
    \param[in]  line: the argument could be selected from enumeration <hal_exti_internal_line_enum>
    \param[in]  exti_type: the argument could be selected from enumeration <hal_exti_type_enum>
    \param[out] none
    \retval     none
*/
void hal_exti_internal_init(hal_exti_internal_line_enum line, hal_exti_type_enum exti_type)
{
    _exti_type_config(line, exti_type);
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  irq_handler: configuration the EXTI callback function, implemented by the user himself
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_exti_gpio_irq_handle_set(hal_gpio_irq_handle_cb irq_handler)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check irq_handler address */
    if(NULL == irq_handler){
        HAL_DEBUGE("irq_handler is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    _gpio_irq_handle = irq_handler;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_exti_gpio_irq_handle_all_reset(void)
{
    _gpio_irq_handle = NULL;
}

/*!
    \brief      EXTI_GPIO interrupt handler content function,which is merely used in EXTI_GPIO_handler
    \param[in]  index: indicate this function will be called by which interrupt handler entry
      \arg        the argument could be selected from enumeration <hal_exti_irq_index>
    \param[out] none
    \retval     none
*/
void hal_exti_gpio_irq(hal_exti_irq_index index)
{
    uint32_t i;
    uint32_t gpio_port;
    uint32_t gpio_pin;
    uint32_t temp;
    uint8_t start_pin, end_pin;
    
    start_pin = (index & 0xF0U) >> 4;
    end_pin = index & 0x0FU;
    
    /* get EXTI GPIO port and pin */
    for (i = start_pin; i <= end_pin; i++) {
        if(0 != (_exti_gpio_used & (1 << i))){
            temp = _EXTI_GPIO_PORT_GET(_exti_gpio_info[i]);
            gpio_port = GPIO_BASE + (temp << 10); 
            temp = _EXTI_GPIO_PIN_GET(_exti_gpio_info[i]);
            gpio_pin = ((uint32_t)1 << temp);
            
            /* get EXTI lines flag when the interrupt flag is set */
            if(RESET != exti_interrupt_flag_get((exti_line_enum)gpio_pin)){
                exti_interrupt_flag_clear((exti_line_enum)gpio_pin);
                if(NULL == _gpio_irq_handle){
                    continue;
                }
                
                /* get EXTI GPIO pin iutput status */
                if(RESET == hal_gpio_input_bit_get(gpio_port, gpio_pin)){
                    _gpio_irq_handle(gpio_pin, EXTI_IRQ_EVENT_FALLING);
                }else{
                    _gpio_irq_handle(gpio_pin, EXTI_IRQ_EVENT_RISING);
                }
            }
        }
    }
}

/*!
    \brief      activate the EXTI line software interrupt/event request
    \param[in]  linex: EXTI line number
                only one parameter can be selected which is shown as below:
      \arg        the argument could be selected from enumeration <hal_exti_line_enum>
    \param[out] none
    \retval     none
*/
void hal_exti_software_interrupt_trigger(hal_exti_line_enum linex)
{
    EXTI_SWIEV |= (uint32_t)linex;
}

/*!
    \brief      enable the configuration of EXTI type
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15, 16, 17, 19, 21, 25), GPIO_PIN_ALL
    \param[in]  exti_type: the argument could be selected from enumeration <hal_exti_type_enum>
    \param[out] none
    \retval     none
*/
static void _exti_type_config(uint32_t pin, hal_exti_type_enum exti_type)
{
    uint32_t reg_temp;

    /* reset the EXTI gpio pin */
    EXTI_INTEN &= ~pin;
    EXTI_EVEN &= ~pin;
    EXTI_RTEN &= ~pin;
    EXTI_FTEN &= ~pin;
    EXTI_PD = pin;
    
    /* set the EXTI trigger type */
    
    /* set the EXTI trigger type as the rising edge trigger */
    reg_temp = EXTI_RTEN;
    if(0 != (exti_type & _EXTI_RISING)){
        reg_temp |= pin;
    }else{
        reg_temp &= ~pin;
    }
    EXTI_RTEN = reg_temp;
    
    /* set the EXTI trigger type as the falling edge trigger */
    reg_temp = EXTI_FTEN;
    if(0 != (exti_type & _EXTI_FALLING)){
        reg_temp |= pin;
    }else{
        reg_temp &= ~pin;
    }
    EXTI_FTEN = reg_temp;
    
    /* set the EXTI trigger type as the event trigger */
    reg_temp = EXTI_EVEN;
    if(0 != (exti_type & _EXTI_EVENT)){
        reg_temp |= pin;
    }else{
        reg_temp &= ~pin;
    }
    EXTI_EVEN = reg_temp;
    
    /* set the EXTI trigger type as the interrupt trigger */
    reg_temp = EXTI_INTEN;
    if(0 != (exti_type & _EXTI_INTERRUPT)){
        reg_temp |= pin;
    }else{
        reg_temp &= ~pin;
    }
    EXTI_INTEN = reg_temp;

}

/*!
    \brief      set the EXTI gpio port and pin
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
*/
static void _exti_gpio_info_set(uint32_t gpio_periph, uint32_t pin)
{
    uint32_t gpio_port;
    uint8_t gpio_pin;
    
    /* set the EXTI gpio port */
    gpio_port = gpio_periph - GPIO_BASE;  
    gpio_port = (gpio_port >> 10U);
    
    /* set the EXTI gpio pin */
    for(gpio_pin = 0U; gpio_pin < 16U; gpio_pin++){
        if((1U << gpio_pin) & pin){
            _exti_gpio_info[gpio_pin] = (uint8_t)((gpio_port << 4) | gpio_pin);
        }
    }
}
