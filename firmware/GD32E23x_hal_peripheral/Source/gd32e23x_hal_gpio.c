/*!
    \file    gd32e23x_hal_gpio.c
    \brief   GPIO driver
    
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

#define _GPIO_GET_OUTPUT_MODE(mode)       ((uint32_t)((mode >> 4) & 0x00000001U))
#define _GPIO_AF_VALUE_MASK               ((uint32_t)0xFFFFFFF8U)
#define _GPIO_PIN_VALUE_MASK              ((uint32_t)0xFFFF0000U)
#define _GPIOC_PIN_VALUE_MASK             ((uint32_t)0xFFFF1FFFU)
#define _GPIOF_PIN_VALUE_MASK             ((uint32_t)0xFFFFFF3CU)

/*!
    \brief      initialize the GPIO initialization structure with the default values
    \param[in]  p_init: GPIO intialization structure
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_gpio_struct_init(hal_gpio_init_struct *p_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_init){
        HAL_DEBUGE("pointer [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* set the GPIO initialization structure with the default values */
    p_init->mode = HAL_GPIO_MODE_INPUT;
    p_init->pull = HAL_GPIO_PULL_NONE;
    p_init->ospeed = HAL_GPIO_OSPEED_2MHZ;
    p_init->af = HAL_GPIO_AF_0;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize GPIO
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F) 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_gpio_deinit(uint32_t gpio_periph, uint32_t pin)
{
    uint32_t i;
    
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
    
    /* reset GPIO port */
    if(GPIO_PIN_ALL == pin){
        gpio_deinit(gpio_periph);
    }else{ 
        /* reset GPIO pin */
        for(i = 0U;i < 16U;i++){
            if((1U << i) & pin){
                /* clear the specified pin mode bits */
                GPIO_CTL(gpio_periph) &= ~GPIO_MODE_MASK(i);
                /* clear the specified pin pupd bits */
                GPIO_PUD(gpio_periph) &= ~GPIO_PUPD_MASK(i);
                /* clear the specified pin output speed bits */
                GPIO_OSPD(gpio_periph) &= ~GPIO_OSPEED_MASK(i);
                
                if(i < 8){
                    /* clear the specified pin alternate function bits */
                    GPIO_AFSEL0(gpio_periph) &= ~GPIO_AFR_MASK(i);
                }else{
                    /* clear the specified pin alternate function bits */
                    GPIO_AFSEL1(gpio_periph) &= ~GPIO_AFR_MASK(i - 8U);
                }
            }
        }
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the GPIO 
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F) 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[in]  p_init: GPIO intialization structure
                  mode: HAL_GPIO_MODE_ANALOG, HAL_GPIO_MODE_INPUT, HAL_GPIO_MODE_OUTPUT_PP
                        HAL_GPIO_MODE_OUTPUT_OD, HAL_GPIO_MODE_AF_PP, HAL_GPIO_MODE_AF_OD
                  pull: HAL_GPIO_PULL_NONE, HAL_GPIO_PULL_UP, HAL_GPIO_PULL_DOWN
                  ospeed: HAL_GPIO_OSPEED_2MHZ, HAL_GPIO_OSPEED_10MHZ, HAL_GPIO_OSPEED_50MHZ
                          HAL_GPIO_OSPEED_MAX
                  af: HAL_GPIO_AF_x(x=0...7))
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_gpio_init(uint32_t gpio_periph, uint32_t pin, hal_gpio_init_struct *p_init)
{
    uint32_t i;
    uint32_t reg_temp;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check pin_init address */
    if(NULL == p_init){
        HAL_DEBUGE("pointer [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check gpio_periph value */
    if((GPIOA != gpio_periph) && (GPIOB != gpio_periph) && (GPIOC != gpio_periph) && (GPIOF != gpio_periph)){
        HAL_DEBUGE("parameter [gpio_periph] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check if pin is PA0 ~ PA15/PB0 ~ PB15 or not*/
    if((GPIOA == gpio_periph) || (GPIOB == gpio_periph)){
        if((0U != (pin & _GPIO_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;}
    }
    
    /* check if pin is PC13 ~ PC15 or not*/
    if((GPIOC == gpio_periph)){
        if((0U != (pin & _GPIOC_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;}
    }
    
    /* check if pin is PF0 ~ PF1/PF6 ~ PF7 or not*/
    if((GPIOF == gpio_periph)){
        if((0U != (pin & _GPIOF_PIN_VALUE_MASK))){
            HAL_DEBUGE("parameter [pin] value is invalid");
            return HAL_ERR_VAL;}
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    for(i = 0U;i < 16U;i++){
        if((1U << i) & pin){
            /* set GPIO_CTL register */
            reg_temp = GPIO_CTL(gpio_periph);
            /* clear the specified pin mode bits */
            reg_temp &= ~GPIO_MODE_MASK(i);
            /* set the specified pin mode bits */
            reg_temp |= GPIO_MODE_SET(i, (CTL_CLTR(3) & p_init->mode));
            GPIO_CTL(gpio_periph) = reg_temp;
            
            /* set GPIO_OMODE register */
            if(GPIO_OTYPE_OD == _GPIO_GET_OUTPUT_MODE(p_init->mode)){
                GPIO_OMODE(gpio_periph) |= (uint32_t)pin;
            }else{
                GPIO_OMODE(gpio_periph) &= (uint32_t)(~pin);
            }
            
            /* set GPIO_OMODE register */
            reg_temp = GPIO_OSPD(gpio_periph);
            reg_temp  &= ~GPIO_OSPEED_MASK(i);
            reg_temp |= GPIO_OSPEED_SET(i, p_init->ospeed);
            GPIO_OSPD(gpio_periph) = reg_temp;
            
            /* set GPIO_PUD register */  
            reg_temp = GPIO_PUD(gpio_periph);
            reg_temp &= ~GPIO_PUPD_MASK(i);
            reg_temp |= GPIO_PUPD_SET(i, p_init->pull);
            GPIO_PUD(gpio_periph) = reg_temp;
            
            /* set GPIO_AFSELx register */ 
            if((HAL_GPIO_MODE_AF_PP == p_init->mode) || (HAL_GPIO_MODE_AF_OD == p_init->mode)){
                if(i < 8){
                    /* set pin0 to pin7 alternate function */
                    reg_temp = GPIO_AFSEL0(gpio_periph);
                    reg_temp &= ~GPIO_AFR_MASK(i);
                    reg_temp |= GPIO_AFR_SET(i, p_init->af);
                    GPIO_AFSEL0(gpio_periph) = reg_temp;
                }else{
                    /* set pin8 to pin15 alternate function */
                    reg_temp = GPIO_AFSEL1(gpio_periph);
                    reg_temp &= ~GPIO_AFR_MASK(i - 8U);
                    reg_temp |= GPIO_AFR_SET(i - 8U, p_init->af);
                    GPIO_AFSEL1(gpio_periph) = reg_temp;
                }
            }
        }
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      set GPIO pin bit
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[out] none
    \retval     none
*/
void hal_gpio_bit_set(uint32_t gpio_periph, uint32_t pin)
{
    GPIO_BOP(gpio_periph) = (uint32_t)pin;
}

/*!
    \brief      reset GPIO pin bit
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F) 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[out] none
    \retval     none
*/
void hal_gpio_bit_reset(uint32_t gpio_periph, uint32_t pin)
{
    GPIO_BC(gpio_periph) = (uint32_t)pin;
}

/*!
    \brief      write data to the specified GPIO pin
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[in]  bit_value: SET or RESET
      \arg        RESET: clear the port pin
      \arg        SET: set the port pin
    \param[out] none
    \retval     none
*/
void hal_gpio_bit_write(uint32_t gpio_periph, uint32_t pin, bit_status bit_value)
{
    if(RESET != bit_value){
        GPIO_BOP(gpio_periph) = (uint32_t)pin;
    }else{
        GPIO_BC(gpio_periph) = (uint32_t)pin;
    }
}


/*!
    \brief      write data to the specified GPIO port
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  data: specify the value to be written to the port output control register
    \param[out] none
    \retval     none
*/
void hal_gpio_port_write(uint32_t gpio_periph, uint16_t data)
{
    GPIO_OCTL(gpio_periph) = (uint32_t)data;
}

/*!
    \brief      get GPIO pin input status
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                only one parameter can be selected which is shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[out] none
    \retval     SET or RESET
*/
FlagStatus hal_gpio_input_bit_get(uint32_t gpio_periph, uint32_t pin)
{
    if((uint32_t)RESET != (GPIO_ISTAT(gpio_periph)&(pin))){
        return SET; 
    }else{
        return RESET;
    }
}

/*!
    \brief      get GPIO all pins input status
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[out] none
    \retval     state of GPIO all pins
*/
uint16_t hal_gpio_input_port_get(uint32_t gpio_periph)
{
    return (uint16_t)GPIO_ISTAT(gpio_periph);
}

/*!
    \brief      get GPIO pin output status
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                only one parameter can be selected which is shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[out] none
    \retval     SET or RESET
*/
FlagStatus hal_gpio_output_bit_get(uint32_t gpio_periph, uint32_t pin)
{
    if((uint32_t)RESET != (GPIO_OCTL(gpio_periph)&(pin))){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      get GPIO all pins output status
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[out] none
    \retval     state of GPIO all pins
*/
uint16_t hal_gpio_output_port_get(uint32_t gpio_periph)
{
    return (uint16_t)GPIO_OCTL(gpio_periph);
}

/*!
    \brief      lock GPIO pin bit
    \param[in]  gpio_periph: GPIOx(x = A,B)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[out] none
    \retval     none
*/
void hal_gpio_pin_lock(uint32_t gpio_periph, uint32_t pin)
{
    uint32_t lock = 0x00010000U;
    lock |= pin;

    /* lock key writing sequence: write 1->write 0->write 1->read 0->read 1 */
    GPIO_LOCK(gpio_periph) = (uint32_t)lock;
    GPIO_LOCK(gpio_periph) = (uint32_t)pin;
    GPIO_LOCK(gpio_periph) = (uint32_t)lock;
    lock = GPIO_LOCK(gpio_periph);
    lock = GPIO_LOCK(gpio_periph);
}

/*!
    \brief      toggle GPIO pin status
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL, PB9/PC13 does not exist on GD32E231
    \param[out] none
    \retval     none
*/
void hal_gpio_bit_toggle(uint32_t gpio_periph, uint32_t pin)
{
    GPIO_TG(gpio_periph) = (uint32_t)pin;
}

/*!
    \brief      toggle GPIO port status
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F)
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[out] none
    \retval     none
*/
void hal_gpio_port_toggle(uint32_t gpio_periph)
{
    GPIO_TG(gpio_periph) = 0x0000FFFFU;
}
