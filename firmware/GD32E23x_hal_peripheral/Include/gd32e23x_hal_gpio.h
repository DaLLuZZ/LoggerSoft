/*!
    \file    gd32e23x_hal_gpio.h
    \brief   definitions for the GPIO
    
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

#ifndef GD32E23X_HAL_GPIO_H
#define GD32E23X_HAL_GPIO_H
#include "gd32e23x_hal.h"

/* GPIO */
/* GPIO mode definitions */
typedef enum {
    HAL_GPIO_MODE_ANALOG = (uint32_t)0x00000003U,                                         /*!< analog mode */
    HAL_GPIO_MODE_INPUT = (uint32_t)0x00000000U,                                          /*!< input floating mode */
    HAL_GPIO_MODE_OUTPUT_PP = (uint32_t)0x00000001U,                                      /*!< output push pull mode */
    HAL_GPIO_MODE_OUTPUT_OD = (uint32_t)0x00000011U,                                      /*!< output open drain mode */
    HAL_GPIO_MODE_AF_PP = (uint32_t)0x0000002U,                                           /*!< alternate function push pull mode */
    HAL_GPIO_MODE_AF_OD = (uint32_t)0x00000012U                                           /*!< alternate function open drain mode */
} hal_gpio_mode_enum;

/* pull-up/pull-down definitions */
typedef enum {
    HAL_GPIO_PULL_NONE = (uint32_t)0x00000000U,                                           /*!< floating mode, no pull-up and pull-down resistors */
    HAL_GPIO_PULL_UP,                                                                     /*!< with pull-up resistor */
    HAL_GPIO_PULL_DOWN                                                                    /*!< with pull-down resistor */
} hal_gpio_pull_enum;

/* GPIO output max speed value */
typedef enum {
    HAL_GPIO_OSPEED_2MHZ = (uint32_t)0x00000000U,                                         /*!< output max speed 2MHz */
    HAL_GPIO_OSPEED_10MHZ = (uint32_t)0x00000001U,                                        /*!< output max speed 10MHz */
    HAL_GPIO_OSPEED_50MHZ = (uint32_t)0x00000003U                                         /*!< output max speed 50MHz */
} hal_gpio_ospeed_enum;

/* GPIO alternate function */
typedef enum {
    HAL_GPIO_AF_0 = (uint32_t)0x00000000U,                                                /*!< alternate function 0 selected */
    HAL_GPIO_AF_1,                                                                        /*!< alternate function 1 selected */
    HAL_GPIO_AF_2,                                                                        /*!< alternate function 2 selected */
    HAL_GPIO_AF_3,                                                                        /*!< alternate function 3 selected */
    HAL_GPIO_AF_4,                                                                        /*!< alternate function 4 selected (port A,B only) */
    HAL_GPIO_AF_5,                                                                        /*!< alternate function 5 selected (port A,B only) */
    HAL_GPIO_AF_6,                                                                        /*!< alternate function 6 selected (port A,B only)*/
    HAL_GPIO_AF_7                                                                         /*!< alternate function 7 selected (port A,B only)*/
} hal_gpio_af_enum;

/* GPIO intialization structure */
typedef struct {
    uint32_t mode;      /* refer to hal_gpio_mode_enum */
    uint32_t pull;      /* refer to hal_gpio_pull_enum */
    uint32_t ospeed;    /* refer to hal_gpio_ospeed_enum */
    uint32_t af;        /* refer to hal_gpio_af_enum */
} hal_gpio_init_struct;

/* function declarations */
/* initialize the GPIO initialization structure with the default values */
int32_t hal_gpio_struct_init(hal_gpio_init_struct *p_init);
/* deinitialize GPIO */
int32_t hal_gpio_deinit(uint32_t gpio_periph, uint32_t pin);
/* initialize the GPIO */
int32_t hal_gpio_init(uint32_t gpio_periph, uint32_t pin, hal_gpio_init_struct *p_gpio_init);

/* set GPIO pin bit */
void hal_gpio_bit_set(uint32_t gpio_periph, uint32_t pin);
/* reset GPIO pin bit */
void hal_gpio_bit_reset(uint32_t gpio_periph, uint32_t pin);
/* write data to the specified GPIO pin */
void hal_gpio_bit_write(uint32_t gpio_periph, uint32_t pin, bit_status bit_value);
/* write data to the specified GPIO port */
void hal_gpio_port_write(uint32_t gpio_periph, uint16_t data);

/* get GPIO pin input status */
FlagStatus hal_gpio_input_bit_get(uint32_t gpio_periph, uint32_t pin);
/* get GPIO port input status */
uint16_t hal_gpio_input_port_get(uint32_t gpio_periph);
/* get GPIO pin output status */
FlagStatus hal_gpio_output_bit_get(uint32_t gpio_periph, uint32_t pin);
/* get GPIO port output status */
uint16_t hal_gpio_output_port_get(uint32_t gpio_periph);

/* lock GPIO pin bit */
void hal_gpio_pin_lock(uint32_t gpio_periph, uint32_t pin);
/* toggle GPIO pin status */
void hal_gpio_bit_toggle(uint32_t gpio_periph, uint32_t pin);
/* toggle GPIO port status */
void hal_gpio_port_toggle(uint32_t gpio_periph);

#endif /* GD32E23X_HAL_GPIO_H */
