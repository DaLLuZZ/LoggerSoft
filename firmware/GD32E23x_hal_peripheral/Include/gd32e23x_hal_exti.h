/*!
    \file    gd32e23x_hal_exti.h
    \brief   definitions for the EXTI
    
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

#ifndef GD32E23X_HAL_EXTI_H
#define GD32E23X_HAL_EXTI_H

#include "gd32e23x_hal.h"

#define EXTI_IRQ_EVENT_FALLING          (0x00000000)                        /*!< EXTI interrupt event falling edge */
#define EXTI_IRQ_EVENT_RISING           (0x00000001)                        /*!< EXTI interrupt event rising edge */

/* GPIO callback function */
typedef void (*hal_gpio_irq_handle_cb)(uint32_t id, uint32_t event);

/* EXTI trigger mode */
typedef enum {
    EXTI_EVENT_TRIG_RISING = 0x00000005U,
    EXTI_EVENT_TRIG_FALLING = 0x00000006U,
    EXTI_EVENT_TRIG_BOTH = 0x00000007U,
    EXTI_INTERRUPT_TRIG_RISING = 0x00000009U,
    EXTI_INTERRUPT_TRIG_FALLING = 0x0000000AU,
    EXTI_INTERRUPT_TRIG_BOTH = 0x0000000BU,
} hal_exti_type_enum;

/* EXTI line number */
typedef enum
{ 
    EXTI_PIN_0                      = BIT(0),           /*!< EXTI line 0 */
    EXTI_PIN_1                      = BIT(1),           /*!< EXTI line 1 */
    EXTI_PIN_2                      = BIT(2),           /*!< EXTI line 2 */
    EXTI_PIN_3                      = BIT(3),           /*!< EXTI line 3 */
    EXTI_PIN_4                      = BIT(4),           /*!< EXTI line 4 */
    EXTI_PIN_5                      = BIT(5),           /*!< EXTI line 5 */
    EXTI_PIN_6                      = BIT(6),           /*!< EXTI line 6 */
    EXTI_PIN_7                      = BIT(7),           /*!< EXTI line 7 */
    EXTI_PIN_8                      = BIT(8),           /*!< EXTI line 8 */
    EXTI_PIN_9                      = BIT(9),           /*!< EXTI line 9 */
    EXTI_PIN_10                     = BIT(10),          /*!< EXTI line 10 */
    EXTI_PIN_11                     = BIT(11),          /*!< EXTI line 11 */
    EXTI_PIN_12                     = BIT(12),          /*!< EXTI line 12 */
    EXTI_PIN_13                     = BIT(13),          /*!< EXTI line 13 */
    EXTI_PIN_14                     = BIT(14),          /*!< EXTI line 14 */
    EXTI_PIN_15                     = BIT(15),          /*!< EXTI line 15 */
    EXTI_LVD_16                     = BIT(16),          /*!< EXTI line 16 */
    EXTI_RTC_ALARM_17               = BIT(17),          /*!< EXTI line 17 */
    EXTI_USBFS_WAKEUP_18            = BIT(18),          /*!< EXTI line 18 */
    EXTI_RTC_TAMPER_TIMESTAMP_19    = BIT(19),          /*!< EXTI line 19 */
    EXTI_CMP_OUTPUT_21              = BIT(21),          /*!< EXTI line 21 */
    EXTI_USART0_WAKEUP_25           = BIT(25),          /*!< EXTI line 25 */
    EXTI_CEC_WAKEUP_27              = BIT(27),          /*!< EXTI line 27 */
}hal_exti_line_enum;

/* EXTI internal line number */
typedef enum {
    EXTI_LINE_16_LVD = BIT(16),
    EXTI_LINE_17_RTC_ALARM = BIT(17),
    EXTI_LINE_19_RTC_TAMPER_TIMESTAMP = BIT(19),
    EXTI_LINE_21_CMP_OUTPUT = BIT(21),
    EXTI_LINE_25_USART0_WAKEUP = BIT(25),
}hal_exti_internal_line_enum;

typedef enum{
    EXTI_0_1_IRQHandler_USED = 0x01U,
    EXTI_2_3_IRQHandler_USED = 0x23U,
    EXTI_4_15_IRQHandler_USED = 0x4fU,
}hal_exti_irq_index;

/* function declarations */

/* deinitialize the EXTI gpio */
int32_t hal_exti_gpio_deinit(uint32_t gpio_periph, uint32_t pin);
/* deinitialize the EXTI internal line */
void hal_exti_internal_deinit(hal_exti_internal_line_enum line);
/* initialize the configuration of EXTI gpio */
int32_t hal_exti_gpio_init(uint32_t gpio_periph, uint32_t pin, uint32_t pull, hal_exti_type_enum exti_type);
/* initialize the configuration of EXTI internal */
void hal_exti_internal_init(hal_exti_internal_line_enum line, hal_exti_type_enum exti_type);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
int32_t hal_exti_gpio_irq_handle_set(hal_gpio_irq_handle_cb irq_handler);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_exti_gpio_irq_handle_all_reset(void);
/* EXTI_GPIO interrupt handler content function,which is merely used in EXTI_GPIO_handler */
void hal_exti_gpio_irq(hal_exti_irq_index index);
/* activate the EXTI line software interrupt/event request */
void hal_exti_software_interrupt_trigger(hal_exti_line_enum linex);
#endif /* GD32E23X_HAL_EXTI_H */
