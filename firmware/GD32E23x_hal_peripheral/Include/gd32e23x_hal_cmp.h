/*!
    \file  gd32e23x_hal_cmp.h
    \brief definitions for the CMP

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

#ifndef GD32E23X_HAL_CMP_H
#define GD32E23X_HAL_CMP_H
#include "gd32e23x_hal.h"

/* the callback of comparator interrupt declaration */
typedef void (*hal_cmp_irq_handle_cb)(void *ptr);

/* noninverting input enum */
typedef enum{
    CMP_PA1 = 0,                                        /*!< PA1 input */
    CMP_PA4_SWCLOSE                                     /*!< PA4 input */
}noninverting_input_enum;

/* external trigger mode enum */
typedef enum{
    CMP_EXTI_NONE = 0,                                  /*!< no exti interrupt or envnt trigger */
    CMP_EXTI_INT_RISING = EXTI_INTERRUPT_TRIG_RISING,   /*!< exti interrupt with rising edge */
    CMP_EXTI_INT_FALLING = EXTI_INTERRUPT_TRIG_FALLING, /*!< exti interrupt with falling edge */
    CMP_EXTI_INT_BOTH = EXTI_INTERRUPT_TRIG_BOTH,       /*!< exti interrupt with both rising and falling edge */
    CMP_EXTI_EVENT_RISING = EXTI_EVENT_TRIG_RISING,     /*!< exti event with rising edge */
    CMP_EXTI_EVENT_FALLING = EXTI_EVENT_TRIG_FALLING,   /*!< exti event with falling edge */
    CMP_EXTI_EVENT_BOTH = EXTI_EVENT_TRIG_BOTH          /*!< exti event with both rising and falling edge */
}external_trigger_mode_enum;

/* comparator state enum */
typedef enum{
    CMP_STATE_NONE = 0,                                 /*!< NONE(default value) */
    CMP_STATE_RESET,                                    /*!< RESET */
    CMP_STATE_READY,                                    /*!< READY */
    CMP_STATE_LOCKED,                                   /*!< LOCKED */
    CMP_STATE_RUN                                       /*!< RUN */
}hal_cmp_state_enum;

/* comparator structure type enum */
typedef enum {
    HAL_CMP_INIT_STRUCT,                                /*!< the comparator initialization structure */
    HAL_CMP_DEV_STRUCT                                  /*!< the comparator device structure */
} hal_cmp_struct_type_enum;

/* comparator init structure */
typedef struct{
    inverting_input_enum inverting_input_select;        /*!< the comparator inverting input selection */
    noninverting_input_enum noninverting_input_select;  /*!< the comparator noninverting input selection */
    cmp_output_enum output_select;                      /*!< the comparator output selection */
    uint32_t output_polarity;                           /*!< the comparator output polarity */
    operating_mode_enum mode;                           /*!< the comparator operating mode */
    cmp_hysteresis_enum hysteresis_level;               /*!< the comparator hysteresis level */
    external_trigger_mode_enum exti_mode;               /*!< the comparator external trigger mode */
}hal_cmp_init_struct;

/* comparator device structure */
typedef struct{
    hal_cmp_irq_handle_cb cmp_irq_handle;               /*!< the comparator interrupt callback */
    hal_cmp_state_enum state;                           /*!< the comparator state */
    uint32_t output_level;                              /*!< the comparator output level */
}hal_cmp_dev_struct;

/* function declarations */
/* initialization functions */
/* comparator init struct deinitialize */
int32_t hal_cmp_struct_init(hal_cmp_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize the comparator device structure and reset the peripheral */
int32_t hal_cmp_deinit(hal_cmp_dev_struct *cmp);
/* initialize the comparator */
int32_t hal_cmp_init(hal_cmp_dev_struct *cmp, hal_cmp_init_struct *p_init);

/* enable or disable functions */
/* start cmp module function with event or not */
int32_t hal_cmp_start(hal_cmp_dev_struct *cmp);
/* stop cmp module function with event or not */
int32_t hal_cmp_stop(hal_cmp_dev_struct *cmp);
/* start cmp module function with interrupt */
int32_t hal_cmp_start_interrupt(hal_cmp_dev_struct *cmp, hal_cmp_irq_handle_cb irq_handler);
/* stop cmp module function with interrupt*/
int32_t hal_cmp_stop_interrupt(hal_cmp_dev_struct *cmp);

/* interrupt handle */
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_cmp_irq_handle_set(hal_cmp_dev_struct *cmp, hal_cmp_irq_handle_cb irq_handler);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_cmp_irq_handle_all_reset(hal_cmp_dev_struct *cmp);
/* cmp external trigger interrupt handler content function,which is merely used in ADC_CMP_IRQHandler */
void hal_cmp_irq(hal_cmp_dev_struct *cmp);

/* control functions */
/* lock the comparator */
int32_t hal_cmp_lock(hal_cmp_dev_struct *cmp);
/* get output level of comparator */
uint32_t hal_cmp_output_level_get(hal_cmp_dev_struct *cmp);
/* get the state of comparator */
hal_cmp_state_enum hal_cmp_state_get(hal_cmp_dev_struct *cmp);

#endif /* GD32E23X_HAL_CMP_H */
