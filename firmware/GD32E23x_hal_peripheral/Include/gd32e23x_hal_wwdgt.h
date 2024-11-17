/*!
    \file    gd32e23x_hal_wwdgt.h
    \brief   definitions for the WWDGT
    
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

#ifndef GD32E23X_HAL_WWDGT_H
#define GD32E23X_HAL_WWDGT_H

#include "gd32e23x_hal.h"

/* constants definitions */
/* WWDGT_CFG_PSC bit field value */
typedef enum{
    HAL_WWDGT_PCLK1_PSC_DIV1 = WWDGT_CFG_PSC_DIV1,                   /*!< the time base of WWDGT = (PCLK1/4096)/1 */
    HAL_WWDGT_PCLK1_PSC_DIV2 = WWDGT_CFG_PSC_DIV2,                   /*!< the time base of WWDGT = (PCLK1/4096)/2 */
    HAL_WWDGT_PCLK1_PSC_DIV4 = WWDGT_CFG_PSC_DIV4,                   /*!< the time base of WWDGT = (PCLK1/4096)/4 */
    HAL_WWDGT_PCLK1_PSC_DIV8 = WWDGT_CFG_PSC_DIV8,                   /*!< the time base of WWDGT = (PCLK1/4096)/8 */
} hal_wwdgt_prescaler_enum;

/* WWDGT initialize structure */
typedef struct {          
    hal_wwdgt_prescaler_enum  prescaler;                             /*!< WWDGT_CFG_PSC bit field value */
    uint16_t                  counter;                               /*!< WWDGT_CTL_CNT bit field value */
    uint16_t                  window_value;                          /*!< WWDGT_CFG_WIN bit field value */
} hal_wwdgt_init_struct;

typedef void (*hal_wwdgt_irq_handle_cb)(void);

/* function declarations */
/* initialize the parameters of WWDGT struct with the default values */
void hal_wwdgt_struct_init(hal_wwdgt_init_struct *p_wwdgt_init);
/* deinitialize WWDGT */
void hal_wwdgt_deinit(void);

/* initialize WWDGT */
int32_t hal_wwdgt_init(hal_wwdgt_init_struct *p_wwdgt_init);
/* start WWDGT module function */
void hal_wwdgt_start(void);
/* start WWDGT module function by interrupt method */
/* the function is non-blocking */
int32_t hal_wwdgt_start_interrupt(hal_wwdgt_irq_handle_cb irq_handle);
/* reload the counter of WWDGT */
int32_t hal_wwdgt_feed(hal_wwdgt_init_struct *p_wwdgt);

/* WWDGT interrupt handler content function */
void hal_wwdgt_irq(void);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_wwdgt_irq_handle_set(hal_wwdgt_irq_handle_cb irq_handle);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_wwdgt_irq_handle_all_reset(void);

#endif /* GD32E23X_HAL_WWDGT_H */
