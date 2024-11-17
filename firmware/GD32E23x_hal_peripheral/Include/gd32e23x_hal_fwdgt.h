/*!
    \file    gd32e23x_hal_fwdgt.h
    \brief   definitions for the FWDGT
    
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

#ifndef GD32E23X_HAL_FWDGT_H
#define GD32E23X_HAL_FWDGT_H

#include "gd32e23x_hal.h"

/* FWDGT_PSC register value */
typedef enum{
    FWDGT_IRC40K_PSC_DIV4 = FWDGT_PSC_DIV4,              /*!< FWDGT prescaler set to 4 */
    FWDGT_IRC40K_PSC_DIV8 = FWDGT_PSC_DIV8,              /*!< FWDGT prescaler set to 8 */
    FWDGT_IRC40K_PSC_DIV16 = FWDGT_PSC_DIV16,            /*!< FWDGT prescaler set to 16 */
    FWDGT_IRC40K_PSC_DIV32 = FWDGT_PSC_DIV32,            /*!< FWDGT prescaler set to 32 */
    FWDGT_IRC40K_PSC_DIV64 = FWDGT_PSC_DIV64,            /*!< FWDGT prescaler set to 64 */
    FWDGT_IRC40K_PSC_DIV128 = FWDGT_PSC_DIV128,          /*!< FWDGT prescaler set to 128 */
    FWDGT_IRC40K_PSC_DIV256 = FWDGT_PSC_DIV256,          /*!< FWDGT prescaler set to 256 */
} fwdgt_prescaler_enum;

/* structure for initialization of the FWDGT */
typedef struct {          
    fwdgt_prescaler_enum  prescaler;                     /*!< FWDGT prescaler divider value */
    uint16_t              reload;                        /*!< FWDGT counter reload value */
    uint16_t              window_value;                  /*!< FWDGT counter window value */
} hal_fwdgt_init_struct;

/* function declarations */
/* initialize the specified structure */
void hal_fwdgt_struct_init(hal_fwdgt_init_struct *p_fwdgt_init);
/* initialize FWDGT */
int32_t hal_fwdgt_init(hal_fwdgt_init_struct *p_fwdgt_init);
/* start FWDGT module function */
void hal_fwdgt_start(void);
/* reload the counter of FWDGT */
void hal_fwdgt_feed(void);

#endif /* GD32E23X_HAL_FWDGT_H */
