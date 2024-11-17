/*!
    \file    gd32e23x_hal_crc.h
    \brief   definitions for the CRC
    
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

#ifndef GD32E23X_HAL_CRC_H
#define GD32E23X_HAL_CRC_H

#include "gd32e23x_hal.h"

/* input data reverse function */
#define CRC_INPUT_REVERSE_NOT             CRC_INPUT_DATA_NOT                /*!< input data not reverse */
#define CRC_INPUT_REVERSE_BYTE            CRC_INPUT_DATA_BYTE               /*!< input data reversed by byte type */
#define CRC_INPUT_REVERSE_HALFWORD        CRC_INPUT_DATA_HALFWORD           /*!< input data reversed by half-word type */
#define CRC_INPUT_REVERSE_WORD            CRC_INPUT_DATA_WORD               /*!< input data reversed by word type */

/* size of polynomial function */
#define CRC_POLYNOMIAL_SIZE_32BIT         CRC_CTL_PS_32                     /*!< 32-bit polynomial for CRC calculation */
#define CRC_POLYNOMIAL_SIZE_16BIT         CRC_CTL_PS_16                     /*!< 16-bit polynomial for CRC calculation */
#define CRC_POLYNOMIAL_SIZE_8BIT          CRC_CTL_PS_8                      /*!< 8-bit polynomial for CRC calculation */
#define CRC_POLYNOMIAL_SIZE_7BIT          CRC_CTL_PS_7                      /*!< 7-bit polynomial for CRC calculation */

/* CRC calculation unit */
#define CRC_DEFAULT_POLYNOMIAL_VALUE      (0x04C11DB7U)                     /*!< CRC default polynomial value */
#define CRC_DEFAULT_INIT_DATA_VALUE       (0xFFFFFFFFU)                     /*!< CRC default initialize value */

/* structure for initialization of the CRC */
typedef struct {          
    ControlStatus output_reverse;                                           /*!< CRC reverse operation of output */
    uint32_t      input_reverse;                                            /*!< CRC reverse operation of input */
    uint32_t      polynomial_size;                                          /*!< CRC size of polynomial function */
    uint32_t      polynomial;                                               /*!< CRC polynomial function */
    uint32_t      initdata;                                                 /*!< CRC initialize data */
} hal_crc_init_struct;

/* function declarations */
/* initialize the CRC structure with the default values */
int32_t hal_crc_struct_init(hal_crc_init_struct *p_crc_init);
/* deinitialize CRC calculation unit */
void hal_crc_deinit(void);
/* initialize CRC */
int32_t hal_crc_init(hal_crc_init_struct *p_crc_init);

/* write the free data register */
void hal_crc_free_data_write(uint8_t data);
/* read the free data register */
uint8_t hal_crc_free_data_read(void);
/* reset data register to the value of initializaiton data register */
void hal_crc_calculate_reset(void);
/* CRC calculate a 32-bit data array */
uint32_t hal_crc_calculate(uint32_t *p_buffer, uint32_t size);

#endif /* GD32E23X_HAL_CRC_H */
