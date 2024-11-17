/*!
    \file    gd32e23x_hal_crc.c
    \brief   CRC driver
    
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

/*!
    \brief      initialize the CRC structure with the default values
    \param[in]  p_crc_init: point to the structure to be deinitialized
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_crc_struct_init(hal_crc_init_struct *p_crc_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check p_crc_init address */
    if(NULL == p_crc_init){
        HAL_DEBUGE("pointer [p_crc_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    p_crc_init->input_reverse = CRC_INPUT_REVERSE_NOT;
    p_crc_init->output_reverse = DISABLE;
    p_crc_init->polynomial_size = CRC_POLYNOMIAL_SIZE_32BIT;
    p_crc_init->polynomial = CRC_DEFAULT_POLYNOMIAL_VALUE;
    p_crc_init->initdata = CRC_DEFAULT_INIT_DATA_VALUE;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize CRC calculation unit
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_crc_deinit(void)
{
    CRC_IDATA = (uint32_t)0xFFFFFFFFU;
    CRC_DATA  = (uint32_t)0xFFFFFFFFU;
    CRC_FDATA = (uint32_t)0x00000000U;
    CRC_POLY  = (uint32_t)0x04C11DB7U;
    CRC_CTL   = CRC_CTL_RST;
}

/*!
    \brief      initialize CRC
    \param[in]  p_crc_init: specify input data reverse function
      \arg        output_reverse: ENABLE / DISABLE
      \arg        input_reverse: CRC_INPUT_REVERSE_NOT, CRC_INPUT_REVERSE_BYTE
                                 CRC_INPUT_REVERSE_HALFWORD, CRC_INPUT_REVERSE_WORD
      \arg        polynomial_size: CRC_POLYNOMIAL_SIZE_32BIT, CRC_POLYNOMIAL_SIZE_16BIT
                                   CRC_POLYNOMIAL_SIZE_8BIT, CRC_POLYNOMIAL_SIZE_7BIT
      \arg        polynomial: CRC_DEFAULT_POLYNOMIAL_VALUE(if don't care this parameter)
      \arg        initdata: CRC_DEFAULT_INIT_DATA_VALUE(if don't care this parameter)
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_crc_init(hal_crc_init_struct *p_crc_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check p_crc_init address */
    if(NULL == p_crc_init){
        HAL_DEBUGE("pointer [p_crc_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* reset crc data register */
    crc_data_register_reset();

    CRC_IDATA = p_crc_init->initdata;
    CRC_POLY = p_crc_init->polynomial;

    /* enable the reverse operation of output data */
    if(ENABLE == p_crc_init->output_reverse){
        CRC_CTL |= (uint32_t)CRC_CTL_REV_O;
    }else{
        CRC_CTL &= (uint32_t)(~ CRC_CTL_REV_O);
    }

    /* configure the CRC input data function */
    crc_input_data_reverse_config(p_crc_init->input_reverse);
    /* configure the CRC size of polynomial function */
    crc_polynomial_size_set(p_crc_init->polynomial_size);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      write the free data register
    \param[in]  data: specify 8-bit data
    \param[out] none
    \retval     none
*/
void hal_crc_free_data_write(uint8_t data)
{
    CRC_FDATA = (uint32_t)data;
}

/*!
    \brief      read the free data register
    \param[in]  none
    \param[out] none
    \retval     8-bit value of the free data register
*/
uint8_t hal_crc_free_data_read(void)
{
    uint8_t fdata;
    fdata = (uint8_t)CRC_FDATA;
    return (fdata);
}

/*!
    \brief      reset data register to the value of initializaiton data register
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_crc_calculate_reset(void)
{
    CRC_CTL |= (uint32_t)CRC_CTL_RST;
}

/*!
    \brief      CRC calculate a 32-bit data array
    \param[in]  p_buffer: pointer to an array of 32 bit data words
    \param[in]  size: size of the array
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
    \retval     32-bit CRC calculate value
*/
uint32_t hal_crc_calculate(uint32_t *p_buffer, uint32_t size)
{
    uint32_t index;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check p_buffer address */
    if(NULL == p_buffer){
        HAL_DEBUGE("pointer [p_buffer] address is invalid");
        while(1);
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    for(index = 0U; index < size; index++){
        CRC_DATA = p_buffer[index];
    }
    return (CRC_DATA); 
}
