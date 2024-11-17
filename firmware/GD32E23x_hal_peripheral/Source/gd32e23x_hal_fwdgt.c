/*!
    \file    gd32e23x_hal_fwdgt.c
    \brief   FWDGT driver
    
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
    \brief      initialize the specified structure
    \param[in]  p_fwdgt_init: the initialization data needed to initialize FWDGT  
    \param[out] none
    \retval     none
*/
void hal_fwdgt_struct_init(hal_fwdgt_init_struct *p_fwdgt_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_fwdgt_init){
        HAL_DEBUGE("parameter [*p_fwdgt_init] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    p_fwdgt_init->prescaler = FWDGT_IRC40K_PSC_DIV256;
    p_fwdgt_init->reload = 0xFFF;
    p_fwdgt_init->window_value = 0xFFF;
}

/*!
    \brief      initialize FWDGT
    \param[in]  p_fwdgt_init: pointer to a hal_fwdgt_init_struct structure which contains 
                parameters for initialization of the FWDGT peripheral
                members of the structure and the member values are shown as below:
                  prescaler: 
                    the argument could be selected from enumeration <fwdgt_prescaler_enum>
                  reload: 0x0 - 0xFFF
                  window_value: 0x0 - 0xFFF, window function is disabled when the value is 0xFFF
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, HAL_ERR_NONE, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_fwdgt_init(hal_fwdgt_init_struct *p_fwdgt_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_fwdgt_init){
        HAL_DEBUGE("pointer [p_fwdgt_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    if(p_fwdgt_init->window_value < p_fwdgt_init->reload){
        HAL_DEBUGE("fwdgt window value is smaller than reload value, it will lead to system reset!");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* enable IRC40K */
    rcu_osci_on(RCU_IRC40K);
    /* wait till IRC40K is ready */
    rcu_osci_stab_wait(RCU_IRC40K);
    
    fwdgt_write_enable();
     
    if(ERROR == fwdgt_config(p_fwdgt_init->reload, (uint8_t)p_fwdgt_init->prescaler)){
        HAL_DEBUGE("fwdgt_config() timeout");
        return HAL_ERR_TIMEOUT;
    }
    
    if(ERROR == fwdgt_window_value_config(p_fwdgt_init->window_value)){
        HAL_DEBUGE("fwdgt_window_value_config() timeout");
        return HAL_ERR_TIMEOUT;
    }
    
    fwdgt_write_disable();
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start FWDGT module function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_fwdgt_start(void)
{
    FWDGT_CTL = FWDGT_KEY_ENABLE;
}

/*!
    \brief      reload the counter of FWDGT
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_fwdgt_feed(void)
{
    FWDGT_CTL = FWDGT_KEY_RELOAD;
}
