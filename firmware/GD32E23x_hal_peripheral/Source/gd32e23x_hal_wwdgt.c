/*!
    \file    gd32e23x_hal_wwdgt.c
    \brief   WWDGT driver
    
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

static hal_wwdgt_irq_handle_cb early_wakeup_handle = NULL;

/*!
    \brief      initialize the parameters of WWDGT struct with the default values
    \param[in]  p_wwdgt_init: the initialization data needed to initialize WWDGT
    \param[out] none
    \retval     none
*/
void hal_wwdgt_struct_init(hal_wwdgt_init_struct *p_wwdgt_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_wwdgt_init){
        HAL_DEBUGE("parameter [*p_wwdgt_init] value is invalid");
    }    
#endif /* 1== HAL_PARAMETER_CHECK */
    
    p_wwdgt_init->prescaler = HAL_WWDGT_PCLK1_PSC_DIV1;
    p_wwdgt_init->counter = 0x7F;
    p_wwdgt_init->window_value = 0x7F;
}

/*!
    \brief      deinitialize WWDGT
    \param[in]  p_wwdgt_init: the initialization data needed to initialize WWDGT
    \param[out] none
    \retval     none
*/
void hal_wwdgt_deinit(void)
{  
    wwdgt_deinit();
}

/*!
    \brief      initialize WWDGT
    \param[in]  p_wwdgt_init: the initialization data needed to initialize WWDGT
                  prescaler: 
                    the argument could be selected from enumeration <hal_wwdgt_prescaler_enum>
                  counter: 0x40 - 0x7F
                  window_value: 0x40 - 0x7F 
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NONE,
                details refer to gd32e23x_hal.h
*/
int32_t hal_wwdgt_init(hal_wwdgt_init_struct *p_wwdgt_init)
{
    uint32_t reg_cfg = 0x0U, reg_ctl = 0x0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_wwdgt_init){
        HAL_DEBUGE("pointer [p_wwdgt_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    if((p_wwdgt_init->counter > 0x7F)||(p_wwdgt_init->counter < 0x40)){
        HAL_DEBUGE("parameter [p_wwdgt_init->counter] value is invalid");
        return HAL_ERR_VAL;
    }    
    
    if((p_wwdgt_init->window_value > 0x7F)||(p_wwdgt_init->window_value < 0x40)){
        HAL_DEBUGE("parameter [p_wwdgt_init->window_value] value is invalid");
        return HAL_ERR_VAL;
    } 
#endif /* 1== HAL_PARAMETER_CHECK */
    /* enable WWDGT clock */
    rcu_periph_clock_enable(RCU_WWDGT);
    
    /* clear WIN and PSC bits, clear CNT bit */
    reg_cfg = WWDGT_CFG &(~((uint32_t)WWDGT_CFG_WIN|(uint32_t)WWDGT_CFG_PSC));
    reg_ctl = WWDGT_CTL &(~(uint32_t)WWDGT_CTL_CNT);
  
    /* configure WIN and PSC bits, configure CNT bit */
    reg_cfg |= (uint32_t)(WWDGT_CFG_WIN & p_wwdgt_init->window_value);
    reg_cfg |= (uint32_t)(p_wwdgt_init->prescaler);
    reg_ctl |= (uint32_t)(WWDGT_CTL_CNT & p_wwdgt_init->counter);
    
    WWDGT_CFG = (uint32_t)reg_cfg;
    WWDGT_CTL = (uint32_t)reg_ctl;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start WWDGT module function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_wwdgt_start(void)
{
    WWDGT_CTL |= WWDGT_CTL_WDGTEN;
}

/*!
    \brief      start WWDGT module function by interrupt method, the function is non-blocking
    \param[in]  irq_handle: the callback handler of WWDGT interrupt
                  The member can be assigned as following parameters:
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
      \arg        hal_irq_handle_cb function pointer: the function is user customized,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_wwdgt_start_interrupt(hal_wwdgt_irq_handle_cb irq_handle)
{
    if(NULL != irq_handle){
        early_wakeup_handle = irq_handle;          
        WWDGT_CFG |= WWDGT_CFG_EWIE;  
    }else{
#if (1 == HAL_PARAMETER_CHECK)
        HAL_DEBUGE("pointer [irq_handle] address is invalid");
#endif /* 1== HAL_PARAMETER_CHECK */
        return HAL_ERR_ADDRESS;
    }

    WWDGT_CTL |= WWDGT_CTL_WDGTEN;
    return HAL_ERR_NONE;
}

/*!
    \brief      reload the counter of WWDGT
    \param[in]  p_wwdgt: the structure needed to configure WWDGT
                  prescaler: 
                    the argument could be selected from enumeration <hal_wwdgt_prescaler_enum>
                  counter: 0x40 - 0x7F
                  window_value: 0x40 - 0x7F  
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_wwdgt_feed(hal_wwdgt_init_struct *p_wwdgt)
{
    uint32_t reg = 0x0U;

#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_wwdgt){
        HAL_DEBUGE("pointer [p_wwdgt] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    if((p_wwdgt->counter > 0x7F)||(p_wwdgt->counter < 0x40)){
        HAL_DEBUGE("parameter [p_wwdgt->counter] value is invalid");
        return HAL_ERR_VAL;
    }   
#endif /* 1== HAL_PARAMETER_CHECK */
    
    reg = WWDGT_CTL &(~(uint32_t)WWDGT_CTL_CNT);
    reg |= (uint32_t)(WWDGT_CTL_CNT & p_wwdgt->counter);
    
    WWDGT_CTL = (uint32_t)reg;
    
    return HAL_ERR_NONE;    
}

/*!
    \brief      WWDGT interrupt handler content function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_wwdgt_irq(void)
{
    /* transmission complete interrupt handle */
    if(RESET != wwdgt_flag_get()){
        if(NULL != early_wakeup_handle){
            early_wakeup_handle();
        }          
        wwdgt_flag_clear();
        
      
    }
}    

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  irq_handle: the callback handler of WWDGT interrupt
                  The member can be assigned as following parameters:
      \arg        NULL: The corresponding callback mechanism is out of use, and the corresponding
                    interrupt is automatically disabled by hardware when condition is satisfied
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
      \arg        hal_irq_handle_cb function pointer: the function is user customized,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_wwdgt_irq_handle_set(hal_wwdgt_irq_handle_cb irq_handle)
{
    if(NULL != irq_handle){
        early_wakeup_handle = irq_handle;
        WWDGT_CFG |= WWDGT_CFG_EWIE;
    }else{
        early_wakeup_handle = NULL;
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_wwdgt_irq_handle_all_reset(void)
{
    early_wakeup_handle = NULL;
}
