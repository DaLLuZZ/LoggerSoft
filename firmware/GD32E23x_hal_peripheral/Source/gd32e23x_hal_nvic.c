/*!
    \file    gd32e23x_hal_nvic.c
    \brief   NVIC driver
    
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
    \brief      set NVIC request priority
    \param[in]  irqn: the NVIC interrupt request
      \arg        the argument could be selected from enumeration <IRQn_Type>
    \param[in]  priority: priority to set
      \arg        0-3(0 is highest priority)
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_nvic_irq_priority_set(IRQn_Type irqn, uint8_t priority)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check priority value */
    if(priority >= ((uint32_t)0x00000001 << __NVIC_PRIO_BITS)){
        HAL_DEBUGE("parameter [priority] value is out of range");
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* set the priority */
    NVIC_SetPriority(irqn, priority);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      get NVIC request priority
    \param[in]  irqn: the NVIC interrupt request
      \arg        the argument could be selected from enumeration <IRQn_Type>
    \param[out] none
    \retval     interrupt priority.
                value is aligned automatically to the implemented priority bits of the microcontroller.
*/
uint32_t hal_nvic_irq_priority_get(IRQn_Type irqn)
{
    return NVIC_GetPriority(irqn);
}

/*!
    \brief      reset NVIC system
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_nvic_system_reset(void)
{
    /* System Reset */
    NVIC_SystemReset();
}

/*!
    \brief      enable peripherals NVIC request
    \param[in]  irqn: the NVIC interrupt request
      \arg        the argument could be selected from enumeration <IRQn_Type>
    \param[in]  priority: priority to set
      \arg        0-3(0 is highest priority)
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_nvic_periph_irq_enable(IRQn_Type irqn, uint8_t priority)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check priority value */
    if(priority >= ((uint32_t)0x00000001 << __NVIC_PRIO_BITS)){
        HAL_DEBUGE("parameter [priority] value is out of range");
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* set the priority and enable the selected IRQ */
    NVIC_SetPriority(irqn, priority);
    NVIC_EnableIRQ(irqn);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      disable NVIC request
    \param[in]  irqn: the NVIC interrupt request
      \arg        the argument could be selected from enumeration <IRQn_Type>
    \param[out] none
    \retval     none
*/
void hal_nvic_periph_irq_disable(IRQn_Type irqn)
{
    /* disable the selected IRQ */
    NVIC_DisableIRQ(irqn);
}

/*!
    \brief      set the NVIC vector table base address
    \param[in]  nvic_vict_tab: the RAM or FLASH base address
                only one parameter can be selected which is shown as below:
      \arg        NVIC_VECTTAB_RAM: RAM base address
      \arg        NVIC_VECTTAB_FLASH: FLASH base address
    \param[in]  offset: vector table offset
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_nvic_vector_table_set(uint32_t nvic_vict_tab, uint32_t offset)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check nvic_vict_tab value */
    if((nvic_vict_tab == NVIC_VECTTAB_RAM) && offset >= ((uint32_t)0x00001BFF)){
        HAL_DEBUGE("parameter [offset] value is out of range");
        return HAL_ERR_VAL;
    }
    
    /* check nvic_vict_tab value */
    if((nvic_vict_tab == NVIC_VECTTAB_FLASH) && offset >= ((uint32_t)0x0000FBFF)){
        HAL_DEBUGE("parameter [offset] value is out of range");
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    SCB->VTOR = nvic_vict_tab | (offset & NVIC_VECTTAB_OFFSET_MASK);
    
    return HAL_ERR_NONE;
}
