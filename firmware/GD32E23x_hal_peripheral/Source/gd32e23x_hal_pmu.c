/*!
    \file    gd32e23x_hal_pmu.c
    \brief   PMU driver
    
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

#define CLOCK_SWITCH_TIMEOUT             ((uint32_t)1000uL)      /*!< timeout value for clock switch */
#define PLL_LOCK_TIMEOUT                 ((uint32_t)3000uL)      /*!< timeout value for pll */

static hal_pmu_irq_handle_cb low_voltage_handle = NULL;

/*!
    \brief      initialize the specified structure
    \param[in]  p_struct: point to the structure to be deinitialized
    \param[out] none
    \retval     none
*/
void hal_pmu_struct_init(void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    ((hal_pmu_lvd_struct*)p_struct)->int_event_mode = HAL_PMU_INT_MODE;
    ((hal_pmu_lvd_struct*)p_struct)->trig_type = HAL_PMU_TRIG_RISING;
    ((hal_pmu_lvd_struct*)p_struct)->lvd_threshold = HAL_PMU_LVDT_0; 
}

/*!
    \brief      deinitialize PMU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_deinit(void)
{
    /* reset PMU */
    rcu_periph_reset_enable(RCU_PMURST);
    rcu_periph_reset_disable(RCU_PMURST);
}

/*!
    \brief      configure EXTI_16 and then configure low voltage detector threshold
    \param[in]  p_lvd: the data needed to configure LVD
                  int_event_mode: HAL_PMU_INT_MODE, HAL_PMU_EVENT_MODE
                  trig_type: HAL_PMU_TRIG_RISING, HAL_PMU_TRIG_FALLING, HAL_PMU_TRIG_BOTH
                  lvd_threshold: 
                    the argument could be selected from enumeration <hal_pmu_low_voltage_enum>
    \param[out] none
    \retval     none
*/
void hal_pmu_lvd_config(hal_pmu_lvd_struct *p_lvd)
{
    uint32_t reg = 0U;

#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_lvd){
        HAL_DEBUGE("pointer [p_lvd] address is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    hal_exti_internal_deinit(EXTI_LINE_16_LVD);
    
    /* set the EXTI mode and enable the interrupts or events from EXTI line 16 */
    switch(p_lvd->int_event_mode){
    case HAL_PMU_INT_MODE:
        EXTI_INTEN |= (uint32_t)EXTI_16;
        break;
    case HAL_PMU_EVENT_MODE:
        EXTI_EVEN |= (uint32_t)EXTI_16;
        break;
    default:
        HAL_DEBUGE("parameter [p_lvd->int_event_mode] value is invalid");
        break;
    }

    /* set the EXTI trigger type */
    switch(p_lvd->trig_type){
    case HAL_PMU_TRIG_RISING:
        EXTI_RTEN |= (uint32_t)EXTI_16;
        EXTI_FTEN &= ~(uint32_t)EXTI_16;
        break;
    case HAL_PMU_TRIG_FALLING:
        EXTI_RTEN &= ~(uint32_t)EXTI_16;
        EXTI_FTEN |= (uint32_t)EXTI_16;
        break;
    case HAL_PMU_TRIG_BOTH:
        EXTI_RTEN |= (uint32_t)EXTI_16;
        EXTI_FTEN |= (uint32_t)EXTI_16;
        break;
    default:
        HAL_DEBUGE("parameter [p_lvd->trig_type] value is invalid");
        break;
    }
    
    reg = PMU_CTL;
    /* clear LVDT bits */
    reg &= ~PMU_CTL_LVDT;
    /* set LVDT bits according to lvdt_n */
    reg |= p_lvd->lvd_threshold;
    PMU_CTL = reg;
}

/*!
    \brief      enable PMU lvd
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_lvd_enable(void)
{
    /* enable LVD */
    PMU_CTL |= PMU_CTL_LVDEN;
}

/*!
    \brief      enable PMU lvd by interrupt method, the function is non-blocking
    \param[in]  irq_handle: the callback handler of PMU interrupt
                  The structure member can be assigned as following parameters:
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
      \arg        hal_irq_handle_cb function pointer: the function is user customized,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_pmu_lvd_enable_interrupt(hal_pmu_irq_handle_cb irq_handle)
{
    /* enable interrupt mode */
    if(NULL != irq_handle){
        low_voltage_handle = irq_handle;
        EXTI_INTEN |= (uint32_t)EXTI_16;
        EXTI_PD = EXTI_16;
    }else{
        HAL_DEBUGE("pointer [irq_handle] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* enable LVD */   
    PMU_CTL |= PMU_CTL_LVDEN;
    return HAL_ERR_NONE;
}

/*!
    \brief      disable PMU lvd
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_lvd_disable(void)
{
    /* disable LVD */
    PMU_CTL &= ~PMU_CTL_LVDEN;
    
    EXTI_INTEN &= (uint32_t)~EXTI_16;    
    low_voltage_handle = NULL;     
    exti_interrupt_flag_clear(EXTI_16);
}

/*!
    \brief      configure LDO output voltage
                these bits set by software when the main PLL closed
    \param[in]  ldo_output:
                only one parameter can be selected which is shown as below:
      \arg        HAL_PMU_LDO_VOLTAGE_HIGH: LDO output voltage high mode
      \arg        HAL_PMU_LDO_VOLTAGE_LOW: LDO output voltage low mode
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_TIMEOUT, HAL_ERR_NONE,
                details refer to gd32e23x_hal.h
*/
int32_t hal_pmu_ldo_output_config(uint32_t ldo_output)
{
    uint32_t reg =0U, source = 0U, pll_en = 0U;
    uint32_t timeout = CLOCK_SWITCH_TIMEOUT, pll_timeout = PLL_LOCK_TIMEOUT;    

#if (1 == HAL_PARAMETER_CHECK)
    if((HAL_PMU_LDO_VOLTAGE_HIGH != ldo_output) && (HAL_PMU_LDO_VOLTAGE_LOW != ldo_output)){
        HAL_DEBUGE("parameter [ldo_output] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    rcu_periph_clock_enable(RCU_PMU);

    source = rcu_system_clock_source_get();
    pll_en = RCU_CTL0 & RCU_CTL0_PLLEN;
    
    /* if pll is the system clock source, switch clock to IRC8M */
    if(RCU_SCSS_PLL == source){
        rcu_osci_on(RCU_IRC8M);
        if(SUCCESS != rcu_osci_stab_wait(RCU_IRC8M)){
            HAL_DEBUGE("turn on RCU_IRC8M timeout");
            return HAL_ERR_TIMEOUT;
        }
        
        rcu_system_clock_source_config(RCU_CKSYSSRC_IRC8M);
        while((RCU_SCSS_IRC8M != rcu_system_clock_source_get()) && (0U != timeout)){
            --timeout;
        }
    
        if(0U == timeout){
            HAL_DEBUGE("configure IRC8M as system clock source timeout");
            return HAL_ERR_TIMEOUT;
        }    
    }
    
    /* turn off PLL */
    rcu_osci_off(RCU_PLL_CK);
    if((RESET != rcu_flag_get(RCU_FLAG_PLLSTB)) && (0U != pll_timeout)){
        HAL_DEBUGE("turn off RCU_PLL_CK timeout");
        return HAL_ERR_TIMEOUT;
    }  
    
    /* configure LDO output voltage */
    reg =  PMU_CTL;
    reg &= ~PMU_CTL_LDOVS;
    reg |= ldo_output;
    PMU_CTL = reg;
    
    /* recover the pll state */
    if(0U != pll_en){
        /* turn on PLL */
        rcu_osci_on(RCU_PLL_CK);
        if((SET != rcu_flag_get(RCU_FLAG_PLLSTB)) && (0U != pll_timeout)){
            HAL_DEBUGE("turn on RCU_PLL_CK timeout");
            return HAL_ERR_TIMEOUT;
        }
        
        rcu_system_clock_source_config(RCU_CKSYSSRC_PLL);
        while((RCU_SCSS_PLL != rcu_system_clock_source_get()) && (0U != timeout)){
            --timeout;
        }
    
        if(0U == timeout){
            HAL_DEBUGE("switch PLL as system clock source timeout");
            return HAL_ERR_TIMEOUT;
        }
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      get LDO output voltage
    \param[in]  none
    \param[out] none
    \retval     LDO_HIGH_VOLTAGE_MODE or LDO_LOW_VOLTAGE_MODE
*/
hal_pmu_ldo_mode_enum hal_pmu_ldo_output_get(void)
{
    if(SET == (PMU_CTL & PMU_CTL_LDOVS_1)){
        return LDO_LOW_VOLTAGE_MODE;
    }else{
        return LDO_HIGH_VOLTAGE_MODE;
    }
}

/*!
    \brief      PMU work at sleep mode
    \param[in]  sleepmodecmd:
                only one parameter can be selected which is shown as below:
      \arg        HAL_WFI_CMD: use WFI command
      \arg        HAL_WFE_CMD: use WFE command
    \param[out] none
    \retval     none
*/
void hal_pmu_to_sleepmode(uint8_t sleepmodecmd)
{
#if (1 == HAL_PARAMETER_CHECK)
    if((HAL_WFI_CMD != sleepmodecmd) && (HAL_WFE_CMD != sleepmodecmd)){
        HAL_DEBUGE("parameter [sleepmodecmd] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* clear sleepdeep bit of Cortex-M23 system control register */
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
    
    /* select WFI or WFE command to enter sleep mode */
    if(HAL_WFI_CMD == sleepmodecmd){
        __WFI();
    }else{
        __WFE();
    }
}

/*!
    \brief      PMU work at deepsleep mode
    \param[in]  ldo:
                only one parameter can be selected which is shown as below:
      \arg        HAL_PMU_LDO_NORMAL: LDO operates normally when pmu enter deepsleep mode
      \arg        HAL_PMU_LDO_LOWPOWER: LDO work at low power mode when pmu enter deepsleep mode
    \param[in]  deepsleepmodecmd:
                only one parameter can be selected which is shown as below:
      \arg        HAL_WFI_CMD: use WFI command
      \arg        HAL_WFE_CMD: use WFE command
    \param[out] none
    \retval     none
*/
void hal_pmu_to_deepsleepmode(uint32_t ldo, uint8_t deepsleepmodecmd)
{
    static uint32_t reg_snap[ 3 ];

#if (1 == HAL_PARAMETER_CHECK)
    if((HAL_PMU_LDO_NORMAL != ldo) && (HAL_PMU_LDO_LOWPOWER != ldo)){
        HAL_DEBUGE("parameter [ldo] value is invalid");
        return;    
    }
    if((HAL_WFI_CMD != deepsleepmodecmd) && (HAL_WFE_CMD != deepsleepmodecmd)){
        HAL_DEBUGE("parameter [deepsleepmodecmd] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* clear stbmod and ldolp bits */
    PMU_CTL &= ~((uint32_t)(PMU_CTL_STBMOD | PMU_CTL_LDOLP));
    
    /* set ldolp bit according to pmu_ldo */
    PMU_CTL |= ldo;
    
    /* set sleepdeep bit of Cortex-M23 system control register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    reg_snap[ 0 ] = REG32( 0xE000E010U );
    reg_snap[ 1 ] = REG32( 0xE000E100U );
    reg_snap[ 2 ] = REG32( 0xE000E104U );
    
    REG32( 0xE000E010U ) &= 0x00010004U;
    REG32( 0xE000E180U )  = 0XF7FFEF19U;
    REG32( 0xE000E184U )  = 0XFFFFFFFFU;
  
    /* select WFI or WFE command to enter deepsleep mode */
    if(HAL_WFI_CMD == deepsleepmodecmd){
        __WFI();
    }else{
        __SEV();
        __WFE();
        __WFE();
    }

    REG32( 0xE000E010U ) = reg_snap[ 0 ] ; 
    REG32( 0xE000E100U ) = reg_snap[ 1 ] ;
    REG32( 0xE000E104U ) = reg_snap[ 2 ] ;
    
    /* reset sleepdeep bit of Cortex-M23 system control register */
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
}

/*!
    \brief      pmu work at standby mode
    \param[in]  standbymodecmd:
                only one parameter can be selected which is shown as below:
      \arg        HAL_WFI_CMD: use WFI command
      \arg        HAL_WFE_CMD: use WFE command
    \param[out] none
    \retval     none
*/
void hal_pmu_to_standbymode(uint8_t standbymodecmd)
{
#if (1 == HAL_PARAMETER_CHECK)
    if((HAL_WFI_CMD != standbymodecmd) && (HAL_WFE_CMD != standbymodecmd)){
        HAL_DEBUGE("parameter [standbymodecmd] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* set sleepdeep bit of Cortex-M23 system control register */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* set stbmod bit */
    PMU_CTL |= PMU_CTL_STBMOD;
        
    /* reset wakeup flag */
    PMU_CTL |= PMU_CTL_WURST;
    
    /* select WFI or WFE command to enter standby mode */
    if(HAL_WFI_CMD == standbymodecmd){
        __WFI();
    }else{
        __WFE();
    }
}

/*!
    \brief      enable wakeup pin
    \param[in]  wakeup_pin: the argument could be selected from enumeration <hal_pmu_wakeup_pin_enum>
    \param[out] none
    \retval     none
*/
void hal_pmu_wakeup_pin_enable(hal_pmu_wakeup_pin_enum wakeup_pin)
{
    PMU_CS |= (uint16_t)wakeup_pin;
}

/*!
    \brief      disable wakeup pin
    \param[in]  wakeup_pin: the argument could be selected from enumeration <hal_pmu_wakeup_pin_enum>
    \param[out] none
    \retval     none
*/
void hal_pmu_wakeup_pin_disable(hal_pmu_wakeup_pin_enum wakeup_pin)
{
    PMU_CS &= ~(uint16_t)(wakeup_pin);
}

/*!
    \brief      enable backup domain write
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_backup_write_enable(void)
{
    PMU_CTL |= PMU_CTL_BKPWEN;
}

/*!
    \brief      disable backup domain write
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_backup_write_disable(void)
{
    PMU_CTL &= ~PMU_CTL_BKPWEN;
}

/*!
    \brief      enable sleep-on-exit: if the SLEEPONEXIT bit is set, the MCU enters Sleep mode
                as soon as it exits from the lowest priority ISR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_sleep_on_exit_enable(void)
{
    SCB->SCR |= (uint32_t)SCB_SCR_SLEEPONEXIT_Msk;
}

/*!
    \brief      disable sleep-on-exit: if the SLEEPONEXIT bit is set, the MCU enters Sleep mode
                as soon as it exits from the lowest priority ISR
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_sleep_on_exit_disable(void)
{
    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPONEXIT_Msk);
} 

/*!
    \brief      enable SEVONPEND: if the SEVONPEND bit is set, an event will be sent when an interrupt
                disabled in NVIC is pending, it's often used to wakeup system for WFE command from a pending interrupt 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_sev_on_pend_enable(void)
{
    SCB->SCR |= (uint32_t)SCB_SCR_SEVONPEND_Msk;
}

/*!
    \brief      disable SEVONPEND: if the SEVONPEND bit is set, an event will be sent when an interrupt
                disabled in NVIC is pending, it's often used to wakeup system for WFE command from a pending interrupt 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_sev_on_pend_disable(void)
{
    SCB->SCR &= ~((uint32_t)SCB_SCR_SEVONPEND_Msk);
}

/*!
    \brief      PMU interrupt handler content function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_irq(void)
{
    /* LVD interrupt handle */
    if(RESET != exti_interrupt_flag_get(EXTI_16)){
        exti_interrupt_flag_clear(EXTI_16);
        
        if(NULL != low_voltage_handle){
            low_voltage_handle();
        }        
    }
}    

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  irq_handle: the callback handler of PMU interrupt
                  The member can be assigned as following parameters:
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
      \arg        hal_irq_handle_cb function pointer: the function is user customized,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_pmu_irq_handle_set(hal_pmu_irq_handle_cb irq_handle)
{
    if(NULL != irq_handle){
        low_voltage_handle = irq_handle;
        EXTI_INTEN |= (uint32_t)EXTI_16;
    }else{
        low_voltage_handle = NULL;
        EXTI_INTEN &= (uint32_t)~EXTI_16;
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_pmu_irq_handle_all_reset(void)
{
    low_voltage_handle = NULL;
}
