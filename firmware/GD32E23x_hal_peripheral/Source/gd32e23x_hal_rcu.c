/*!
    \file    gd32e23x_hal_rcu.c
    \brief   RCU driver
    
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

#define _CKOUT_GPIO_CLK_ENABLE()       hal_rcu_periph_clk_enable(RCU_GPIOA)
#define _CKOUT_GPIO_PORT               GPIOA
#define _CKOUT_GPIO_PIN                GPIO_PIN_8
#define _RCU_CKSYSSRC_INDEX(x)         (x)

hal_rcu_irq_struct rcu_irq = {NULL, NULL, NULL, NULL, NULL, NULL, NULL};

/* RCU system clock oscillator stable flag */
static const rcu_flag_enum _rcu_stab_flag[] = {
    RCU_FLAG_IRC8MSTB,
    RCU_FLAG_HXTALSTB,
    RCU_FLAG_PLLSTB
};

/* RCU system clock source timeout */
static const uint32_t _rcu_timeout[] = {
    RCU_IRC8M_TIMEOUT,
    RCU_HXTAL_TIMEOUT,
    RCU_PLL_TIMEOUT
};

/* RCU system clock source */
static const uint32_t _rcu_scss[] = {
    RCU_SCSS_IRC8M,
    RCU_SCSS_HXTAL,
    RCU_SCSS_PLL
};

/*!
    \brief      deinitialize the RCU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rcu_deinit(void)
{
    rcu_deinit();

    SystemCoreClock = IRC8M_VALUE;
}

/*!
    \brief      initialize the RCU structure
    \param[in]  rcu_struct_type: the RCU structure type, can be one of following values:
      \arg        HAL_RCU_CLK_STRUCT: RCU clock structure type
      \arg        HAL_RCU_OSCI_STRUCT: RCU oscillator structure type
      \arg        HAL_RCU_PERIPHCLK_STRUCT: RCU peripheral clock structure type
    \param[in]  p_struct: pointer of the RCU structure
    \param[out] none
    \retval     none
*/
void hal_rcu_struct_init(hal_rcu_struct_type_enum rcu_struct_type, void *p_struct)
{
    switch(rcu_struct_type){
    case HAL_RCU_CLK_STRUCT:
        /* initialize rcu clock structure with the default values */
        ((hal_rcu_clk_struct *)p_struct)->clock_type = RCU_CLKTYPE_NONE;
        ((hal_rcu_clk_struct *)p_struct)->ahbclk_divider = RCU_SYSCLK_AHBDIV1;
        ((hal_rcu_clk_struct *)p_struct)->apb1clk_divider = RCU_AHBCLK_APB1DIV1;
        ((hal_rcu_clk_struct *)p_struct)->apb2clk_divider = RCU_AHBCLK_APB2DIV1;
        ((hal_rcu_clk_struct *)p_struct)->sysclk_source = RCU_SYSCLK_SRC_IRC8M;
        break;

    case HAL_RCU_OSCI_STRUCT:
        /* initialize rcu oscillator structure with the default values */
        ((hal_rcu_osci_struct *)p_struct)->hxtal.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->hxtal.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->lxtal.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->lxtal.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->irc28m.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->irc28m.adjust_value = 0U;
        ((hal_rcu_osci_struct *)p_struct)->irc28m.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->irc8m.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->irc8m.adjust_value = 0U;
        ((hal_rcu_osci_struct *)p_struct)->irc8m.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->irc40k.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->irc40k.state = RCU_OSC_NONE;
        ((hal_rcu_osci_struct *)p_struct)->pll.need_configure = DISABLE;
        ((hal_rcu_osci_struct *)p_struct)->pll.state = RCU_OSC_NONE;
        break;

    case HAL_RCU_PERIPHCLK_STRUCT:
        /* initialize rcu peripheral clock structure with the default values */
        ((hal_rcu_periphclk_struct *)p_struct)->periph_clock_type = RCU_PERIPH_CLKTYPE_NONE;
        ((hal_rcu_periphclk_struct *)p_struct)->adc_clock_source = RCU_ADCCK_IRC28M_DIV2;
        ((hal_rcu_periphclk_struct *)p_struct)->rtc_clock_source = RCU_RTC_CLKSRC_NONE;
        ((hal_rcu_periphclk_struct *)p_struct)->usart0_clock_source = RCU_USART0_CLKSRC_IRC8M;
        break;

    default:
        break;
    }
}

/*!
    \brief      enable the peripherals clock
    \param[in]  periph: RCU peripherals
                  the argument could be selected from enumeration <rcu_periph_enum>
    \param[out] none
    \retval     none
*/
void hal_rcu_periph_clk_enable(rcu_periph_enum periph)
{
    rcu_periph_clock_enable(periph);
}

/*!
    \brief      disable the peripherals clock
    \param[in]  periph: RCU peripherals
                  the argument could be selected from enumeration <rcu_periph_enum>
    \param[out] none
    \retval     none
*/
void hal_rcu_periph_clk_disable(rcu_periph_enum periph)
{
    rcu_periph_clock_disable(periph);
}

/*!
    \brief      initialize the RCU extended peripherals(RTC, Usart0 and ADC) clocks
    \param[in]  periph_clk: the pointer of peripheral clock structure
                  periph_clock_type(member): can be a combination of the following values
                    RCU_PERIPH_CLKTYPE_NONE: no clock type
                    RCU_PERIPH_CLKTYPE_RTC: RTC clock type
                    RCU_PERIPH_CLKTYPE_USART0: usart0 clock type
                    RCU_PERIPH_CLKTYPE_ADC: ADC clock type
                  rtc_clock_source(member): only one parameter can be selected which is shown as below
                    RCU_RTC_CLKSRC_NONE: RTC has no clock source
                    RCU_RTC_CLKSRC_LXTAL: RTC select LXTAL as clock source
                    RCU_RTC_CLKSRC_IRC40K: RTC select IRC40K as clock source
                    RCU_RTC_CLKSRC_HXTAL_DIV32: RTC select HXTAL/32 as clock source
                  usart0_clock_source(member): only one parameter can be selected which is shown as below
                    RCU_USART_CLKSRC_APB2: USART0 select APB2 as clock source
                    RCU_USART_CLKSRC_SYS: USART0 select SYSCLK as clock source
                    RCU_USART_CLKSRC_LXTAL: USART0 select LXTAL as clock source
                    RCU_USART_CLKSRC_IRC8M: USART0 select IRC8M as clock source
                  adc_clock_source: ADC clock source
                    the argument could be selected from enumeration <rcu_adc_clock_enum>
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_rcu_periph_clock_config(hal_rcu_periphclk_struct *periph_clk)
{
    uint32_t backup_reg = 0U;
    FlagStatus pwr_state = RESET;

#if (1 == HAL_PARAMETER_CHECK)

    if (NULL == periph_clk) {
        HAL_DEBUGE("parameter [periph_clk] is a NULL pointer.");

        return HAL_ERR_ADDRESS;
    }

    if(periph_clk->periph_clock_type > RCU_PERIPH_CLKTYPE_MAX){
        HAL_DEBUGE("parameter member [periph_clk->periph_clock_type] is invalid.");

        return HAL_ERR_VAL;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure RTC clock */
    if(RCU_PERIPH_CLKTYPE_RTC == (periph_clk->periph_clock_type & RCU_PERIPH_CLKTYPE_RTC)){
        /* need to enable pmu clock */
        if(RESET == (RCU_APB1EN & RCU_APB1EN_PMUEN)){
            rcu_periph_clock_enable (RCU_PMU);

            pwr_state = SET;
        }

        /* RTC clock need to activate backup domain */
        if(RESET == (PMU_CTL & PMU_CTL_BKPWEN)){
            uint32_t tick_start = 0U;

            pmu_backup_write_enable();

            tick_start = hal_basetick_count_get();

            while(RESET == (PMU_CTL & PMU_CTL_BKPWEN)){
                if(SET == hal_basetick_timeout_check(tick_start, RCU_BP_TIMEOUT)){
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* check if the RTC clock source is modified */
        backup_reg = RCU_BDCTL & RCU_BDCTL_RTCSRC;

        if((0x00000000U != backup_reg) && \
             (backup_reg != (periph_clk->rtc_clock_source & RCU_BDCTL_RTCSRC))){
            /* store the BDCTL register value before resetting the backup domain */
            backup_reg = RCU_BDCTL & ~RCU_BDCTL_RTCSRC;

            /* reset the backup domain */
            rcu_bkp_reset_enable();
            rcu_bkp_reset_disable();

            RCU_BDCTL = backup_reg;

            if(RESET != (backup_reg & RCU_BDCTL_LXTALEN)){
                if(ERROR == rcu_osci_stab_wait(RCU_LXTAL)){
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* configure the RTC clock source */
        rcu_rtc_clock_config(periph_clk->rtc_clock_source);

        if(SET == pwr_state){
            rcu_periph_clock_disable (RCU_PMU);

            pwr_state = RESET;
        }
    }

    /* configure the USART0 clock source */
    if(RCU_PERIPH_CLKTYPE_USART0 == (periph_clk->periph_clock_type & RCU_PERIPH_CLKTYPE_USART0)){
        rcu_usart_clock_config(periph_clk->usart0_clock_source);
    }

    /* configure the ADC clock source */
    if(RCU_PERIPH_CLKTYPE_ADC == (periph_clk->periph_clock_type & RCU_PERIPH_CLKTYPE_ADC)){
        rcu_adc_clock_config(periph_clk->adc_clock_source);
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      get the peripherals clock frequency
    \param[in]  periph_clk: peripheral clock type, only one parameter can be selected which is shown as below
      \arg          RCU_PERIPH_CLKTYPE_NONE: no clock type
      \arg          RCU_PERIPH_CLKTYPE_RTC: RTC clock type
      \arg          RCU_PERIPH_CLKTYPE_USART0: usart0 clock type
      \arg          RCU_PERIPH_CLKTYPE_ADC: ADC clock type
      \arg          RCU_PERIPH_CLKTYPE_APB1TIMER: APB1 timer clock type
      \arg          RCU_PERIPH_CLKTYPE_APB2TIMER: APB2 timer clock type
    \param[out] none
    \retval     peripheral clock frequency
*/
uint32_t hal_rcu_periph_clkfreq_get(uint32_t periph_clk)
{
    uint32_t periph_freq = 0U;
    uint32_t src_clk = 0U;

#if (1 == HAL_PARAMETER_CHECK)

    if(periph_clk >= RCU_PERIPH_CLKTYPE_MAX){
        HAL_DEBUGE("parameter [periph_clk] is invalid.");

        return 0U;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(periph_clk){
    case RCU_PERIPH_CLKTYPE_RTC:
        /* get the current RTC clock source */
        src_clk = RCU_BDCTL & RCU_BDCTL_RTCSRC;

        if((RCU_RTCSRC_LXTAL == src_clk) &&(SET == rcu_flag_get(RCU_FLAG_LXTALSTB))){
            periph_freq = LXTAL_VALUE;
        }else if((RCU_RTCSRC_IRC40K == src_clk) &&(SET == rcu_flag_get(RCU_FLAG_IRC40KSTB))){
            periph_freq = IRC40K_VALUE;
        }else if((RCU_RTCSRC_HXTAL_DIV32 == src_clk) &&(SET == rcu_flag_get(RCU_FLAG_HXTALSTB))){
            periph_freq = HXTAL_VALUE / 32U;
        }else{
            periph_freq = 0U;
        }
        break;
    case RCU_PERIPH_CLKTYPE_USART0:
        /* get the current USART0 clock source */
        periph_freq = rcu_clock_freq_get(CK_USART);
        break;
    case RCU_PERIPH_CLKTYPE_ADC:
        /* get the current ADC clock source */
        periph_freq = rcu_clock_freq_get(CK_ADC);
        break;
    case RCU_PERIPH_CLKTYPE_APB1TIMER:
        /* get the current APB1 TIMER clock source */
        if(RCU_APB1_CKAHB_DIV1 == (RCU_CFG0 & RCU_CFG0_APB1PSC)){
            periph_freq = rcu_clock_freq_get(CK_APB1);
        }else{
            periph_freq = rcu_clock_freq_get(CK_APB1) * 2;
        }
        break;
    case RCU_PERIPH_CLKTYPE_APB2TIMER:
        /* get the current APB2 TIMER clock source */
        if(RCU_APB2_CKAHB_DIV1 == (RCU_CFG0 & RCU_CFG0_APB2PSC)){
            periph_freq = rcu_clock_freq_get(CK_APB2);
        }else{
            periph_freq = rcu_clock_freq_get(CK_APB2) * 2;
        }
        break;
    default:
        break;
    }

    return periph_freq;
}

/*!
    \brief      configure the RCU oscillators 
    \param[in]  rcu_osci: the pointer of the RCU oscillators structure
                  hxtal: HXTAL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  lxtal: LXTAL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc28m: IRC28M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    calibration_value: calibration value
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc8m: IRC8M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    calibration_value: calibration value
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc40k: IRC40K status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  pll: PLL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                    pll_source: RCU_PLL_SRC_HXTAL, RCU_PLL_SRC_IRC8M_DIV2
                    pre_div: the argument could be selected from enumeration <hal_rcu_pll_prediv_enum>
                    pll_mul: the argument could be selected from enumeration <hal_rcu_pll_mul_enum>
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_rcu_osci_config(hal_rcu_osci_struct *rcu_osci)
{
    FlagStatus pwr_state = RESET;

#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == rcu_osci){
        HAL_DEBUGE("parameter [rcu_osci] is a NULL pointer.");

        return HAL_ERR_ADDRESS;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* clear all rcu configuration */
    rcu_deinit();

    /* configure HXTAL */
    if(ENABLE == rcu_osci->hxtal.need_configure){
        /* it is not allowed to be disabled when the HXTAL is used as system clock or clock source of PLL */
        if((RCU_SCSS_HXTAL == rcu_system_clock_source_get()) || \
             ((RCU_SCSS_PLL == rcu_system_clock_source_get()) && \
              (RCU_PLLSRC_HXTAL == (RCU_CFG0 & RCU_CFG0_PLLSEL)))){
            if((RESET != (RCU_CTL0 & RCU_CTL0_HXTALSTB)) && (RCU_OSC_OFF == rcu_osci->hxtal.state)){
                return HAL_ERR_VAL;
            }
        } else {
            /* configure the new HXTAL state */
            switch(rcu_osci->hxtal.state){
            case RCU_OSC_OFF:
                rcu_osci_off(RCU_HXTAL);
                rcu_osci_bypass_mode_disable(RCU_HXTAL);
                break;
            case RCU_OSC_ON:
                rcu_osci_on(RCU_HXTAL);
                break;
            case RCU_OSC_BYPASS:
                rcu_osci_off(RCU_HXTAL);
                rcu_osci_bypass_mode_enable(RCU_HXTAL);
                rcu_osci_on(RCU_HXTAL);
                break;
            default:
                break;
            }

            if(RCU_OSC_OFF != rcu_osci->hxtal.state){
                if(ERROR == rcu_osci_stab_wait(RCU_HXTAL)){
                    return HAL_ERR_TIMEOUT;
                }
            } else {
                uint32_t tick_start = hal_basetick_count_get();

                /* wait till HXTAL is disable */
                while(RESET != (RCU_CTL0 & RCU_CTL0_HXTALSTB)){
                    if(SET == hal_basetick_timeout_check(tick_start, RCU_HXTAL_TIMEOUT)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
        }
    }

    /* configure LXTAL */
    if(ENABLE == rcu_osci->lxtal.need_configure){
        if(RESET == (RCU_APB1EN & RCU_APB1EN_PMUEN)){
            rcu_periph_clock_enable(RCU_PMU);

            pwr_state = SET;
        }

        /* update LXTAL configuration in backup domain control register */
        if(RESET == (PMU_CTL & PMU_CTL_BKPWEN)){
            uint32_t tick_start = 0U;

            /* enable write access to backup domain */
            pmu_backup_write_enable();

            tick_start = hal_basetick_count_get();

            while(RESET == (PMU_CTL & PMU_CTL_BKPWEN)){
                if(SET == hal_basetick_timeout_check(tick_start, RCU_BP_TIMEOUT)){
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* configure the new LXTAL state */
        switch(rcu_osci->lxtal.state){
        case RCU_OSC_OFF:
            rcu_osci_off(RCU_LXTAL);
            rcu_osci_bypass_mode_disable(RCU_LXTAL);
            break;
        case RCU_OSC_ON:
            rcu_osci_on(RCU_LXTAL);
            break;
        case RCU_OSC_BYPASS:
            rcu_osci_off(RCU_LXTAL);
            rcu_osci_bypass_mode_enable(RCU_LXTAL);
            rcu_osci_on(RCU_LXTAL);
            break;
        default:
            break;
        }

        if(RCU_OSC_OFF != rcu_osci->lxtal.state){
            if(ERROR == rcu_osci_stab_wait(RCU_LXTAL)){
                return HAL_ERR_TIMEOUT;
            }
        }else{
            /* wait till LXTAL is disabled */
            uint32_t tick_start = hal_basetick_count_get();

            while(RESET != (RCU_BDCTL & RCU_BDCTL_LXTALSTB)){
                if(SET == hal_basetick_timeout_check(tick_start, RCU_LXTAL_TIMEOUT)){
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        if(SET == pwr_state){
            rcu_periph_clock_disable(RCU_PMU);

            pwr_state = RESET;
        }
    }

    /* configure IRC8M */
    if(ENABLE == rcu_osci->irc8m.need_configure){
        /* it is not allowed to be disabled when IRC8M is used as system clock or as PLL source */
        if((RCU_SCSS_IRC8M == rcu_system_clock_source_get()) || \
             ((RCU_SCSS_PLL == rcu_system_clock_source_get()) && \
              (RCU_PLLSRC_IRC8M_DIV2 == (RCU_CFG0 & RCU_CFG0_PLLSEL)))){
            if((RESET != (RCU_CTL0 & RCU_CTL0_IRC8MSTB)) && (RCU_OSC_OFF == rcu_osci->irc8m.state)){
                return HAL_ERR_VAL;
            }else{
                if (rcu_osci->irc8m.adjust_value){
                    /* adjusts the IRC8M calibration value */
                    rcu_irc8m_adjust_value_set(rcu_osci->irc8m.adjust_value);
                }
            }
        }else{
            if(RCU_OSC_OFF != rcu_osci->irc8m.state){
                rcu_osci_on(RCU_IRC8M);

                /* wait till IRC8M is stable */
                if(ERROR == rcu_osci_stab_wait(RCU_IRC8M)){
                    return HAL_ERR_TIMEOUT;
                }

                if (rcu_osci->irc8m.adjust_value){
                    /* adjusts the IRC8M calibration value */
                    rcu_irc8m_adjust_value_set(rcu_osci->irc8m.adjust_value);
                }
            }else{
                uint32_t tick_start = 0U;

                rcu_osci_off(RCU_IRC8M);

                tick_start = hal_basetick_count_get();

                /* wait till IRC8M is disabled */
                while(RESET != (RCU_CTL0 & RCU_CTL0_IRC8MSTB)){
                    if(SET == hal_basetick_timeout_check(tick_start, RCU_IRC8M_TIMEOUT)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
        }
    }

    /* configure IRC28M */
    if(ENABLE == rcu_osci->irc28m.need_configure){
        /* set the IRC28M new state */
        if(RCU_OSC_ON == rcu_osci->irc28m.state){
            RCU_CFG2 &= ~RCU_CFG2_ADCSEL;

            rcu_osci_on(RCU_IRC28M);

            /* wait till IRC28M is stable */
            if(ERROR == rcu_osci_stab_wait(RCU_IRC28M)){
                return HAL_ERR_TIMEOUT;
            }

            if(rcu_osci->irc28m.adjust_value){
                /* adjusts the IRC28M calibration value */
                rcu_irc28m_adjust_value_set(rcu_osci->irc28m.adjust_value);
            }
        }else if(RCU_OSC_ADCCTL == rcu_osci->irc28m.state){
            RCU_CFG2 &= ~RCU_CFG2_ADCSEL;
            
            rcu_osci_on(RCU_IRC28M);
            
            /* wait till IRC28M is stable */
            if(ERROR == rcu_osci_stab_wait(RCU_IRC28M)){
                return HAL_ERR_TIMEOUT;
            }

            if(rcu_osci->irc28m.adjust_value){
                /* adjusts the IRC28M calibration value */
                rcu_irc28m_adjust_value_set(rcu_osci->irc28m.adjust_value);
            }
        }else{
            uint32_t tick_start = 0U;

            RCU_CFG2 |= RCU_CFG2_ADCSEL;

            rcu_osci_off(RCU_IRC28M);

            tick_start = hal_basetick_count_get();

            /* wait till IRC28M is disabled */
            while(RESET != (RCU_CTL1 & RCU_CTL1_IRC28MSTB)){
                if(SET == hal_basetick_timeout_check(tick_start, RCU_IRC28M_TIMEOUT)){
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }

    /* configure IRC40K */
    if(ENABLE == rcu_osci->irc40k.need_configure){
        /* set the IRC40K new state */
        if(RCU_OSC_OFF != rcu_osci->irc40k.state){
            rcu_osci_on(RCU_IRC40K);

            /* wait till IRC40K is stable */
            if(ERROR == rcu_osci_stab_wait(RCU_IRC40K)){
                return HAL_ERR_TIMEOUT;
            }
        }else{
            uint32_t tick_start = 0U;

            rcu_osci_off(RCU_IRC40K);

            tick_start = hal_basetick_count_get();

            /* wait till IRC40K is disabled */
            while(RESET != (RCU_RSTSCK & RCU_RSTSCK_IRC40KSTB)){
                if(SET == hal_basetick_timeout_check(tick_start, RCU_IRC40K_TIMEOUT)){
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }

    /* configure PLL */
    if(ENABLE == rcu_osci->pll.need_configure){
        /* check if the PLL is used as system clock or not */
        if(RCU_OSC_NONE != rcu_osci->pll.state){
            if(RCU_SCSS_PLL != rcu_system_clock_source_get()){
                uint32_t tick_start = 0U;

                if(RCU_OSC_ON == rcu_osci->pll.state){
                    rcu_osci_off(RCU_PLL_CK);

                    tick_start = hal_basetick_count_get();

                    /* wait till PLL is disabled */
                    while(RESET != (RCU_CTL0 & RCU_CTL0_PLLSTB)){
                        if(SET == hal_basetick_timeout_check(tick_start, RCU_PLL_TIMEOUT)){
                            return HAL_ERR_TIMEOUT;
                        }
                    }

                    /* configure the PLL */
                    rcu_hxtal_prediv_config(rcu_osci->pll.pre_div);
                    rcu_pll_config(rcu_osci->pll.pll_source, rcu_osci->pll.pll_mul);

                    rcu_osci_on(RCU_PLL_CK);

                    /* wait till PLL is stable */
                    if(ERROR == rcu_osci_stab_wait(RCU_PLL_CK)){
                        return HAL_ERR_TIMEOUT;
                    }
                }else{
                    rcu_osci_off(RCU_PLL_CK);

                    tick_start = hal_basetick_count_get();

                    /* wait till PLL is disabled */
                    while(RESET != (RCU_CTL0 & RCU_CTL0_PLLSTB)){
                        if(SET == hal_basetick_timeout_check(tick_start, RCU_PLL_TIMEOUT)){
                            return HAL_ERR_TIMEOUT;
                        }
                    }
                }
            }else{
                return HAL_ERR_VAL;
            }
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      get the RCU oscillators configuration
    \param[in]  rcu_osci: the pointer of the RCU oscillators structure
                  hxtal: HXTAL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  lxtal: LXTAL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc28m: IRC28M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    calibration_value: calibration value
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc8m: IRC8M status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    calibration_value: calibration value
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  irc40k: IRC40K status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                  pll: PLL status structure
                    need_cofigure: configure flag, can be ENABLE or DISABLE
                    status: the argument could be selected from enumeration <hal_rcu_osc_state_enum>
                    pll_source: RCU_PLL_SRC_HXTAL, RCU_PLL_SRC_IRC8M_DIV2
                    pre_div: the argument could be selected from enumeration <hal_rcu_pll_prediv_enum>
                    pll_mul: the argument could be selected from enumeration <hal_rcu_pll_mul_enum>
    \param[out] none
    \retval     none
*/
void hal_rcu_osci_config_get(hal_rcu_osci_struct *rcu_osci)
{
#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == rcu_osci){
        HAL_DEBUGE("parameter [rcu_osci] is a NULL pointer.");

        return;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure all the oscillator type */
    rcu_osci->hxtal.need_configure = ENABLE;
    rcu_osci->lxtal.need_configure = ENABLE;
    rcu_osci->irc8m.need_configure = ENABLE;
    rcu_osci->irc28m.need_configure = ENABLE;
    rcu_osci->irc40k.need_configure = ENABLE;
    rcu_osci->pll.need_configure = ENABLE;

    /* get the current HXTAL state */
    if(RESET != (RCU_CTL0 & RCU_CTL0_HXTALBPS)){
        rcu_osci->hxtal.state = RCU_OSC_BYPASS;
    }else if(RESET != (RCU_CTL0 & RCU_CTL0_HXTALEN)){
        rcu_osci->hxtal.state = RCU_OSC_ON;
    }else{
        rcu_osci->hxtal.state = RCU_OSC_OFF;
    }

    /* get the current LXTAL state */
    if(RESET != (RCU_BDCTL & RCU_BDCTL_LXTALBPS)){
        rcu_osci->lxtal.state = RCU_OSC_BYPASS;
    }else if(RESET != (RCU_BDCTL & RCU_BDCTL_LXTALEN)){
        rcu_osci->lxtal.state = RCU_OSC_ON;
    }else{
        rcu_osci->lxtal.state = RCU_OSC_OFF;
    }

    /* get the current IRC8M state and adjust value */
    if(RESET != (RCU_CTL0 & RCU_CTL0_IRC8MEN)){
        rcu_osci->irc8m.state = RCU_OSC_ON;
    }else{
        rcu_osci->irc8m.state = RCU_OSC_OFF;
    }

    rcu_osci->irc8m.adjust_value = (uint8_t)((RCU_CTL0 & RCU_CTL0_IRC8MCALIB) >> 8);

    /* get the current IRC28M state and adjust value */
    if(RESET == (RCU_CFG2 & RCU_CFG2_ADCSEL)){
        rcu_osci->irc28m.state = RCU_OSC_ADCCTL;
    }else{
        if(RESET != (RCU_CTL1 & RCU_CTL1_IRC28MEN)){
            rcu_osci->irc28m.state = RCU_OSC_ON;
        }else{
            rcu_osci->irc28m.state = RCU_OSC_OFF;
        }
    }

    rcu_osci->irc28m.adjust_value = (uint8_t)((RCU_CTL1 & RCU_CTL1_IRC28MCALIB) >> 8);

    /* get the current IRC40K state */
    if(RESET != (RCU_RSTSCK & RCU_RSTSCK_IRC40KEN)){
        rcu_osci->irc40k.state = RCU_OSC_ON;
    }else{
        rcu_osci->irc40k.state = RCU_OSC_OFF;
    }

    /* get the current PLL state */
    if(RESET != (RCU_CTL0 & RCU_CTL0_PLLEN)){
        rcu_osci->pll.state = RCU_OSC_ON;
    }else{
        rcu_osci->pll.state = RCU_OSC_OFF;
    }

    /* get the PLL parameters */
    rcu_osci->pll.pll_mul = (hal_rcu_pll_mul_enum)(RCU_CFG0 & RCU_CFG0_PLLMF);
    rcu_osci->pll.pll_source = (hal_rcu_pll_src_enum)(RCU_CFG0 & RCU_CFG0_PLLSEL);
    rcu_osci->pll.pre_div = (hal_rcu_pll_prediv_enum)(RCU_CFG0 & RCU_CFG0_PLLPREDV);
}

/*!
    \brief      configure the RCU clock
    \param[in]  rcu_clk: the pointer of the RCU clock structure
                  clock_type(member): can be a combination of the following values
                    RCU_CLKTYPE_NONE: no clock type
                    RCU_CLKTYPE_SYSCLK: system clock type
                    RCU_CLKTYPE_AHBCLK: AHB bus clock type
                    RCU_CLKTYPE_APB1CLK: APB1 bus clock type
                    RCU_CLKTYPE_APB2CLK: APB2 bus clock type
                  sysclk_source(member): can be one of the following values
                    RCU_SYSCLK_SRC_IRC8M: IRC8M as system clock source
                    RCU_SYSCLK_SRC_HXTAL: HXTAL as system clock source
                    RCU_SYSCLK_SRC_PLL: PLL as system clock source
                  ahbclk_divider(member): AHB clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_ahbdiv_enum>
                  apb1clk_divider(member): APB1 clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_apb1div_enum>
                  apb2clk_divider(member): APB2 clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_apb2div_enum>
    \param[in]  fmc_wscnt: FMC wait state count
                  it depends on the AHB clock frequency, can be one of following values:
      \arg          WS_WSCNT_0: if AHB clock frequency <= 24MHz
      \arg          WS_WSCNT_1: if AHB clock frequency <= 48MHz
      \arg          WS_WSCNT_2: if AHB clock frequency <= 72MHz
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
    \note       user need compute the AHB clock freuuency before, then decide the fmc_wscnt value
*/
int32_t hal_rcu_clock_config(hal_rcu_clk_struct *rcu_clk, uint8_t fmc_wscnt)
{
#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == rcu_clk){
        HAL_DEBUGE("parameter [rcu_clk] is a NULL pointer.");

        return HAL_ERR_ADDRESS;
    }

    if(fmc_wscnt > 2) {
        HAL_DEBUGE("parameter [fmc_wscnt] is invalid.");

        return HAL_ERR_VAL;
    }

    if(rcu_clk->clock_type > RCU_CLKTYPE_MAX){
        HAL_DEBUGE("parameter member [rcu_clk->clock_type] is invalid.");

        return HAL_ERR_VAL;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* to correctly use flash memory, the count of wait states (WSCNT) must be 
       correctly programmed according to the frequency of the CPU clock (AHBCLK) of the device. */

    if(fmc_wscnt > (FMC_WS & FMC_WS_WSCNT)){
        /* If AHB increase, configure flash wait state count before */
        fmc_wscnt_set(fmc_wscnt);

        if((FMC_WS & FMC_WS_WSCNT) != fmc_wscnt){
            return HAL_ERR_VAL;
        }
    }

    /* configure system clock */
    if(RCU_CLKTYPE_SYSCLK == (rcu_clk->clock_type & RCU_CLKTYPE_SYSCLK)){
        uint32_t tick_start = 0U;
        uint32_t time_out = _rcu_timeout[_RCU_CKSYSSRC_INDEX(rcu_clk->sysclk_source)];

        if(RESET == rcu_flag_get(_rcu_stab_flag[_RCU_CKSYSSRC_INDEX(rcu_clk->sysclk_source)])){
            return HAL_ERR_VAL;
        }

        /* configure the system clock source */
        rcu_system_clock_source_config(CFG0_SCS(rcu_clk->sysclk_source));

        tick_start = hal_basetick_count_get();

        /* wait till system clock source is stable */
        while(_rcu_scss[_RCU_CKSYSSRC_INDEX(rcu_clk->sysclk_source)] != rcu_system_clock_source_get()){
            if(SET == hal_basetick_timeout_check(tick_start, time_out)){
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* configure AHB bus clock */
    if(RCU_CLKTYPE_AHBCLK == (rcu_clk->clock_type & RCU_CLKTYPE_AHBCLK)){
        /* configure AHB clock */
        rcu_ahb_clock_config(rcu_clk->ahbclk_divider);
    }

    if(fmc_wscnt < (FMC_WS & FMC_WS_WSCNT)){
        /* If AHB decrease, configure flash wait state count after */
        fmc_wscnt_set(fmc_wscnt);

        if((FMC_WS & FMC_WS_WSCNT) != fmc_wscnt){
            return HAL_ERR_VAL;
        }
    }

    /* configure APB1 bus clock */
    if(RCU_CLKTYPE_APB1CLK == (rcu_clk->clock_type & RCU_CLKTYPE_APB1CLK)){
        rcu_apb1_clock_config(rcu_clk->apb1clk_divider);
    }

    /* configure APB2 bus clock */
    if(RCU_CLKTYPE_APB2CLK == (rcu_clk->clock_type & RCU_CLKTYPE_APB2CLK)){
        rcu_apb2_clock_config(rcu_clk->apb2clk_divider);
    }

    /* update the SystemCoreClock global variable */
    SystemCoreClock = rcu_clock_freq_get(CK_SYS);

    /* configure the source of time base considering new system clocks settings */
    hal_basetick_init(g_basetick_source);

    return HAL_ERR_NONE;
}

/*!
    \brief      get the RCU clock configuration
    \param[in]  rcu_clk: the pointer of the RCU clock structure
                  clock_type(member): can be a combination of the following values
                    RCU_CLKTYPE_NONE: no clock type
                    RCU_CLKTYPE_SYSCLK: system clock type
                    RCU_CLKTYPE_AHBCLK: AHB bus clock type
                    RCU_CLKTYPE_APB1CLK: APB1 bus clock type
                    RCU_CLKTYPE_APB2CLK: APB2 bus clock type
                  sysclk_source(member): can be one of the following values
                    RCU_SYSCLK_SRC_IRC8M: IRC8M as system clock source
                    RCU_SYSCLK_SRC_HXTAL: HXTAL as system clock source
                    RCU_SYSCLK_SRC_PLL: PLL as system clock source
                  ahbclk_divider(member): AHB clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_ahbdiv_enum>
                  apb1clk_divider(member): APB1 clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_apb1div_enum>
                  apb2clk_divider(member): APB2 clock divider
                    the argument could be selected from enumeration <hal_rcu_sysclk_apb2div_enum>
    \param[in]  fmc_wscnt: pointer of FMC wait state count
    \param[out] none
    \retval     none
*/
void hal_rcu_clock_config_get(hal_rcu_clk_struct *rcu_clk, uint8_t *fmc_wscnt)
{
#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == rcu_clk){
        HAL_DEBUGE("parameter [rcu_clk] is a NULL pointer.");

        return;
    }

    if(NULL == fmc_wscnt){
        HAL_DEBUGE("parameter [fmc_wscnt] is a NULL pointer.");

        return;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* get all clock type configuration */
    rcu_clk->clock_type = RCU_CLKTYPE_SYSCLK | RCU_CLKTYPE_AHBCLK | RCU_CLKTYPE_APB1CLK | RCU_CLKTYPE_APB2CLK;
    rcu_clk->sysclk_source = (hal_rcu_sysclk_src_enum)(RCU_CFG0 & RCU_CFG0_SCS);
    rcu_clk->ahbclk_divider = (hal_rcu_sysclk_ahbdiv_enum)(RCU_CFG0 & RCU_CFG0_AHBPSC);
    rcu_clk->apb1clk_divider = (hal_rcu_ahbclk_apbldiv_enum)(RCU_CFG0 & RCU_CFG0_APB1PSC);
    rcu_clk->apb2clk_divider = (hal_rcu_ahbclk_apb2div_enum)(RCU_CFG0 & RCU_CFG0_APB2PSC);

    /* get flash wait status configuration */
    *fmc_wscnt = FMC_WS & FMC_WS_WSCNT;
}

/*!
    \brief      configure the clock out to output on CKOUT pin
    \param[in]  ckout_src: clock out source
                  the argument could be selected from enumeraion <hal_rcu_ckout_src_enum>
    \param[in]  ckout_div: clock out divider
                  the argument could be selected from enumeraion <hal_rcu_ckout_div_enum>
    \param[out] none
    \retval     
*/
void hal_rcu_clock_out_config(hal_rcu_ckout_src_enum ckout_src, hal_rcu_ckout_div_enum ckout_div)
{
    uint32_t clkout_src = 0U;

    hal_gpio_init_struct gpio_init_struct;

    _CKOUT_GPIO_CLK_ENABLE();

    /* configure clock out GPIO pin */
    gpio_init_struct.mode = HAL_GPIO_MODE_AF_PP;
    gpio_init_struct.ospeed = HAL_GPIO_OSPEED_50MHZ;
    gpio_init_struct.pull = HAL_GPIO_PULL_NONE;
    gpio_init_struct.af = HAL_GPIO_AF_0;

    hal_gpio_init(_CKOUT_GPIO_PORT, _CKOUT_GPIO_PIN, &gpio_init_struct);

    if(RCU_CKOUT_SRC_CKPLL_DIV1 == ckout_src){
        clkout_src = RCU_CKOUTSRC_CKPLL_DIV1;
    }else{
        clkout_src = CFG0_CKOUTSEL(ckout_src);
    }

    /* clock out configuration */
    rcu_ckout_config(clkout_src, ckout_div);
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  prcu_irq: pointer of RCU interrupt handler structure
    \param[out] none
    \retval     none
*/
void hal_rcu_irq_handle_set(hal_rcu_irq_struct *prcu_irq)
{
#if (1 == HAL_PARAMETER_CHECK)

    if(NULL == prcu_irq){
        HAL_DEBUGE("parameter [prcu_irq] is a NULL pointer.");

        return;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */

    /* HXTAL stable interrupt handler set */
    if(NULL != prcu_irq->hxtal_stable_handle){
        rcu_irq.hxtal_stable_handle = prcu_irq->hxtal_stable_handle;

        rcu_interrupt_enable(RCU_INT_HXTALSTB);
    }else{
        rcu_irq.hxtal_stable_handle = NULL;

        rcu_interrupt_disable(RCU_INT_HXTALSTB);
    }

    /* IRC28M stable interrupt handler set */
    if(NULL != prcu_irq->irc28m_stable_handle){
        rcu_irq.irc28m_stable_handle = prcu_irq->irc28m_stable_handle;

        rcu_interrupt_enable(RCU_INT_IRC28MSTB);
    }else{
        rcu_irq.irc28m_stable_handle = NULL;

        rcu_interrupt_disable(RCU_INT_IRC28MSTB);
    }

    /* IRC40K stable interrupt handler set */
    if(NULL != prcu_irq->irc40k_stable_handle){
        rcu_irq.irc40k_stable_handle = prcu_irq->irc40k_stable_handle;

        rcu_interrupt_enable(RCU_INT_IRC40KSTB);
    }else{
        rcu_irq.irc40k_stable_handle = NULL;

        rcu_interrupt_disable(RCU_INT_IRC40KSTB);
    }

    /* IRC8M stable interrupt handler set */
    if(NULL != prcu_irq->irc8m_stable_handle){
        rcu_irq.irc8m_stable_handle = prcu_irq->irc8m_stable_handle;

        rcu_interrupt_enable(RCU_INT_IRC8MSTB);
    }else{
        rcu_irq.irc8m_stable_handle = NULL;

        rcu_interrupt_disable(RCU_INT_IRC8MSTB);
    }

    /* LXTAL stable interrupt handler set */
    if(NULL != prcu_irq->lxtal_stable_handle){
        rcu_irq.lxtal_stable_handle = prcu_irq->lxtal_stable_handle;

        rcu_interrupt_enable(RCU_INT_LXTALSTB);
    }else{
        rcu_irq.lxtal_stable_handle = NULL;

        rcu_interrupt_disable(RCU_INT_LXTALSTB);
    }

    /* PLL stable interrupt handler set */
    if(NULL != prcu_irq->pll_stable_handle){
        rcu_irq.pll_stable_handle = prcu_irq->pll_stable_handle;

        rcu_interrupt_enable(RCU_INT_PLLSTB);
    }else{
        rcu_irq.pll_stable_handle = NULL;

        rcu_interrupt_disable(RCU_INT_PLLSTB);
    }

    /* HXTAL stuck interrupt handler set */
    if(NULL != prcu_irq->hxtal_stuck_handle){
        rcu_irq.hxtal_stuck_handle = prcu_irq->hxtal_stuck_handle;
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rcu_irq_handle_all_reset(void)
{
    /* reset all interrupt handler */
    rcu_irq.hxtal_stable_handle = NULL;
    rcu_irq.hxtal_stuck_handle = NULL;
    rcu_irq.irc28m_stable_handle = NULL;
    rcu_irq.irc40k_stable_handle = NULL;
    rcu_irq.irc8m_stable_handle = NULL;
    rcu_irq.lxtal_stable_handle = NULL;
    rcu_irq.pll_stable_handle = NULL;
}

/*!
    \brief      RCU interrupt handler content function,which is merely used in RCU_handler
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rcu_irq(void)
{
    /* IRC40K stable interrupt handler */
    if(SET == rcu_interrupt_flag_get(RCU_INT_FLAG_IRC40KSTB)){
        rcu_interrupt_flag_clear(RCU_INT_FLAG_IRC40KSTB_CLR);
        if(NULL != rcu_irq.irc40k_stable_handle){
            rcu_irq.irc40k_stable_handle(NULL);
        }
    }

    /* IRC8M stable interrupt handler */
    if(SET == rcu_interrupt_flag_get(RCU_INT_FLAG_IRC8MSTB)){
        rcu_interrupt_flag_clear(RCU_INT_FLAG_IRC8MSTB_CLR);
        if(NULL != rcu_irq.irc8m_stable_handle){
            rcu_irq.irc8m_stable_handle(NULL);
        }
    }

    /* IRC28M stable interrupt handler */
    if(SET == rcu_interrupt_flag_get(RCU_INT_FLAG_IRC28MSTB)){
        rcu_interrupt_flag_clear(RCU_INT_FLAG_IRC28MSTB_CLR);
        if(NULL != rcu_irq.irc28m_stable_handle){
            rcu_irq.irc28m_stable_handle(NULL);
        }
    }

    /* LXTAL stable interrupt handler */
    if(SET == rcu_interrupt_flag_get(RCU_INT_FLAG_LXTALSTB)){
        rcu_interrupt_flag_clear(RCU_INT_FLAG_LXTALSTB_CLR);
        if(NULL != rcu_irq.lxtal_stable_handle){
            rcu_irq.lxtal_stable_handle(NULL);
        }
    }

    /* HXTAL stable interrupt handler */
    if(SET == rcu_interrupt_flag_get(RCU_INT_FLAG_HXTALSTB)){
        rcu_interrupt_flag_clear(RCU_INT_FLAG_HXTALSTB_CLR);
        if(NULL != rcu_irq.hxtal_stable_handle){
            rcu_irq.hxtal_stable_handle(NULL);
        }
    }

    /* PLL stable interrupt handler */
    if(SET == rcu_interrupt_flag_get(RCU_INT_FLAG_PLLSTB)){
        rcu_interrupt_flag_clear(RCU_INT_FLAG_PLLSTB_CLR);
        if(NULL != rcu_irq.pll_stable_handle){
            rcu_irq.pll_stable_handle(NULL);
        }
    }

    /* HXTAL stuck interrupt handler */
    if(SET == rcu_interrupt_flag_get(RCU_INT_FLAG_CKM)){
        rcu_interrupt_flag_clear(RCU_INT_FLAG_CKM_CLR);
        if(NULL != rcu_irq.hxtal_stuck_handle){
            rcu_irq.hxtal_stuck_handle(NULL);
        }
    }
}
