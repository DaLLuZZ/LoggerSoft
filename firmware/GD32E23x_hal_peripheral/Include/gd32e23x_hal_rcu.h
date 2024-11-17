/*!
    \file    gd32e23x_hal_rcu.h
    \brief   definitions for the RCU
    
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

#ifndef GD32E23X_HAL_RCU_H
#define GD32E23X_HAL_RCU_H

#include "gd32e23x_hal.h"

/* RCU structure type */
typedef enum{
    HAL_RCU_CLK_STRUCT,         /*!< RCU clock structure type */
    HAL_RCU_OSCI_STRUCT,        /*!< RCU oscillator structure type */
    HAL_RCU_PERIPHCLK_STRUCT,   /*!< RCU peripheral clock structure type */
}hal_rcu_struct_type_enum;

/* RCU oscillator state */
typedef enum{
    RCU_OSC_NONE,     /*!< the oscillator is not configured */
    RCU_OSC_OFF,      /*!< the oscillator deactivation*/
    RCU_OSC_ON,       /*!< the oscillator activation */
    RCU_OSC_BYPASS,   /*!< external clock source for the oscillator */
    RCU_OSC_ADCCTL,   /*!< IRC28M may be clock source of ADC */
}hal_rcu_osc_state_enum;

/* usart clock source */
typedef enum{
    RCU_USART0_CLKSRC_APB2  = RCU_USART0SRC_CKAPB2,  /*!< CK_USART0 select CK_APB2 */
    RCU_USART0_CLKSRC_SYS   = RCU_USART0SRC_CKSYS,   /*!< CK_USART0 select CK_SYS */
    RCU_USART0_CLKSRC_LXTAL = RCU_USART0SRC_LXTAL,   /*!< CK_USART0 select LXTAL */
    RCU_USART0_CLKSRC_IRC8M = RCU_USART0SRC_IRC8M    /*!< CK_USART0 select IRC8M */
}hal_rcu_usart_clksrc_enum;

/* RTC clock source */
typedef enum{
    RCU_RTC_CLKSRC_NONE        = RCU_RTCSRC_NONE,        /*!< no clock selected */
    RCU_RTC_CLKSRC_LXTAL       = RCU_RTCSRC_LXTAL,       /*!< LXTAL selected as RTC source clock */
    RCU_RTC_CLKSRC_IRC40K      = RCU_RTCSRC_IRC40K,      /*!< IRC40K selected as RTC source clock */
    RCU_RTC_CLKSRC_HXTAL_DIV32 = RCU_RTCSRC_HXTAL_DIV32  /*!< HXTAL/32 selected as RTC source clock */
}hal_rcu_rtc_clksrc_enum;

typedef enum{
    RCU_PLL_SRC_HXTAL      = RCU_PLLSRC_HXTAL,           /*!< PLL clock source select HXTAL */
    RCU_PLL_SRC_IRC8M_DIV2 = RCU_PLLSRC_IRC8M_DIV2       /*!< PLL clock source select IRC8M/2 */
}hal_rcu_pll_src_enum;

typedef enum{
    RCU_PLL_PREDIV1  = RCU_PLL_PREDV1,     /*!< PLL not divided */
    RCU_PLL_PREDIV2  = RCU_PLL_PREDV2,     /*!< PLL divided by 2 */
    RCU_PLL_PREDIV3  = RCU_PLL_PREDV3,     /*!< PLL divided by 3 */
    RCU_PLL_PREDIV4  = RCU_PLL_PREDV4,     /*!< PLL divided by 4 */
    RCU_PLL_PREDIV5  = RCU_PLL_PREDV5,     /*!< PLL divided by 5 */
    RCU_PLL_PREDIV6  = RCU_PLL_PREDV6,     /*!< PLL divided by 6 */
    RCU_PLL_PREDIV7  = RCU_PLL_PREDV7,     /*!< PLL divided by 7 */
    RCU_PLL_PREDIV8  = RCU_PLL_PREDV8,     /*!< PLL divided by 8 */
    RCU_PLL_PREDIV9  = RCU_PLL_PREDV9,     /*!< PLL divided by 9 */
    RCU_PLL_PREDIV10 = RCU_PLL_PREDV10,    /*!< PLL divided by 10 */
    RCU_PLL_PREDIV11 = RCU_PLL_PREDV11,    /*!< PLL divided by 11 */
    RCU_PLL_PREDIV12 = RCU_PLL_PREDV12,    /*!< PLL divided by 12 */
    RCU_PLL_PREDIV13 = RCU_PLL_PREDV13,    /*!< PLL divided by 13 */
    RCU_PLL_PREDIV14 = RCU_PLL_PREDV14,    /*!< PLL divided by 14 */
    RCU_PLL_PREDIV15 = RCU_PLL_PREDV15,    /*!< PLL divided by 15 */
    RCU_PLL_PREDIV16 = RCU_PLL_PREDV16     /*!< PLL divided by 16 */
}hal_rcu_pll_prediv_enum;

/* PLL multiply factor */
typedef enum{
    RCU_PLL_MULT2  = RCU_PLL_MUL2,           /*!< PLL source clock multiply by 2 */
    RCU_PLL_MULT3  = RCU_PLL_MUL3,           /*!< PLL source clock multiply by 3 */
    RCU_PLL_MULT4  = RCU_PLL_MUL4,           /*!< PLL source clock multiply by 4 */
    RCU_PLL_MULT5  = RCU_PLL_MUL5,           /*!< PLL source clock multiply by 5 */
    RCU_PLL_MULT6  = RCU_PLL_MUL6,           /*!< PLL source clock multiply by 6 */
    RCU_PLL_MULT7  = RCU_PLL_MUL7,           /*!< PLL source clock multiply by 7 */
    RCU_PLL_MULT8  = RCU_PLL_MUL8,           /*!< PLL source clock multiply by 8 */
    RCU_PLL_MULT9  = RCU_PLL_MUL9,           /*!< PLL source clock multiply by 9 */
    RCU_PLL_MULT10 = RCU_PLL_MUL10,          /*!< PLL source clock multiply by 10 */
    RCU_PLL_MULT11 = RCU_PLL_MUL11,          /*!< PLL source clock multiply by 11 */
    RCU_PLL_MULT12 = RCU_PLL_MUL12,          /*!< PLL source clock multiply by 12 */
    RCU_PLL_MULT13 = RCU_PLL_MUL13,          /*!< PLL source clock multiply by 13 */
    RCU_PLL_MULT14 = RCU_PLL_MUL14,          /*!< PLL source clock multiply by 14 */
    RCU_PLL_MULT15 = RCU_PLL_MUL15,          /*!< PLL source clock multiply by 15 */
    RCU_PLL_MULT16 = RCU_PLL_MUL16,          /*!< PLL source clock multiply by 16 */
    RCU_PLL_MULT17 = RCU_PLL_MUL17,          /*!< PLL source clock multiply by 17 */
    RCU_PLL_MULT18 = RCU_PLL_MUL18,          /*!< PLL source clock multiply by 18 */
    RCU_PLL_MULT19 = RCU_PLL_MUL19,          /*!< PLL source clock multiply by 19 */
    RCU_PLL_MULT20 = RCU_PLL_MUL20,          /*!< PLL source clock multiply by 20 */
    RCU_PLL_MULT21 = RCU_PLL_MUL21,          /*!< PLL source clock multiply by 21 */
    RCU_PLL_MULT22 = RCU_PLL_MUL22,          /*!< PLL source clock multiply by 22 */
    RCU_PLL_MULT23 = RCU_PLL_MUL23,          /*!< PLL source clock multiply by 23 */
    RCU_PLL_MULT24 = RCU_PLL_MUL24,          /*!< PLL source clock multiply by 24 */
    RCU_PLL_MULT25 = RCU_PLL_MUL25,          /*!< PLL source clock multiply by 25 */
    RCU_PLL_MULT26 = RCU_PLL_MUL26,          /*!< PLL source clock multiply by 26 */
    RCU_PLL_MULT27 = RCU_PLL_MUL27,          /*!< PLL source clock multiply by 27 */
    RCU_PLL_MULT28 = RCU_PLL_MUL28,          /*!< PLL source clock multiply by 28 */
    RCU_PLL_MULT29 = RCU_PLL_MUL29,          /*!< PLL source clock multiply by 29 */
    RCU_PLL_MULT30 = RCU_PLL_MUL30,          /*!< PLL source clock multiply by 30 */
    RCU_PLL_MULT31 = RCU_PLL_MUL31,          /*!< PLL source clock multiply by 31 */
    RCU_PLL_MULT32 = RCU_PLL_MUL32,          /*!< PLL source clock multiply by 32 */
}hal_rcu_pll_mul_enum;

/* system clock source */
typedef enum{
    RCU_SYSCLK_SRC_IRC8M = RCU_CKSYSSRC_IRC8M,   /*!< system clock source select IRC8M */
    RCU_SYSCLK_SRC_HXTAL = RCU_CKSYSSRC_HXTAL,   /*!< system clock source select HXTAL */
    RCU_SYSCLK_SRC_PLL   = RCU_CKSYSSRC_PLL      /*!< system clock source select PLL */
}hal_rcu_sysclk_src_enum;

/* AHB prescaler selection */
typedef enum{
    RCU_SYSCLK_AHBDIV1   = RCU_AHB_CKSYS_DIV1,    /*!< AHB prescaler select CK_SYS */
    RCU_SYSCLK_AHBDIV2   = RCU_AHB_CKSYS_DIV2,    /*!< AHB prescaler select CK_SYS/2 */
    RCU_SYSCLK_AHBDIV4   = RCU_AHB_CKSYS_DIV4,    /*!< AHB prescaler select CK_SYS/4 */
    RCU_SYSCLK_AHBDIV8   = RCU_AHB_CKSYS_DIV8,    /*!< AHB prescaler select CK_SYS/8 */
    RCU_SYSCLK_AHBDIV16  = RCU_AHB_CKSYS_DIV16,   /*!< AHB prescaler select CK_SYS/16 */
    RCU_SYSCLK_AHBDIV64  = RCU_AHB_CKSYS_DIV64,   /*!< AHB prescaler select CK_SYS/64 */
    RCU_SYSCLK_AHBDIV128 = RCU_AHB_CKSYS_DIV128,  /*!< AHB prescaler select CK_SYS/128 */
    RCU_SYSCLK_AHBDIV256 = RCU_AHB_CKSYS_DIV256,  /*!< AHB prescaler select CK_SYS/256 */
    RCU_SYSCLK_AHBDIV512 = RCU_AHB_CKSYS_DIV512   /*!< AHB prescaler select CK_SYS/512 */
}hal_rcu_sysclk_ahbdiv_enum;

/* APB1 prescaler selection */
typedef enum{
    RCU_AHBCLK_APB1DIV1  = RCU_APB1_CKAHB_DIV1,   /*!< APB1 prescaler select CK_AHB */
    RCU_AHBCLK_APB1DIV2  = RCU_APB1_CKAHB_DIV2,   /*!< APB1 prescaler select CK_AHB/2 */
    RCU_AHBCLK_APB1DIV4  = RCU_APB1_CKAHB_DIV4,   /*!< APB1 prescaler select CK_AHB/4 */
    RCU_AHBCLK_APB1DIV8  = RCU_APB1_CKAHB_DIV8,   /*!< APB1 prescaler select CK_AHB/8 */
    RCU_AHBCLK_APB1DIV16 = RCU_APB1_CKAHB_DIV16   /*!< APB1 prescaler select CK_AHB/16 */
}hal_rcu_ahbclk_apbldiv_enum;

/* APB2 prescaler selection */
typedef enum{
    RCU_AHBCLK_APB2DIV1  = RCU_APB2_CKAHB_DIV1,   /*!< APB2 prescaler select CK_AHB */
    RCU_AHBCLK_APB2DIV2  = RCU_APB2_CKAHB_DIV2,   /*!< APB2 prescaler select CK_AHB/2 */
    RCU_AHBCLK_APB2DIV4  = RCU_APB2_CKAHB_DIV4,   /*!< APB2 prescaler select CK_AHB/4 */
    RCU_AHBCLK_APB2DIV8  = RCU_APB2_CKAHB_DIV8,   /*!< APB2 prescaler select CK_AHB/8 */
    RCU_AHBCLK_APB2DIV16 = RCU_APB2_CKAHB_DIV16   /*!< APB2 prescaler select CK_AHB/16 */
}hal_rcu_ahbclk_apb2div_enum;

/* RCU clock out source */
typedef enum{
    RCU_CKOUT_SRC_NONE       = 0U,        /*!< no clock selected */
    RCU_CKOUT_SRC_IRC28M     = 1U,        /*!< CK_OUT clock source select IRC28M */
    RCU_CKOUT_SRC_IRC40K     = 2U,        /*!< CK_OUT clock source select IRC40K */
    RCU_CKOUT_SRC_LXTAL      = 3U,        /*!< CK_OUT clock source select LXTAL */
    RCU_CKOUT_SRC_CKSYS      = 4U,        /*!< CK_OUT clock source select CKSYS */
    RCU_CKOUT_SRC_IRC8M      = 5U,        /*!< CK_OUT clock source select IRC8M */
    RCU_CKOUT_SRC_HXTAL      = 6U,        /*!< CK_OUT clock source select HXTAL */
    RCU_CKOUT_SRC_CKPLL_DIV2 = 7U,        /*!< CK_OUT clock source select PLL/2 */
    RCU_CKOUT_SRC_CKPLL_DIV1 = 8U         /*!< CK_OUT clock source select PLL */
}hal_rcu_ckout_src_enum;

/* RCU clock out divider */
typedef enum{
    RCU_CLKOUT_DIV1   = RCU_CKOUT_DIV1,          /*!< CK_OUT is divided by 1 */
    RCU_CLKOUT_DIV2   = RCU_CKOUT_DIV2,          /*!< CK_OUT is divided by 2 */
    RCU_CLKOUT_DIV4   = RCU_CKOUT_DIV4,          /*!< CK_OUT is divided by 4 */
    RCU_CLKOUT_DIV8   = RCU_CKOUT_DIV8,          /*!< CK_OUT is divided by 8 */
    RCU_CLKOUT_DIV16  = RCU_CKOUT_DIV16,         /*!< CK_OUT is divided by 16 */
    RCU_CLKOUT_DIV32  = RCU_CKOUT_DIV32,         /*!< CK_OUT is divided by 32 */
    RCU_CLKOUT_DIV64  = RCU_CKOUT_DIV64,         /*!< CK_OUT is divided by 64 */
    RCU_CLKOUT_DIV128 = RCU_CKOUT_DIV128,        /*!< CK_OUT is divided by 128 */
}hal_rcu_ckout_div_enum;

typedef rcu_adc_clock_enum hal_rcu_adc_clksrc_enum;

/* RCU clock */
typedef struct{
    uint32_t clock_type;                           /*!< rcu clock type to be configured */
    hal_rcu_sysclk_src_enum sysclk_source;         /*!< the system clock source */
    hal_rcu_sysclk_ahbdiv_enum ahbclk_divider;     /*!< the AHB clock divider */
    hal_rcu_ahbclk_apbldiv_enum apb1clk_divider;   /*!< the APB1 clock divider */
    hal_rcu_ahbclk_apb2div_enum apb2clk_divider;   /*!< the APB2 clock divider */
}hal_rcu_clk_struct;

/* RCU HXTAL status */
typedef struct{
    ControlStatus need_configure;             /*!< the oscillators configure flag */
    hal_rcu_osc_state_enum state;             /*!< the oscillators state */
}hal_rcu_hxtal_struct;

/* RCU LXTAL status */
typedef struct{
    ControlStatus need_configure;             /*!< the oscillators configure flag */
    hal_rcu_osc_state_enum state;             /*!< the oscillators state */
}hal_rcu_lxtal_struct;

/* RCU IRC8M status */
typedef struct{
    ControlStatus need_configure;             /*!< the oscillators configure flag */
    uint8_t adjust_value;                     /*!< the oscillators adjust value */
    hal_rcu_osc_state_enum state;             /*!< the oscillators state */
}hal_rcu_irc8m_struct;

/* RCU IRC28M status */
typedef struct{
    ControlStatus need_configure;             /*!< the oscillators configure flag */
    uint8_t adjust_value;                     /*!< the oscillators adjust value */
    hal_rcu_osc_state_enum state;             /*!< the oscillators state */
}hal_rcu_irc28m_struct;

/* RCU IRC40K status */
typedef struct{
    ControlStatus need_configure;             /*!< the oscillators configure flag */
    hal_rcu_osc_state_enum state;             /*!< the oscillators state */
}hal_rcu_irc40k_struct;

/* RCU PLL */
typedef struct{
    ControlStatus need_configure;             /*!< the oscillators configure flag */
    hal_rcu_osc_state_enum state;             /*!< the oscillators state */
    hal_rcu_pll_src_enum pll_source;          /*!< PLL entry clock source */
    hal_rcu_pll_prediv_enum pre_div;          /*!< predivision factor of PLL input clock */
    hal_rcu_pll_mul_enum pll_mul;             /*!< multiplication factor of PLL input clock */
}hal_rcu_pll_struct;

/* RCU oscillators */
typedef struct{
    hal_rcu_hxtal_struct hxtal;               /*!< HXTAL status structure */
    hal_rcu_lxtal_struct lxtal;               /*!< LXTAL status structure */
    hal_rcu_irc8m_struct irc8m;               /*!< IRC8M status structure */
    hal_rcu_irc28m_struct irc28m;             /*!< IRC28M status structure */
    hal_rcu_irc40k_struct irc40k;             /*!< IRC40K status structure */
    hal_rcu_pll_struct pll;                   /*!< PLL structure parameters */
}hal_rcu_osci_struct;

/* RCU peripheal clock */
typedef struct{
    uint32_t periph_clock_type;                          /*!< peripherals clock type selection */
    hal_rcu_rtc_clksrc_enum rtc_clock_source;            /*!< RTC clock source selection */
    hal_rcu_usart_clksrc_enum usart0_clock_source;       /*!< usart0 clock source selection */
    hal_rcu_adc_clksrc_enum adc_clock_source;            /*!< ADC clock source selection */
}hal_rcu_periphclk_struct;

/* RCU interrupt handler structure */
typedef struct{
    hal_irq_handle_cb pll_stable_handle;       /*!< PLL clock stable interrupt */
    hal_irq_handle_cb irc40k_stable_handle;    /*!< IRC40K clock stable interrupt */
    hal_irq_handle_cb irc8m_stable_handle;     /*!< IRC8M clock stable interrupt */
    hal_irq_handle_cb irc28m_stable_handle;    /*!< IRC28M clock stable interrupt */
    hal_irq_handle_cb lxtal_stable_handle;     /*!< LXTAL clock stable interrupt */
    hal_irq_handle_cb hxtal_stable_handle;     /*!< HXTAL clock stable interrupt */
    hal_irq_handle_cb hxtal_stuck_handle;      /*!< HXTAL clock stuck interrupt */
}hal_rcu_irq_struct;

#define RCU_BP_TIMEOUT                        100U    /*!< disable backup domain write protection state timeout (in ms) */

#ifndef RCU_LXTAL_TIMEOUT
    #define RCU_LXTAL_TIMEOUT                 5000U   /*!< LXTAL state change timeout (in ms) */
#endif

#ifndef RCU_HXTAL_TIMEOUT
    #define RCU_HXTAL_TIMEOUT                 100U    /*!< HXTAL state change timeout (in ms) */
#endif

#define RCU_IRC8M_TIMEOUT                     4U      /*!< IRC8M state change timeout (in ms) */
#define RCU_IRC28M_TIMEOUT                    4U      /*!< IRC28M state change timeout (in ms) */
#define RCU_IRC40K_TIMEOUT                    4U      /*!< IRC40K state change timeout (in ms) */
#define RCU_PLL_TIMEOUT                       4U      /*!< PLL state change timeout (in ms) */

#define RCU_CLKTYPE_NONE                      (uint32_t)(0x00000000U) /*!< no clock to configure */
#define RCU_CLKTYPE_SYSCLK                    (uint32_t)(0x00000001U) /*!< system clock to configure */
#define RCU_CLKTYPE_AHBCLK                    (uint32_t)(0x00000002U) /*!< AHB clock to configure */
#define RCU_CLKTYPE_APB1CLK                   (uint32_t)(0x00000004U) /*!< APB1 clock to configure */
#define RCU_CLKTYPE_APB2CLK                   (uint32_t)(0x00000008U) /*!< APB2 clock to configure */
#define RCU_CLKTYPE_MAX                       (uint32_t)(0x0000000FU) /*!< all clock type to configure */

#define RCU_PERIPH_CLKTYPE_NONE               (uint32_t)(0x00000000U) /*!< no peripheal clock to configure */
#define RCU_PERIPH_CLKTYPE_RTC                (uint32_t)(0x00000001U) /*!< RTC peripheal clock to configure */
#define RCU_PERIPH_CLKTYPE_USART0             (uint32_t)(0x00000002U) /*!< USART0 peripheal clock to configure */
#define RCU_PERIPH_CLKTYPE_ADC                (uint32_t)(0x00000004U) /*!< ADC peripheal clock to configure */
#define RCU_PERIPH_CLKTYPE_APB1TIMER          (uint32_t)(0x00000008U) /*!< APB1 timer peripheal clock to get */
#define RCU_PERIPH_CLKTYPE_APB2TIMER          (uint32_t)(0x00000010U) /*!< APB2 timer peripheal clock to get */
#define RCU_PERIPH_CLKTYPE_MAX                (uint32_t)(0x0000001FU) /*!< all peripheal clock type to configure */

extern hal_rcu_irq_struct rcu_irq;

/* function declarations */
/* deinitialize the RCU */
void hal_rcu_deinit(void);

/* initialize the RCU structure */
void hal_rcu_struct_init(hal_rcu_struct_type_enum rcu_struct_type, void *p_struct);

/* enable the peripherals clock */
void hal_rcu_periph_clk_enable(rcu_periph_enum periph);

/* disable the peripherals clock */
void hal_rcu_periph_clk_disable(rcu_periph_enum periph);

/* initializes the RCU extended peripherals(RTC, Usart0 and ADC) clocks */
int32_t hal_rcu_periph_clock_config(hal_rcu_periphclk_struct *periph_clk);

/* get the peripherals clock frequency */
uint32_t hal_rcu_periph_clkfreq_get(uint32_t periph_clk);

/* configure the RCU oscillators */
int32_t hal_rcu_osci_config(hal_rcu_osci_struct *rcu_osci);

/* get the RCU oscillators configuration */
void hal_rcu_osci_config_get(hal_rcu_osci_struct *rcu_osci);

/* configure the RCU clock */
int32_t hal_rcu_clock_config(hal_rcu_clk_struct *rcu_clk, uint8_t fmc_wscnt);

/* get the RCU clock configuration */
void hal_rcu_clock_config_get(hal_rcu_clk_struct *rcu_clk, uint8_t *fmc_wscnt);

/* configure the clock out to output on CKOUT pin */
void hal_rcu_clock_out_config(hal_rcu_ckout_src_enum ckout_src, hal_rcu_ckout_div_enum ckout_div);

/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_rcu_irq_handle_set(hal_rcu_irq_struct *prcu_irq);

/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_rcu_irq_handle_all_reset(void);

/* RCU interrupt handler content function,which is merely used in RCU_handler */
void hal_rcu_irq(void);

#endif /* GD32E23X_HAL_RCU_H */
