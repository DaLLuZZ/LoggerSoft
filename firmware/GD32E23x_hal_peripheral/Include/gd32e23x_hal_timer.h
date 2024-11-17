/*!
    \file    gd32e23x_hal_timer.h
    \brief   definitions for the TIMER
    
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

#ifndef GD32E23X_HAL_TIMER_H
#define GD32E23X_HAL_TIMER_H

#include "gd32e23x_hal.h"

/* init struct type */
typedef enum {
    HAL_TIMER_BASIC_STRUCT,                                                         /*!< hal_timer_basic_struct */
    HAL_TIMER_CLOCKSOURCE_STRUCT,                                                   /*!< hal_timer_clocksource_struct */
    HAL_TIMER_SLAVEMODE_STRUCT,                                                     /*!< hal_timer_slavemode_struct */
    HAL_TIMER_INPUTCAPTURE_STRUCT,                                                  /*!< hal_timer_inputcapture_struct */
    HAL_TIMER_QUADRATURE_DECODER_STRUCT,                                            /*!< hal_timer_quadrature_decoder_struct */
    HAL_TIMER_SINGLEPULSE_STRUCT,                                                   /*!< hal_timer_singlepulse_struct */
    HAL_TIMER_OUTPUTCOMPARE_STRUCT,                                                 /*!< hal_timer_outputcompare_struct */
    HAL_TIMER_OCPRE_CLEAR_STRUCT,                                                   /*!< hal_timer_ocper_clear_struct */
    HAL_TIMER_HALL_STRUCT,                                                          /*!< hal_timer_hall_struct */
    HAL_TIMER_BREAKCONFIG_STRUCT,                                                   /*!< hal_timer_breakconfig_struct */
    HAL_TIMER_IRQ_STRUCT,                                                           /* TIMER device interrupt callback function pointer structure */
    HAL_TIMER_DEV_STRUCT                                                            /*!< TIMER device information struct */
} hal_timer_struct_type_enum;

/* the service channelx(x=0..3) */
typedef enum {
    TIMER_SERVICE_CHANNEL_NONE = 0x00,                                              /*!< none channel service */
    TIMER_SERVICE_CHANNEL_0,                                                        /*!< service chanenl0 */
    TIMER_SERVICE_CHANNEL_1,                                                        /*!< service chanenl1 */
    TIMER_SERVICE_CHANNEL_2,                                                        /*!< service chanenl2 */
    TIMER_SERVICE_CHANNEL_3                                                         /*!< service chanenl3 */
} hal_timer_service_channel_enum;

/* master mode control */
typedef enum {
    TIMRE_TRGO_SRC_RESET  = TIMER_TRI_OUT_SRC_RESET,                                /*!< the selection of TRGO signal:RESET */                   
    TIMRE_TRGO_SRC_ENABLE = TIMER_TRI_OUT_SRC_ENABLE,                               /*!< the selection of TRGO signal:ENABLE */             
    TIMRE_TRGO_SRC_UPDATE = TIMER_TRI_OUT_SRC_UPDATE,                               /*!< the selection of TRGO signal:UPDATE */
    TIMRE_TRGO_SRC_CH0    = TIMER_TRI_OUT_SRC_CH0,                                  /*!< the selection of TRGO signal: channel0 capture/compare pulse */
    TIMRE_TRGO_SRC_O0CPRE = TIMER_TRI_OUT_SRC_O0CPRE,                               /*!< the selection of TRGO signal:O0CPRE */
    TIMRE_TRGO_SRC_O1CPRE = TIMER_TRI_OUT_SRC_O1CPRE,                               /*!< the selection of TRGO signal:O1CPRE */
    TIMRE_TRGO_SRC_O2CPRE = TIMER_TRI_OUT_SRC_O2CPRE,                               /*!< the selection of TRGO signal:O2CPRE */
    TIMRE_TRGO_SRC_O3CPRE = TIMER_TRI_OUT_SRC_O3CPRE                                /*!< the selection of TRGO signal:O3CPRE */
} hal_timer_trgo_selection_enum;

/* slave mode control */
typedef enum{
    TIMER_SLAVE_DISABLE_MODE = TIMER_SLAVE_MODE_DISABLE,                            /*!< slave mode disable */
    TIMER_SLAVE_MODE_RESTART_MODE = TIMER_SLAVE_MODE_RESTART,                       /*!< restart mode */
    TIMER_SLAVE_MODE_PAUSE_MODE = TIMER_SLAVE_MODE_PAUSE,                           /*!< pause mode */
    TIMER_SLAVE_MODE_EVENT_MODE = TIMER_SLAVE_MODE_EVENT,                           /*!< event mode */
    TIMER_SLAVE_MODE_EXTERNAL0_MODE = TIMER_SLAVE_MODE_EXTERNAL0                    /*!< external clock mode 0 */
} hal_timer_slave_mode_enum;

/*!< trigger input source */
typedef enum{
    TIMER_TRIGGER_SOURCE_ITI0 = TIMER_SMCFG_TRGSEL_ITI0,                            /*!< trigger input source selection:ITI0 */
    TIMER_TRIGGER_SOURCE_ITI1 = TIMER_SMCFG_TRGSEL_ITI1,                            /*!< trigger input source selection:ITI1 */
    TIMER_TRIGGER_SOURCE_ITI2 = TIMER_SMCFG_TRGSEL_ITI2,                            /*!< trigger input source selection:ITI2 */
    TIMER_TRIGGER_SOURCE_ITI3 = TIMER_SMCFG_TRGSEL_ITI3,                            /*!< trigger input source selection:ITI3 */
    TIMER_TRIGGER_SOURCE_CI0FE0 = TIMER_SMCFG_TRGSEL_CI0FE0,                        /*!< trigger input source selection:CI0FE0 */
    TIMER_TRIGGER_SOURCE_CI1FE1 = TIMER_SMCFG_TRGSEL_CI1FE1,                        /*!< trigger input source selection:CI1FE1 */
    TIMER_TRIGGER_SOURCE_CI0FED = TIMER_SMCFG_TRGSEL_CI0F_ED,                       /*!< trigger input source selection:CI0FED */
    TIMER_TRIGGER_SOURCE_ETIFP = (uint32_t)0x00000001,                              /*!< trigger input source selection:ETIFP */
    TIMER_TRIGGER_SOURCE_DISABLE = (uint32_t)0x00000002                             /*!< trigger input source selection:none */
} hal_timer_input_trigger_source_enum;

/*!< clock input and trigger input source polarity */
typedef enum{
    TIMER_CLOCK_TRIGGER_POLARITY_ETI_RISING = TIMER_ETP_RISING,                     /*!< clock input source is ETI, active high or rising edge active */
    TIMER_CLOCK_TRIGGER_POLARITY_ETI_FALLING = TIMER_ETP_FALLING,                   /*!< clock input source is ETI, active low or falling edge active */
    TIMER_CLOCK_TRIGGER_POLARITY_RISING = (uint32_t)TIMER_IC_POLARITY_RISING,       /*!< clock input source is CIx(x=0,1), rising edge active */
    TIMER_CLOCK_TRIGGER_POLARITY_FALLING = (uint32_t)TIMER_IC_POLARITY_FALLING,     /*!< clock input source is CIx(x=0,1), falling edge active */
    TIMER_CLOCK_TRIGGER_POLARITY_BOTH_EDGE = (uint32_t)TIMER_IC_POLARITY_BOTH_EDGE  /*!< clock input source is CI0F_ED, both rising and falling edge active */
} hal_timer_clock_trigger_polarity_enum;

/* clock input source */
typedef enum{
    TIMER_CLOCK_SOURCE_CK_TIMER = (uint32_t)0x00000001,                             /*!< clock input source selection:Internal clock CK_TIMER */
    TIMER_CLOCK_SOURCE_ITI0 = TIMER_SMCFG_TRGSEL_ITI0,                              /*!< clock input source selection:ITI0 */
    TIMER_CLOCK_SOURCE_ITI1 = TIMER_SMCFG_TRGSEL_ITI1,                              /*!< clock input source selection:ITI1 */
    TIMER_CLOCK_SOURCE_ITI2 = TIMER_SMCFG_TRGSEL_ITI2,                              /*!< clock input source selection:ITI2 */
    TIMER_CLOCK_SOURCE_ITI3 = TIMER_SMCFG_TRGSEL_ITI3,                              /*!< clock input source selection:ITI3 */
    TIMER_CLOCK_SOURCE_CI0FE0 = TIMER_SMCFG_TRGSEL_CI0FE0,                          /*!< clock input source selection:CI0FE0 */
    TIMER_CLOCK_SOURCE_CI1FE1 = TIMER_SMCFG_TRGSEL_CI1FE1,                          /*!< clock input source selection:CI1FE1 */
    TIMER_CLOCK_SOURCE_CI0FED = TIMER_SMCFG_TRGSEL_CI0F_ED,                         /*!< clock input source selection:CI0FED */
    TIMER_CLOCK_SOURCE_ETIMODE0 = TIMER_SMCFG_TRGSEL_ETIFP,                         /*!< clock input source selection:ETI in external clock mode0 */
    TIMER_CLOCK_SOURCE_ETIMODE1 = TIMER_SMCFG_SMC1                                  /*!< clock input source selection:ETI in external clock mode1 */
} hal_timer_clock_source_enum;

/* TIMER software event generation source */
typedef enum{
    TIMER_SOFTWARE_EVENT_UPDATE = TIMER_EVENT_SRC_UPG,                              /*!< update event generation */
    TIMER_SOFTWARE_EVENT_CH0 = TIMER_EVENT_SRC_CH0G,                                /*!< channel 0 capture or compare event generation */
    TIMER_SOFTWARE_EVENT_CH1 = TIMER_EVENT_SRC_CH1G,                                /*!< channel 1 capture or compare event generation */
    TIMER_SOFTWARE_EVENT_CH2 = TIMER_EVENT_SRC_CH2G,                                /*!< channel 2 capture or compare event generation */
    TIMER_SOFTWARE_EVENT_CH3 = TIMER_EVENT_SRC_CH3G,                                /*!< channel 3 capture or compare event generation */
    TIMER_SOFTWARE_EVENT_CMT = TIMER_EVENT_SRC_CMTG,                                /*!< channel commutation event generation */
    TIMER_SOFTWARE_EVENT_TRG = TIMER_EVENT_SRC_TRGG,                                /*!< trigger event generation */
    TIMER_SOFTWARE_EVENT_BRK = TIMER_EVENT_SRC_BRKG                                 /*!< break event generation */
}hal_timer_software_event_enum;

/* TIMER channel compare output control */
typedef enum{
    TIMER_OUTPUT_TIMING_MODE = TIMER_OC_MODE_TIMING,                                /*!< timing mode */
    TIMER_OUTPUT_ACTIVE_MODE = TIMER_OC_MODE_ACTIVE,                                /*!< active mode */
    TIMER_OUTPUT_INACTIVE_MODE = TIMER_OC_MODE_INACTIVE,                            /*!< inactive mode */
    TIMER_OUTPUT_TOGGLE_MODE = TIMER_OC_MODE_TOGGLE,                                /*!< toggle mode */
    TIMER_OUTPUT_LOW_MODE = TIMER_OC_MODE_LOW,                                      /*!< force low mode */
    TIMER_OUTPUT_HIGH_MODE = TIMER_OC_MODE_HIGH,                                    /*!< force high mode */
    TIMER_OUTPUT_PWM0_MODE = TIMER_OC_MODE_PWM0,                                    /*!< PWM0 mode */
    TIMER_OUTPUT_PWM1_MODE = TIMER_OC_MODE_PWM1                                     /*!< PWM1 mode*/
} hal_timer_output_compare_enum;

/* DMA transfer access start address */
typedef enum{
    TIMER_DMA_START_ADDRESS_CTL0 = TIMER_DMACFG_DMATA_CTL0,                         /*!< DMA transfer address is TIMER_CTL0 */
    TIMER_DMA_START_ADDRESS_CTL1 = TIMER_DMACFG_DMATA_CTL1,                         /*!< DMA transfer address is TIMER_CTL1 */
    TIMER_DMA_START_ADDRESS_SMCFG = TIMER_DMACFG_DMATA_SMCFG,                       /*!< DMA transfer address is TIMER_SMCFG */
    TIMER_DMA_START_ADDRESS_DMAINTEN = TIMER_DMACFG_DMATA_DMAINTEN,                 /*!< DMA transfer address is TIMER_DMAINTEN */
    TIMER_DMA_START_ADDRESS_INTF = TIMER_DMACFG_DMATA_INTF,                         /*!< DMA transfer address is TIMER_INTF */
    TIMER_DMA_START_ADDRESS_SWEVG = TIMER_DMACFG_DMATA_SWEVG,                       /*!< DMA transfer address is TIMER_SWEVG */
    TIMER_DMA_START_ADDRESS_CHCTL0 = TIMER_DMACFG_DMATA_CHCTL0,                     /*!< DMA transfer address is TIMER_CHCTL0 */
    TIMER_DMA_START_ADDRESS_CHCTL1 = TIMER_DMACFG_DMATA_CHCTL1,                     /*!< DMA transfer address is TIMER_CHCTL1 */
    TIMER_DMA_START_ADDRESS_CHCTL2 = TIMER_DMACFG_DMATA_CHCTL2,                     /*!< DMA transfer address is TIMER_CHCTL2 */
    TIMER_DMA_START_ADDRESS_CNT = TIMER_DMACFG_DMATA_CNT,                           /*!< DMA transfer address is TIMER_CNT */
    TIMER_DMA_START_ADDRESS_PSC = TIMER_DMACFG_DMATA_PSC,                           /*!< DMA transfer address is TIMER_PSC */
    TIMER_DMA_START_ADDRESS_CAR = TIMER_DMACFG_DMATA_CAR,                           /*!< DMA transfer address is TIMER_CAR */
    TIMER_DMA_START_ADDRESS_CREP = TIMER_DMACFG_DMATA_CREP,                         /*!< DMA transfer address is TIMER_CREP */
    TIMER_DMA_START_ADDRESS_CH0CV = TIMER_DMACFG_DMATA_CH0CV,                       /*!< DMA transfer address is TIMER_CH0CV */
    TIMER_DMA_START_ADDRESS_CH1CV = TIMER_DMACFG_DMATA_CH1CV,                       /*!< DMA transfer address is TIMER_CH1CV */
    TIMER_DMA_START_ADDRESS_CH2CV = TIMER_DMACFG_DMATA_CH2CV,                       /*!< DMA transfer address is TIMER_CH2CV */
    TIMER_DMA_START_ADDRESS_CH3CV = TIMER_DMACFG_DMATA_CH3CV,                       /*!< DMA transfer address is TIMER_CH3CV */
    TIMER_DMA_START_ADDRESS_CCHP = TIMER_DMACFG_DMATA_CCHP,                         /*!< DMA transfer address is TIMER_CCHP */
    TIMER_DMA_START_ADDRESS_DMACFG = TIMER_DMACFG_DMATA_DMACFG                      /*!< DMA transfer address is TIMER_DMACFG */
} hal_timer_dma_start_address_enum;

/* TIMER DMA mode request source */
typedef enum{
    TIMER_DMA_REQUEST_SOURCE_UPDATE = TIMER_DMA_UPD,                                /*!< update DMA enable */
    TIMER_DMA_REQUEST_SOURCE_CH0 = TIMER_DMA_CH0D,                                  /*!< channel 0 DMA enable */
    TIMER_DMA_REQUEST_SOURCE_CH1 = TIMER_DMA_CH1D,                                  /*!< channel 1 DMA enable */
    TIMER_DMA_REQUEST_SOURCE_CH2 = TIMER_DMA_CH2D,                                  /*!< channel 2 DMA enable */
    TIMER_DMA_REQUEST_SOURCE_CH3 = TIMER_DMA_CH3D,                                  /*!< channel 3 DMA enable */
    TIMER_DMA_REQUEST_SOURCE_CMT = TIMER_DMA_CMTD,                                  /*!< commutation DMA request enable */
    TIMER_DMA_REQUEST_SOURCE_TRG = TIMER_DMA_TRGD                                   /*!< trigger DMA enable */
} hal_timer_dma_request_source_enum;

/* TIMER device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb dmaerror_usercb;                      /*!< DMA channel error interrupt call */
    hal_irq_handle_cb dmatc_update_usercb;                  /*!< DMA transmission complete interrupt(TC) user callback for update dma request */
    hal_irq_handle_cb dmatc_chx_cmppwm_usercb;              /*!< DMA transmission complete interrupt(TC) user callback for channelx compare output and pwm output dma request */
    hal_irq_handle_cb dmatc_chx_capture_usercb;             /*!< DMA transmission complete interrupt(TC) user callback for channelx input capture dma request */
    hal_irq_handle_cb dmatc_commutation_usercb;             /*!< DMA transmission complete interrupt(TC) user callback for commutation DMA request */
    hal_irq_handle_cb dmatc_trigger_usercb;                 /*!< DMA transmission complete interrupt(TC) user callback for trigger DMA request */
    hal_irq_handle_cb update_usercb;                        /*!< TIMER update interrupt call */
    hal_irq_handle_cb channelx_compare_usercb;              /*!< channel interrupt for compare output */
    hal_irq_handle_cb channelx_pwm_usercb;                  /*!< channel interrupt for PWM output */
    hal_irq_handle_cb channelx_capture_usercb;              /*!< channel interrupt for input capture */
    hal_irq_handle_cb commutation_usercb;                   /*!< TIMER commutation interrupt call */
    hal_irq_handle_cb trigger_usercb;                       /*!< TIMER trigger interrupt call */
    hal_irq_handle_cb break_usercb;                         /*!< TIMER break interrupt call */
} hal_timer_dev_irq_struct;

/* TIMER device infomation struct*/
typedef struct{
    uint32_t periph;                                        /*!< TIMERx(x=0,2,5,13..16) */
    hal_timer_service_channel_enum service_channel;         /*!< the service channelx */
    hal_dma_dev_struct *hdma[7];                            /*!< DMA dev infomation index */
    hal_timer_dev_irq_struct timer_irq;                     /*!< TIMER device interrupt callback function pointer structure */
}hal_timer_dev_struct;

/*TIMER user call back function */
typedef void (*hal_timer_user_cb)(hal_timer_dev_struct *timer_dev);

/* TIMER interrupt user callback function pointer structure */
typedef struct {
    hal_timer_user_cb dmaerror_usercb;                      /*!< DMA channel error interrupt call */
    hal_timer_user_cb dmatc_update_usercb;                  /*!< DMA transmission complete interrupt(TC) user callback for update dma request */
    hal_timer_user_cb dmatc_chx_cmppwm_usercb;              /*!< DMA transmission complete interrupt(TC) user callback for channelx compare output and pwm output dma request */
    hal_timer_user_cb dmatc_chx_capture_usercb;             /*!< DMA transmission complete interrupt(TC) user callback for channelx input capture dma request */
    hal_timer_user_cb dmatc_commutation_usercb;             /*!< DMA transmission complete interrupt(TC) user callback for commutation DMA request */
    hal_timer_user_cb dmatc_trigger_usercb;                 /*!< DMA transmission complete interrupt(TC) user callback for trigger DMA request */
    hal_timer_user_cb update_usercb;                        /*!< TIMER update interrupt call */
    hal_timer_user_cb channelx_compare_usercb;              /*!< channel interrupt for compare output */
    hal_timer_user_cb channelx_pwm_usercb;                  /*!< channel interrupt for PWM output */
    hal_timer_user_cb channelx_capture_usercb;              /*!< channel interrupt for input capture */
    hal_timer_user_cb commutation_usercb;                   /*!< TIMER commutation interrupt call */
    hal_timer_user_cb trigger_usercb;                       /*!< TIMER trigger interrupt call */
    hal_timer_user_cb break_usercb;                         /*!< TIMER break interrupt call */
} hal_timer_irq_struct;

/* TIMER basic config struct */
typedef struct{
    uint16_t prescaler;                                     /*!< prescaler value */
    uint16_t alignedmode;                                   /*!< aligned mode */
    uint16_t counterdirection;                              /*!< counter direction */
    uint32_t period;                                        /*!< period value */
    uint16_t clockdivision;                                 /*!< clock division value */
    uint8_t  repetitioncounter;                             /*!< the counter repetition value */
    uint32_t autoreload_shadow;                             /*!< auto-reload shadow enable or disable, TIMERx_CTL0:bit7(ARSE) */
    hal_timer_trgo_selection_enum trgo_selection;           /*!< master mode control, TIMERx_CTL1:bit4~6(MMC) */
    uint32_t master_slave_mode;                             /*!< master-slave mode, TIMERx_SMCFG:bit7(MSM) */
}hal_timer_basic_struct;

/* TIMER clock source config struct */
typedef struct{
    hal_timer_clock_source_enum clock_source;               /*!< select clock input source */
    hal_timer_clock_trigger_polarity_enum clock_polarity;   /*!< clock input source polarity */
    uint32_t clock_prescaler;                               /*!< clock input source prescaler */
    uint32_t clock_filter;                                  /*!< clock input source filter, 0x00~0x0F */
}hal_timer_clocksource_struct;

/* TIMER slave mode config struct */
typedef struct{
    hal_timer_slave_mode_enum slavemode;                    /*!< slave mode control */
    hal_timer_input_trigger_source_enum trigger_selection;  /*!< trigger selection */
    hal_timer_clock_trigger_polarity_enum trigger_polarity; /*!< trigger input source polarity */
    uint32_t trigger_prescaler;                             /*!< trigger input source prescaler */
    uint32_t trigger_filter;                                /*!< trigger input source filter, 0x00~0x0F */
}hal_timer_slavemode_struct;

/* TIMER quadrature decoder mode struct */
typedef struct{
    uint32_t decodemode;                                    /*!< quadrature decoder mode */
    uint32_t ci0_selection;                                 /*!< channel0 I/O mode selection */
    uint32_t ci0_polarity;                                  /*!< channel0 input capture polarity */
    uint32_t ci0_filter;                                    /*!< channel0 input capture filter control */
    uint32_t ci0_prescaler;                                 /*!< channel0 input capture prescaler */
    uint32_t ci1_selection;                                 /*!< channel1 I/O mode selection */
    uint32_t ci1_polarity;                                  /*!< channel1 input capture polarity */
    uint32_t ci1_filter;                                    /*!< channel1 input capture filter control */
    uint32_t ci1_prescaler;                                 /*!< channel1 input capture prescaler */
}hal_timer_quadrature_decoder_struct;

/* TIMER output compare mode struct */
typedef struct{
    hal_timer_output_compare_enum output_mode;              /*!< channel compare output control */
    uint32_t output_pulse;                                  /*!< channel capture or compare value */
    uint32_t output_fastmode;                               /*!< channel output compare fast */
    uint16_t output_polarity;                               /*!< channel output polarity */
    uint16_t outputn_polarity;                              /*!< channel complementary output polarity */
    uint16_t output_idlestate;                              /*!< idle state of channel output */
    uint16_t outputn_idlestate;                             /*!< idle state of channel complementary output */
}hal_timer_outputcompare_struct;

/* TIMER single pulse mode struct */
typedef struct{
    hal_timer_output_compare_enum sp_ocmode;                /*!< channel compare output control */
    uint32_t sp_pulse;                                      /*!< channel capture or compare value */
    uint16_t sp_ocpolarity;                                 /*!< channel output polarity */
    uint16_t sp_ocnpolarity;                                /*!< channel complementary output polarity */
    uint16_t sp_ocidlestate;                                /*!< idle state of channel output */
    uint16_t sp_ocnidlestate;                               /*!< idle state of channel complementary output */
    uint32_t sp_ci_selection;                               /*!< channel1 I/O mode selection */
    uint32_t sp_ci_polarity;                                /*!< channel1 input capture polarity */
    uint32_t sp_ci_filter;                                  /*!< channel1 input capture filter control */
}hal_timer_singlepulse_struct;

/* TIMER output compare clear struct */
typedef struct{
    uint32_t ocpre_clear_source;                            /*!< OCPRE clear source selection */
    uint32_t ocpre_clear_state;                             /*!< ENABLE or DISABLE */
    uint32_t trigger_polarity;                              /*!< external trigger polarity */
    uint32_t trigger_prescaler;                             /*!< external trigger prescaler */
    uint32_t trigger_filter;                                /*!< external trigger filter control */
}hal_timer_ocpre_clear_struct;

/* TIMER HALL sensor interface function */
typedef struct{
    uint32_t cmt_delay;                                     /*!< commutation delay(channel 1 value) */
    uint32_t hall_polarity;                                 /*!< channel0 input capture polarity */
    uint32_t hall_filter;                                   /*!< channel0 input capture filter control */
    uint32_t hall_prescaler;                                /*!< channel0 input capture prescaler */
}hal_timer_hall_struct;

/* TIMER channel input capture parameter struct */
typedef timer_ic_parameter_struct hal_timer_inputcapture_struct;

/* TIMER break parameter struct */
typedef timer_break_parameter_struct hal_timer_breakconfig_struct;

/* registers definitions */
#define TIMER_CTL0_ADDRESS(timerx)               (uint32_t)((timerx) + 0x00U)       /*!< TIMER control register 0 */
#define TIMER_CTL1_ADDRESS(timerx)               (uint32_t)((timerx) + 0x04U)      /*!< TIMER control register 1 */
#define TIMER_SMCFG_ADDRESS(timerx)              (uint32_t)((timerx) + 0x08U)      /*!< TIMER slave mode configuration register */
#define TIMER_DMAINTEN_ADDRESS(timerx)           (uint32_t)((timerx) + 0x0CU)      /*!< TIMER DMA and interrupt enable register */
#define TIMER_INTF_ADDRESS(timerx)               (uint32_t)((timerx) + 0x10U)      /*!< TIMER interrupt flag register */
#define TIMER_SWEVG_ADDRESS(timerx)              (uint32_t)((timerx) + 0x14U)      /*!< TIMER software event generation register */
#define TIMER_CHCTL0_ADDRESS(timerx)             (uint32_t)((timerx) + 0x18U)      /*!< TIMER channel control register 0 */
#define TIMER_CHCTL1_ADDRESS(timerx)             (uint32_t)((timerx) + 0x1CU)      /*!< TIMER channel control register 1 */
#define TIMER_CHCTL2_ADDRESS(timerx)             (uint32_t)((timerx) + 0x20U)      /*!< TIMER channel control register 2 */
#define TIMER_CNT_ADDRESS(timerx)                (uint32_t)((timerx) + 0x24U)      /*!< TIMER counter register */
#define TIMER_PSC_ADDRESS(timerx)                (uint32_t)((timerx) + 0x28U)      /*!< TIMER prescaler register */
#define TIMER_CAR_ADDRESS(timerx)                (uint32_t)((timerx) + 0x2CU)      /*!< TIMER counter auto reload register */
#define TIMER_CREP_ADDRESS(timerx)               (uint32_t)((timerx) + 0x30U)      /*!< TIMER counter repetition register */
#define TIMER_CH0CV_ADDRESS(timerx)              (uint32_t)((timerx) + 0x34U)      /*!< TIMER channel 0 capture/compare value register */
#define TIMER_CH1CV_ADDRESS(timerx)              (uint32_t)((timerx) + 0x38U)      /*!< TIMER channel 1 capture/compare value register */
#define TIMER_CH2CV_ADDRESS(timerx)              (uint32_t)((timerx) + 0x3CU)      /*!< TIMER channel 2 capture/compare value register */
#define TIMER_CH3CV_ADDRESS(timerx)              (uint32_t)((timerx) + 0x40U)      /*!< TIMER channel 3 capture/compare value register */
#define TIMER_CCHP_ADDRESS(timerx)               (uint32_t)((timerx) + 0x44U)      /*!< TIMER complementary channel protection register */
#define TIMER_DMACFG_ADDRESS(timerx)             (uint32_t)((timerx) + 0x48U)      /*!< TIMER DMA configuration register */
#define TIMER_DMATB_ADDRESS(timerx)              (uint32_t)((timerx) + 0x4CU)      /*!< TIMER DMA transfer buffer register */
#define TIMER_IRMP_ADDRESS(timerx)               (uint32_t)((timerx) + 0x50U)      /*!< TIMER channel input remap register */
#define TIMER_CFG_ADDRESS(timerx)                (uint32_t)((timerx) + 0xFCU)      /*!< TIMER configuration register */

/* DMA dev infomation index */
#define TIM_DMA_ID_UP                       ((uint8_t) 0x0U)                        /*!< specify update DMA */
#define TIM_DMA_ID_CH0                      ((uint8_t) 0x1U)                        /*!< specify channel 0 DMA */
#define TIM_DMA_ID_CH1                      ((uint8_t) 0x2U)                        /*!< specify channel 1 DMA */
#define TIM_DMA_ID_CH2                      ((uint8_t) 0x3U)                        /*!< specify channel 2 DMA */
#define TIM_DMA_ID_CH3                      ((uint8_t) 0x4U)                        /*!< specify channel 3 DMA */
#define TIM_DMA_ID_CMT                      ((uint8_t) 0x5U)                        /*!< specify commutation DMA request */
#define TIM_DMA_ID_TRG                      ((uint8_t) 0x6U)                        /*!< specify trigger DMA */

/* auto-reload shadow enable or disable,TIMERx_CTL0:bit7(ARSE) */
#define AUTO_RELOAD_SHADOW_ENABLE           BIT(7)
#define AUTO_RELOAD_SHADOW_DISABLE          (uint32_t)(0x00000000)

/* clock input source prescaler */
#define TIMER_CLOCK_PRESCALER_DIV1          TIMER_EXT_TRI_PSC_OFF                   /*!< clock input source no divided for ETI */
#define TIMER_CLOCK_PRESCALER_DIV2          TIMER_EXT_TRI_PSC_DIV2                  /*!< clock input source divided by 2 for ETI */
#define TIMER_CLOCK_PRESCALER_DIV4          TIMER_EXT_TRI_PSC_DIV4                  /*!< clock input source divided by 4 for ETI */
#define TIMER_CLOCK_PRESCALER_DIV8          TIMER_EXT_TRI_PSC_DIV8                  /*!< clock input source divided by 8 for ETI */

/* channel 0 trigger input selection */
#define TIMER_CI0_CH0IN                     TIMER_HALLINTERFACE_DISABLE             /*!< The TIMERx_CH0 pin input is selected as channel 0 trigger input. */
#define TIMER_CI0_XOR_CH012                 TIMER_HALLINTERFACE_ENABLE              /*!< The result of combinational XOR of TIMERx_CH0, CH1 and CH2 pins is selected as channel 0 trigger input */

/*  */
#define TIMER_QUADRATURE_DECODER_MODE0      TIMER_ENCODER_MODE0                     /*!< quadrature decoder mode 0 */
#define TIMER_QUADRATURE_DECODER_MODE1      TIMER_ENCODER_MODE1                     /*!< quadrature decoder mode 1 */
#define TIMER_QUADRATURE_DECODER_MODE2      TIMER_ENCODER_MODE2                     /*!< quadrature decoder mode 2 */

/* channel mask */
#define TIMER_CHX_EN_MASK                   ((uint32_t)(TIMER_CHCTL2_CH0EN|\
                                            TIMER_CHCTL2_CH1EN|TIMER_CHCTL2_CH2EN|\
                                            TIMER_CHCTL2_CH3EN))                    /*!< channel enable mask */
#define TIMER_CHNX_EN_MASK                  ((uint32_t)(TIMER_CHCTL2_CH0NEN|\
                                            TIMER_CHCTL2_CH1NEN|\
                                            TIMER_CHCTL2_CH2NEN))                   /*!< complementary channel enable mask */

/* function declarations */
/* initialize the TIMER structure with the default values */
int32_t hal_timer_struct_init(hal_timer_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize TIMER */
int32_t hal_timer_deinit(hal_timer_dev_struct *timer_dev);

/*  timebase settings*/
/* initialize TIMER timebase  */
int32_t hal_timer_basic_init(hal_timer_dev_struct *timer_dev, uint32_t timer_periph, hal_timer_basic_struct *timer_basic);
/* TIMER counter enable */
int32_t hal_timer_start_counter(hal_timer_dev_struct *timer_dev);
/* TIMER counter disable */
int32_t hal_timer_stop_counter(hal_timer_dev_struct *timer_dev);
/* TIMER counter and update interrupt enable */
int32_t hal_timer_start_counter_interrupt(hal_timer_dev_struct *timer_dev);
/* TIMER counter and update interrupt stop */
int32_t hal_timer_stop_counter_interrupt(hal_timer_dev_struct *timer_dev);
/* TIMER counter and update DMA request enable */
int32_t hal_timer_start_counter_dma(hal_timer_dev_struct *timer_dev, uint32_t *buffer, uint16_t blength);
/* TIMER counter and update DMA request disable */
int32_t hal_timer_stop_counter_dma(hal_timer_dev_struct *timer_dev);

/* slave mode settings */
/* TIMER slave mode congfig */
int32_t hal_timer_slave_mode_config(hal_timer_dev_struct *timer_dev, hal_timer_slavemode_struct *timer_slavemode);
/* TIMER slave mode congfig and interrupt congfig */
int32_t hal_timer_slave_mode_interrupt_config(hal_timer_dev_struct *timer_dev, hal_timer_slavemode_struct *timer_slavemode);

/* timer clock source config */
int32_t hal_timer_clock_source_config(hal_timer_dev_struct *timer_dev, hal_timer_clocksource_struct *timer_clock);
/* TIMER break config */
int32_t hal_timer_break_config(hal_timer_dev_struct *timer_dev, hal_timer_breakconfig_struct *break_cofig);
/* timer OxCPRE signal clear config  */
int32_t hal_timer_ocpre_clear_config(hal_timer_dev_struct *timer_dev,hal_timer_ocpre_clear_struct *oc_clear, uint32_t channel);
/* TIMER CI0 trigger input selection*/
int32_t hal_timer_ci0_input_selection(hal_timer_dev_struct *timer_dev,uint32_t ci0_select);
/* TIMERx remap */
int32_t hal_timer_remap_config(hal_timer_dev_struct *timer_dev,uint32_t remap);
/* TIMER generate a software event */
int32_t hal_timer_generate_event(hal_timer_dev_struct *timer_dev, hal_timer_software_event_enum event_src);

/* compare output settings */
/* initialize TIMER output compare mode */
int32_t hal_timer_outputcompare_init(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_outputcompare_struct *timer_compare);
/* start TIMER output compare mode */
int32_t hal_timer_start_outputcompare(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* stop TIMER output compare mode */
int32_t hal_timer_stop_outputcompare(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER output compare mode and channel interrupt */
int32_t hal_timer_start_outputcompare_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* stop TIMER output compare mode and channel interrupt */
int32_t hal_timer_stop_outputcompare_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER output compare mode and channel dma request */
int32_t hal_timer_start_outputcompare_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, uint32_t *buffer, uint16_t blength);
/* stop TIMER output compare mode and channel dma request */
int32_t hal_timer_stop_outputcompare_dma(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER complementary output compare mode */
int32_t hal_timer_start_outputcompare_negtive(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* stop TIMER complementary output compare mode */
int32_t hal_timer_stop_outputcompare_negtive(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER complementary output compare mode and channel interrupt */
int32_t hal_timer_start_outputcompare_negtive_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* stop TIMER complementary output compare mode and channel interrupt */
int32_t hal_timer_stop_outputcompare_negtive_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER complementary output compare mode and channel dma request */
int32_t hal_timer_start_outputcompare_negtive_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, uint32_t *buffer, uint16_t blength);
/* stop TIMER complementary output compare mode and channel dma request */
int32_t hal_timer_stop_outputcompare_negtive_dma(hal_timer_dev_struct *timer_dev, uint16_t channel);

/* pwm output settings */
/* initialize TIMER pwm mode */
int32_t hal_timer_pwm_init(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_outputcompare_struct *timer_compare);
/* start TIMER pwm mode */
int32_t hal_timer_start_pwm(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* stop TIMER pwm mode */
int32_t hal_timer_stop_pwm(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER pwm mode and channel interrupt */
int32_t hal_timer_start_pwm_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* stop TIMER pwm mode and channel interrupt */
int32_t hal_timer_stop_pwm_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER pwm mode and channel dma request */
int32_t hal_timer_start_pwm_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, uint32_t *buffer, uint16_t blength);
/* stop TIMER pwm mode and channel dma request */
int32_t hal_timer_stop_pwm_dma(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER complementary pwm mode */
int32_t hal_timer_start_pwm_negtive(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* stop TIMER complementary pwm mode */
int32_t hal_timer_stop_pwm_negtive(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER complementary pwm mode and channel interrupt */
int32_t hal_timer_start_pwm_negtive_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* stop TIMER complementary pwm mode and channel interrupt */
int32_t hal_timer_stop_pwm_negtive_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* start TIMER complementary pwm mode and channel dma request */
int32_t hal_timer_start_pwm_negtive_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, uint32_t *buffer, uint16_t blength);
/* stop TIMER complementary pwm mode and channel dma request */
int32_t hal_timer_stop_pwm_negtive_dma(hal_timer_dev_struct *timer_dev, uint16_t channel);

/* input capture settings*/
/* initialize TIMER input capture mode */
int32_t hal_timer_inputcapture_init(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_inputcapture_struct *timer_capture);
/* start TIMER input capture mode */
int32_t hal_timer_start_inputcapture(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* stop TIMER input capture mode */
int32_t hal_timer_stop_inputcapture(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* start TIMER input capture mode and channel interrupt */
int32_t hal_timer_start_inputcapture_interrupt(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* stop TIMER input capture mode and channel interrupt */
int32_t hal_timer_stop_inputcapture_interrupt(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* start TIMER channel input capture and channel dma request */
int32_t hal_timer_start_inputcapture_dma(hal_timer_dev_struct *timer_dev, uint32_t channel, uint32_t *buffer, uint16_t blength);
/* stop TIMER channel input capture and channel dma request */
int32_t hal_timer_stop_inputcapture_dma(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* read TIMER channel capture compare register value */
uint32_t hal_timer_read_capture_value(hal_timer_dev_struct *timer_dev, uint16_t channel);

/* single pulse mode */
/* initialize TIMER single pulse mode */
int32_t hal_timer_singlepulse_mode_init(hal_timer_dev_struct *timer_dev, uint32_t timer_singlepulse);
/* TIMER single pulse mode configure */
int32_t hal_timer_singlepulse_channel_config(hal_timer_dev_struct *timer_dev,hal_timer_singlepulse_struct *singlepulse,uint32_t channel_out,uint32_t channel_in);
/* start TIMER single pulse mode */
int32_t hal_timer_start_singlepulse(hal_timer_dev_struct *timer_dev, uint32_t out_channel);
/* stop TIMER single pulse mode */
int32_t hal_timer_stop_singlepulse(hal_timer_dev_struct *timer_dev, uint32_t out_channel);
/* start TIMER single pulse mode and channel interrupt */
int32_t hal_timer_start_singlepulse_interrupt(hal_timer_dev_struct *timer_dev, uint32_t out_channel);
/* stop TIMER single pulse mode and channel interrupt */
int32_t hal_timer_stop_singlepulse_interrupt(hal_timer_dev_struct *timer_dev, uint32_t out_channel);
/* start TIMER complementay channel single pulse mode */
int32_t hal_timer_start_negtive_singlepulse(hal_timer_dev_struct *timer_dev, uint16_t out_channel);
/* stop TIMER complementay channel single pulse mode */
int32_t hal_timer_stop_negtive_singlepulse(hal_timer_dev_struct *timer_dev, uint16_t out_channel);
/* start TIMER complementay channel single pulse mode and channel interrupt */
int32_t hal_timer_start_negtive_singlepulse_interrupt(hal_timer_dev_struct *timer_dev, uint16_t out_channel);
/* stop TIMER complementay channel single pulse mode and channel interrupt */
int32_t hal_timer_stop_negtive_singlepulse_interrupt(hal_timer_dev_struct *timer_dev, uint16_t out_channel);

/* quadrature decoder mode settings */
/* initialize TIMER quadrature decoder  mode */
int32_t hal_timer_quadrature_decoder_init(hal_timer_dev_struct *timer_dev, hal_timer_quadrature_decoder_struct *decode);
/* start TIMER decode mode */
int32_t hal_timer_start_quadrature_decoder(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* stop TIMER decode mode */
int32_t hal_timer_stop_quadrature_decoder(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* start TIMER decode mode and channel interrupt */
int32_t hal_timer_start_quadrature_decoder_interrupt(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* stop TIMER decode mode and channel interrupt */
int32_t hal_timer_stop_quadrature_decoder_interrupt(hal_timer_dev_struct *timer_dev, uint32_t channel);
/* start TIMER decode mode and channel dma request */
int32_t hal_timer_start_quadrature_decoder_dma(hal_timer_dev_struct *timer_dev, uint32_t channel, uint32_t *buffer0, uint32_t *buffer1, uint16_t blength);
/* stop TIMER decode mode and channel dma request */
int32_t hal_timer_stop_quadrature_decoder_dma(hal_timer_dev_struct *timer_dev, uint32_t channel);

/* DMA mode settings*/
/* start TIMER DMA mode for writing data to TIMER */
int32_t hal_timer_start_dmatransfer_write(hal_timer_dev_struct *timer_dev, hal_timer_dma_start_address_enum dma_startaddr, uint32_t dma_count,\
                            hal_timer_dma_request_source_enum dma_reqsrc, uint32_t *srcbuffer);
/* stop TIMER DMA mode for writing data to TIMER */
int32_t hal_timer_stop_dmatransfer_write(hal_timer_dev_struct *timer_dev, hal_timer_dma_request_source_enum dma_reqsrc);
/* start TIMER DMA mode for read data from TIMER */
int32_t hal_timer_start_dmatransfer_read(hal_timer_dev_struct *timer_dev, hal_timer_dma_start_address_enum dma_startaddr, uint32_t dma_count,\
                            hal_timer_dma_request_source_enum dma_reqsrc, uint32_t *srcbuffer);
/* stop TIMER DMA mode for read data from TIMER */
int32_t hal_timer_stop_dmatransfer_read(hal_timer_dev_struct *timer_dev, hal_timer_dma_request_source_enum dma_reqsrc);

/* HALL sensor interface settings */
/* initialize TIMER HALL mode */
int32_t hal_timer_hall_init(hal_timer_dev_struct *timer_dev,hal_timer_hall_struct *hall_init);
/* start TIMER HALL mode */
int32_t hal_timer_start_hall(hal_timer_dev_struct *timer_dev);
/* stop TIMER HALL mode */
int32_t hal_timer_stop_hall(hal_timer_dev_struct *timer_dev);
/* start TIMER HALL mode and channel interrupt */
int32_t hal_timer_start_hall_interrupt(hal_timer_dev_struct *timer_dev);
/* stop TIMER HALL mode and channel interrupt */
int32_t hal_timer_stop_hall_interrupt(hal_timer_dev_struct *timer_dev);
/* start TIMER HALL mode and channel dma request */
int32_t hal_timer_start_hall_dma(hal_timer_dev_struct *timer_dev, uint32_t *buffer, uint16_t blength);
/* stop TIMER HALL mode and channel dma request */
int32_t hal_timer_stop_hall_dma(hal_timer_dev_struct *timer_dev);

/* commutation control settings */
/* config TIMER commutation_event */
int32_t hal_timer_commutation_event_config(hal_timer_dev_struct *timer_dev,uint32_t trigger_source, uint32_t com_source);
/* config TIMER commutation_event and enable CMT interrupt */
int32_t hal_timer_commutation_event_interrupt_config(hal_timer_dev_struct *timer_dev,uint32_t trigger_source, uint32_t com_source);
/* config TIMER commutation_event and enable CMT DMA request */
int32_t hal_timer_commutation_event_dma_config(hal_timer_dev_struct *timer_dev,uint32_t trigger_source, uint32_t com_source);

/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
int32_t hal_timer_irq_handle_set(hal_timer_dev_struct *timer_dev, hal_timer_irq_struct *user_func);
/* reset all user-defined interrupt callback function */
int32_t hal_timer_irq_handle_all_reset(hal_timer_dev_struct *timer_dev);
/* interupt handle*/
void hal_timer_irq(hal_timer_dev_struct *timer_dev);

/* commonly used register settings */
/* write the current counter value, TIMERx_CNT */
int32_t hal_timer_write_counter_value(hal_timer_dev_struct *timer_dev, uint32_t counter_value);
/* read the current counter value, TIMERx_CNT */
uint32_t hal_timer_read_counter_value(hal_timer_dev_struct *timer_dev);
/* write counter auto reload value, TIMERx_CAR */
int32_t hal_timer_write_autoreload_value(hal_timer_dev_struct *timer_dev, uint32_t autoreload);
/* read counter auto reload value, TIMERx_CAR */
uint32_t hal_timer_read_autoreload_value(hal_timer_dev_struct *timer_dev);
/* write counter repetition value, TIMERx_CREP */
int32_t hal_timer_write_repetition_value(hal_timer_dev_struct *timer_dev, uint32_t repetition_value);
/* read counter repetition value, TIMERx_CREP */
uint32_t hal_timer_read_repetition_value(hal_timer_dev_struct *timer_dev);
/* write prescaler value of the counter clock, TIMERx_PSC */
int32_t hal_timer_write_prescaler_value(hal_timer_dev_struct *timer_dev, uint32_t prescaler);
/* read prescaler value of the counter clock, TIMERx_PSC */
uint32_t hal_timer_read_prescaler_value(hal_timer_dev_struct *timer_dev);
/* write channely capture/compare value, TIMERx_CHyCV */
int32_t hal_timer_write_channelx_value(hal_timer_dev_struct *timer_dev, uint32_t channel, uint32_t value);
/* read channely capture/compare value, TIMERx_CHyCV */
uint32_t hal_timer_read_channelx_value(hal_timer_dev_struct *timer_dev, uint32_t channel);

#endif /* GD32E23X_HAL_TIMER_H */
