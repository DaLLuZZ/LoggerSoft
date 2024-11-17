/*!
    \file    gd32e23x_hal_timer.c
    \brief   TIMER driver
    
    \version 2019-03-14, V1.0.0, firmware for GD32E10x
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

#define UINT16_T_MAX 65535
#define UINT8_T_MAX 255

/* DMA callback function */
/* DMA transmission complete(TC) callback for TIMER update dma request */
static void _update_dmatc_callback(void *dma);
/* DMA error callback for TIMER all dma request */
static void _timer_dmaerror_callback(void *dma);
/* DMA transmission complete(TC) callback for TIMER channel input capture dma request */
static void _channel_capture_dmatc_callback(void *dma);
/* DMA transmission complete(TC) callback for TIMER compare and pwm dma request */
static void _channel_compare_pwm_dmatc_callback(void *dma);
/* DMA transmission complete(TC) callback for TIMER commutation dma request */
static void _commutation_dmatc_callback(void *dma);
/* DMA transmission complete(TC) callback for TIMER trigger dma request */
static void _trigger_dmatc_callback(void *dma);

/* disable TIMER */
static int32_t _timer_disable(hal_timer_dev_struct *timer_dev);
/* enable or disable TIMER primary output */
static void _timer_primary_output_config(hal_timer_dev_struct *timer_dev , ControlStatus state);

/*!
    \brief      initialize the TIMER structure with the default values
    \param[in]  hal_struct_type: refer to hal_timer_struct_type_enum
    \param[in]  p_struct: point to TIMER structure that contains the configuration information
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_struct_init(hal_timer_struct_type_enum struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    switch(struct_type){
        case HAL_TIMER_BASIC_STRUCT:
            /* initialize TIMER basic config struct with the default values */
            ((hal_timer_basic_struct*)p_struct)->prescaler         = 0U;
            ((hal_timer_basic_struct*)p_struct)->alignedmode       = TIMER_COUNTER_EDGE;
            ((hal_timer_basic_struct*)p_struct)->counterdirection  = TIMER_COUNTER_UP;
            ((hal_timer_basic_struct*)p_struct)->period            = 65535U;
            ((hal_timer_basic_struct*)p_struct)->clockdivision     = TIMER_CKDIV_DIV1;
            ((hal_timer_basic_struct*)p_struct)->repetitioncounter = 0U;
            ((hal_timer_basic_struct*)p_struct)->autoreload_shadow = AUTO_RELOAD_SHADOW_DISABLE;
            ((hal_timer_basic_struct*)p_struct)->trgo_selection = TIMRE_TRGO_SRC_RESET;
            ((hal_timer_basic_struct*)p_struct)->master_slave_mode = TIMER_MASTER_SLAVE_MODE_DISABLE;
            break;
        case HAL_TIMER_CLOCKSOURCE_STRUCT:
            /* initialize TIMER clock source struct with the default values */
            ((hal_timer_clocksource_struct*)p_struct)->clock_source         = TIMER_CLOCK_SOURCE_CK_TIMER;
            ((hal_timer_clocksource_struct*)p_struct)->clock_polarity       = TIMER_CLOCK_TRIGGER_POLARITY_ETI_RISING;
            ((hal_timer_clocksource_struct*)p_struct)->clock_prescaler  = TIMER_CLOCK_PRESCALER_DIV1;
            ((hal_timer_clocksource_struct*)p_struct)->clock_filter            = 0U;
            break;
        case HAL_TIMER_SLAVEMODE_STRUCT:
            /* initialize TIMER slave mode struct with the default values */
            ((hal_timer_slavemode_struct*)p_struct)->slavemode         = TIMER_SLAVE_DISABLE_MODE;
            ((hal_timer_slavemode_struct*)p_struct)->trigger_selection       = TIMER_TRIGGER_SOURCE_ITI0;
            ((hal_timer_slavemode_struct*)p_struct)->trigger_polarity  = TIMER_CLOCK_TRIGGER_POLARITY_RISING;
            ((hal_timer_slavemode_struct*)p_struct)->trigger_prescaler            = TIMER_EXT_TRI_PSC_OFF;
            ((hal_timer_slavemode_struct*)p_struct)->trigger_filter            = 0U;
            break;
        case HAL_TIMER_INPUTCAPTURE_STRUCT:
            /* initialize TIMER input capture struct with the default values */
            ((hal_timer_inputcapture_struct*)p_struct)->icfilter         = 0U;
            ((hal_timer_inputcapture_struct*)p_struct)->icpolarity      = TIMER_IC_POLARITY_RISING;
            ((hal_timer_inputcapture_struct*)p_struct)->icprescaler  = TIMER_IC_PSC_DIV1;
            ((hal_timer_inputcapture_struct*)p_struct)->icselection            = TIMER_IC_SELECTION_DIRECTTI;
            break;
        case HAL_TIMER_QUADRATURE_DECODER_STRUCT:
            /* initialize TIMER quadrature decoder struct with the default values */
            ((hal_timer_quadrature_decoder_struct*)p_struct)->decodemode = TIMER_QUADRATURE_DECODER_MODE0;
            ((hal_timer_quadrature_decoder_struct*)p_struct)->ci0_filter = 0;
            ((hal_timer_quadrature_decoder_struct*)p_struct)->ci0_polarity = TIMER_IC_POLARITY_RISING;
            ((hal_timer_quadrature_decoder_struct*)p_struct)->ci0_prescaler = TIMER_IC_PSC_DIV1;
            ((hal_timer_quadrature_decoder_struct*)p_struct)->ci0_selection = TIMER_IC_SELECTION_DIRECTTI;
            ((hal_timer_quadrature_decoder_struct*)p_struct)->ci1_filter = 0;
            ((hal_timer_quadrature_decoder_struct*)p_struct)->ci1_polarity = TIMER_IC_POLARITY_RISING;
            ((hal_timer_quadrature_decoder_struct*)p_struct)->ci1_prescaler = TIMER_IC_PSC_DIV1;
            ((hal_timer_quadrature_decoder_struct*)p_struct)->ci1_selection = TIMER_IC_SELECTION_DIRECTTI;
            break;
        case HAL_TIMER_SINGLEPULSE_STRUCT:
            /* initialize TIMER single pulse struct with the default values */
            ((hal_timer_singlepulse_struct*)p_struct)->sp_ci_filter = 0;
            ((hal_timer_singlepulse_struct*)p_struct)->sp_ci_polarity = TIMER_IC_POLARITY_RISING;
            ((hal_timer_singlepulse_struct*)p_struct)->sp_ci_selection = TIMER_IC_SELECTION_DIRECTTI;
            ((hal_timer_singlepulse_struct*)p_struct)->sp_ocidlestate = TIMER_OC_IDLE_STATE_LOW;
            ((hal_timer_singlepulse_struct*)p_struct)->sp_ocmode = TIMER_OUTPUT_TIMING_MODE;
            ((hal_timer_singlepulse_struct*)p_struct)->sp_ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
            ((hal_timer_singlepulse_struct*)p_struct)->sp_ocnpolarity = TIMER_OCN_POLARITY_HIGH;
            ((hal_timer_singlepulse_struct*)p_struct)->sp_ocpolarity = TIMER_OC_POLARITY_HIGH;
            ((hal_timer_singlepulse_struct*)p_struct)->sp_pulse = 0;
            break;
        case HAL_TIMER_OUTPUTCOMPARE_STRUCT:
            /* initialize TIMER output compare struct with the default values */
            ((hal_timer_outputcompare_struct*)p_struct)->output_fastmode = TIMER_OC_FAST_DISABLE;
            ((hal_timer_outputcompare_struct*)p_struct)->output_idlestate = TIMER_OC_IDLE_STATE_LOW;
            ((hal_timer_outputcompare_struct*)p_struct)->output_mode = TIMER_OUTPUT_TIMING_MODE;
            ((hal_timer_outputcompare_struct*)p_struct)->outputn_idlestate = TIMER_OCN_IDLE_STATE_LOW;
            ((hal_timer_outputcompare_struct*)p_struct)->outputn_polarity =TIMER_OCN_POLARITY_HIGH ;
            ((hal_timer_outputcompare_struct*)p_struct)->output_polarity = TIMER_OC_POLARITY_HIGH;
            ((hal_timer_outputcompare_struct*)p_struct)->output_pulse = 0;
            break;
        case HAL_TIMER_OCPRE_CLEAR_STRUCT:
            /* initialize TIMER OxCPRE clear struct with the default values */
            ((hal_timer_ocpre_clear_struct*)p_struct)->ocpre_clear_source = TIMER_OCPRE_CLEAR_SOURCE_DISABLE;
            ((hal_timer_ocpre_clear_struct*)p_struct)->ocpre_clear_state = DISABLE;
            ((hal_timer_ocpre_clear_struct*)p_struct)->trigger_filter = 0;
            ((hal_timer_ocpre_clear_struct*)p_struct)->trigger_polarity = TIMER_ETP_RISING;
            ((hal_timer_ocpre_clear_struct*)p_struct)->trigger_prescaler = TIMER_EXT_TRI_PSC_OFF;
            break;
        case HAL_TIMER_HALL_STRUCT:
            /* initialize TIMER HALL struct with the default values */
            ((hal_timer_hall_struct*)p_struct)->cmt_delay = 0;
            ((hal_timer_hall_struct*)p_struct)->hall_filter = 0;
            ((hal_timer_hall_struct*)p_struct)->hall_polarity = TIMER_IC_POLARITY_RISING;
            ((hal_timer_hall_struct*)p_struct)->hall_prescaler = TIMER_IC_PSC_DIV1;
            break;
        case HAL_TIMER_BREAKCONFIG_STRUCT:
            /* initialize TIMER break config struct with the default values */
            ((hal_timer_breakconfig_struct*)p_struct)->breakpolarity = TIMER_BREAK_POLARITY_LOW;
            ((hal_timer_breakconfig_struct*)p_struct)->breakstate = TIMER_BREAK_DISABLE;
            ((hal_timer_breakconfig_struct*)p_struct)->deadtime = 0;
            ((hal_timer_breakconfig_struct*)p_struct)->ideloffstate = TIMER_IOS_STATE_DISABLE;
            ((hal_timer_breakconfig_struct*)p_struct)->outputautostate = TIMER_OUTAUTO_DISABLE;
            ((hal_timer_breakconfig_struct*)p_struct)->protectmode = TIMER_CCHP_PROT_OFF;
            ((hal_timer_breakconfig_struct*)p_struct)->runoffstate = TIMER_ROS_STATE_DISABLE;
            break;
        case HAL_TIMER_IRQ_STRUCT:
            ((hal_timer_irq_struct*)p_struct)->break_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->channelx_capture_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->channelx_compare_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->channelx_pwm_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->commutation_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->dmaerror_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->dmatc_chx_capture_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->dmatc_chx_cmppwm_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->dmatc_commutation_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->dmatc_trigger_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->dmatc_update_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->trigger_usercb = NULL;
            ((hal_timer_irq_struct*)p_struct)->update_usercb = NULL;
            break; 
        case HAL_TIMER_DEV_STRUCT:
            /* initialize TIMER device information struct with the default values */
            ((hal_timer_dev_struct*)p_struct)->hdma[0] = NULL;
            ((hal_timer_dev_struct*)p_struct)->hdma[1] = NULL;
            ((hal_timer_dev_struct*)p_struct)->hdma[2] = NULL;
            ((hal_timer_dev_struct*)p_struct)->hdma[3] = NULL;
            ((hal_timer_dev_struct*)p_struct)->hdma[4] = NULL;
            ((hal_timer_dev_struct*)p_struct)->hdma[5] = NULL;
            ((hal_timer_dev_struct*)p_struct)->hdma[6] = NULL;
            ((hal_timer_dev_struct*)p_struct)->periph = 0;
            ((hal_timer_dev_struct*)p_struct)->service_channel = TIMER_SERVICE_CHANNEL_NONE;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.break_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.channelx_capture_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.channelx_compare_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.channelx_pwm_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.commutation_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.trigger_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.update_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.dmaerror_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.dmatc_chx_capture_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.dmatc_chx_cmppwm_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.dmatc_commutation_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.dmatc_trigger_usercb = NULL;
            ((hal_timer_dev_struct*)p_struct)->timer_irq.dmatc_update_usercb = NULL;
            break;
        default:
            HAL_DEBUGW("parameter [struct_type] value is undefine");
            break;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize TIMER
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_deinit(hal_timer_dev_struct *timer_dev)
{
    uint32_t periph;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    periph = timer_dev->periph;
    /* deinit a TIMER */
    timer_deinit(timer_dev->periph);
    /* initialize the TIMER structure with the default values */
    hal_timer_struct_init(HAL_TIMER_DEV_STRUCT,timer_dev);
    timer_dev->periph = periph;
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize TIMER timebase
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_periph: specify which TIMER timebase is initialized 
      \arg        TIMERx(x=0,2,5,13..16)
    \param[in]  timer_basic: TIMER basic structure pointer
                  prescaler: prescaler value of the counter clock,0~65535
                  alignedmode: TIMER_COUNTER_EDGE, TIMER_COUNTER_CENTER_DOWN, TIMER_COUNTER_CENTER_UP, TIMER_COUNTER_CENTER_BOTH
                  counterdirection: TIMER_COUNTER_UP, TIMER_COUNTER_DOWN
                  period: counter auto reload value,0~65535
                  clockdivision: TIMER_CKDIV_DIV1, TIMER_CKDIV_DIV2, TIMER_CKDIV_DIV4
                  repetitioncounter: counter repetition value,0~255
                  autoreload_shadow:AUTO_RELOAD_SHADOW_ENABLE, AUTO_RELOAD_SHADOW_DISABLE
                  trgo_selection:
                    the argument could be selected from enumeration <hal_timer_trgo_selection_enum>
                  master_slave_mode:TIMER_MASTER_SLAVE_MODE_ENABLE, TIMER_MASTER_SLAVE_MODE_DISABLE
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_basic_init(hal_timer_dev_struct *timer_dev, uint32_t timer_periph, hal_timer_basic_struct *timer_basic)
{
    timer_parameter_struct timerbase;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == timer_basic){
        HAL_DEBUGE("pointer [timer_basic] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if((TIMER0 != timer_periph) && (TIMER2 != timer_periph) && (TIMER5 != timer_periph)\
        && (TIMER13 != timer_periph) && (TIMER14 != timer_periph) && (TIMER15 != timer_periph)\
        &&(TIMER16 != timer_periph))
    {
        HAL_DEBUGE("pointer [timer_periph] address is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* struct assignment */
    timer_dev->periph = timer_periph;
    timerbase.alignedmode = timer_basic->alignedmode;
    timerbase.clockdivision = timer_basic->clockdivision;
    timerbase.counterdirection = timer_basic->counterdirection;
    timerbase.period = timer_basic->period;
    timerbase.prescaler = timer_basic->prescaler;
    timerbase.repetitioncounter = timer_basic->repetitioncounter;
    /* initialize TIMER basic settings */
    timer_init(timer_dev->periph,(&timerbase));
    /* setting the shadow register for TIMERx_CAR register */
    TIMER_CTL0(timer_dev->periph) &= TIMER_CTL0_ARSE;
    TIMER_CTL0(timer_dev->periph) |= timer_basic->autoreload_shadow;
    /* configure TIMER master slave mode */
    timer_master_slave_mode_config(timer_dev->periph, timer_basic->master_slave_mode);
    /* select TIMER master mode output trigger source */
    timer_master_output_trigger_source_select(timer_dev->periph, timer_basic->trgo_selection);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER counter enable
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_counter(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER counter disable
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_counter(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* disable a TIMER */
    timer_disable(timer_dev->periph);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER counter and update interrupt enable
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_counter_interrupt(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* enable the TIMER interrupt */
    timer_interrupt_enable(timer_dev->periph, TIMER_INT_UP);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER counter and update interrupt stop
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_counter_interrupt(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* disable the TIMER interrupt */
    timer_interrupt_disable(timer_dev->periph, TIMER_INT_UP);
    /* disable a TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      TIMER counter and update DMA request enable 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  buffer: the source memory buffer address
    \param[in]  blength: the number of data to be transferred from source to destination,0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_counter_dma(hal_timer_dev_struct *timer_dev, uint32_t *buffer, uint16_t blength)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == buffer){
        HAL_DEBUGE("pointer [buffer] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* DMA channel full transfer finish interrupt handler pointer */
    dma_irq.full_finish_handle = _update_dmatc_callback;
    /* DMA channel error interrupt handler pointer */
    dma_irq.error_handle = _timer_dmaerror_callback;
    /* DMA channel half transfer finish interrupt handler pointer */
    dma_irq.half_finish_handle = NULL;
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_UP], (uint32_t)buffer, TIMER_CAR_ADDRESS(timer_dev->periph), blength, &dma_irq);
    /* enable the TIMER DMA update request */
    timer_dma_enable(timer_dev->periph, TIMER_DMA_UPD);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER counter and update DMA request disable 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_counter_dma(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* disable the TIMER DMA update request */
    timer_dma_disable(timer_dev->periph, TIMER_DMA_UPD);
    /* stop DMA transfer */
    hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_UP]);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    
    return ret_val;
}

/*!
    \brief      TIMER slave mode config 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_slavemode: TIMER slave mode config struct
                  slavemode:
                    the argument could be selected from enumeration <hal_timer_slave_mode_enum>
                  trigger_selection:
                    the argument could be selected from enumeration <hal_timer_input_trigger_source_enum> 
                  trigger_polarity:
                    the argument could be selected from enumeration <hal_timer_clock_trigger_polarity_enum> 
                  trigger_prescaler:
                    only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_PSC_OFF: no divided
      \arg          TIMER_EXT_TRI_PSC_DIV2: divided by 2
      \arg          TIMER_EXT_TRI_PSC_DIV4: divided by 4
      \arg          TIMER_EXT_TRI_PSC_DIV8: divided by 8
                  trigger_filter: 0x0~0xF
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_slave_mode_config(hal_timer_dev_struct *timer_dev, hal_timer_slavemode_struct *timer_slavemode)
{
    uint32_t chctl2;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == timer_slavemode){
        HAL_DEBUGE("pointer [timer_slavemode] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* select TIMER input trigger source  */
    timer_input_trigger_source_select(timer_dev->periph, timer_slavemode->trigger_selection);
    /* select TIMER slave mode */
    timer_slave_mode_select(timer_dev->periph, timer_slavemode->slavemode);
    switch(timer_slavemode->trigger_selection){
        case TIMER_TRIGGER_SOURCE_ITI0:
            /* no need to config polarity, prescaler, filter */
            break;
        case TIMER_TRIGGER_SOURCE_ITI1:
            /* no need to config polarity, prescaler, filter */
            break;
        case TIMER_TRIGGER_SOURCE_ITI2:
            /* no need to config polarity, prescaler, filter */
            break;
        case TIMER_TRIGGER_SOURCE_ITI3:
            /* no need to config polarity, prescaler, filter */
            break;
        case TIMER_TRIGGER_SOURCE_CI0FE0:
            chctl2 = TIMER_CHCTL2(timer_dev->periph);
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            /* reset the CH0P and CH0NP bits */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
            /* config polarity */
            TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)(timer_slavemode->trigger_polarity);
            /* reset the CH0CAPFLT bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
            /* config filter */
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 4U);
            TIMER_CHCTL2(timer_dev->periph) = chctl2;
            break;
        case TIMER_TRIGGER_SOURCE_CI1FE1:
            chctl2 = TIMER_CHCTL2(timer_dev->periph);
            /* reset the CH1EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
            /* reset the CH1P and CH1NP bits */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
            /* config polarity */
            TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_polarity) << 4U);
            /* reset the CH1CAPFLT bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
            /* config filter */
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 12U);
            TIMER_CHCTL2(timer_dev->periph) = chctl2;
            break;
        case TIMER_TRIGGER_SOURCE_CI0FED:
            chctl2 = TIMER_CHCTL2(timer_dev->periph);
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            /* reset the CH0CAPFLT bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
            /* config filter */
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 4U);
            TIMER_CHCTL2(timer_dev->periph) = chctl2;
            break;
        case TIMER_TRIGGER_SOURCE_ETIFP:
            /* external clock mode 1 disabled */
            TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC1;
            /* configure TIMER external trigger input */
            timer_external_trigger_config(timer_dev->periph, timer_slavemode->trigger_prescaler, 
            timer_slavemode->trigger_polarity, timer_slavemode->trigger_filter);
            break;
        default:
            HAL_DEBUGW("parameter [timer_slavemode->trigger_selection] value is undefine");
            break;
    }
    /* disable the TIMER interrupt */
    timer_interrupt_disable(timer_dev->periph, TIMER_INT_TRG);
    /* disable the TIMER DMA trigger request */
    timer_dma_disable(timer_dev->periph, TIMER_DMA_TRGD);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER slave mode congfig and interrupt congfig 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_slavemode: TIMER slave mode config struct
                  slavemode:
                    the argument could be selected from enumeration <hal_timer_slave_mode_enum>
                  trigger_selection:
                    the argument could be selected from enumeration <hal_timer_input_trigger_source_enum> 
                  trigger_polarity:
                    the argument could be selected from enumeration <hal_timer_clock_trigger_polarity_enum> 
                  trigger_prescaler:
                    only one parameter can be selected which is shown as below:
      \arg          TIMER_EXT_TRI_PSC_OFF: no divided
      \arg          TIMER_EXT_TRI_PSC_DIV2: divided by 2
      \arg          TIMER_EXT_TRI_PSC_DIV4: divided by 4
      \arg          TIMER_EXT_TRI_PSC_DIV8: divided by 8
                  trigger_filter: 0x0~0xF
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_slave_mode_interrupt_config(hal_timer_dev_struct *timer_dev, hal_timer_slavemode_struct *timer_slavemode)
{
    uint32_t chctl2;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == timer_slavemode){
        HAL_DEBUGE("pointer [timer_slavemode] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* select TIMER input trigger source  */
    timer_input_trigger_source_select(timer_dev->periph, timer_slavemode->trigger_selection);
    /* select TIMER slave mode */
    timer_slave_mode_select(timer_dev->periph, timer_slavemode->slavemode);
    switch(timer_slavemode->trigger_selection){
        case TIMER_TRIGGER_SOURCE_ITI0:
            /* no need to config polarity, prescaler, filter */
            break;
        case TIMER_TRIGGER_SOURCE_ITI1:
            /* no need to config polarity, prescaler, filter */
            break;
        case TIMER_TRIGGER_SOURCE_ITI2:
            /* do not config polarity, prescaler, filter */
            break;
        case TIMER_TRIGGER_SOURCE_ITI3:
            /* no need to config polarity, prescaler, filter */
            break;
        case TIMER_TRIGGER_SOURCE_CI0FE0:
            chctl2 = TIMER_CHCTL2(timer_dev->periph);
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            /* reset the CH0P and CH0NP bits */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
            /* config polarity */
            TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)(timer_slavemode->trigger_polarity);
            /* reset the CH0CAPFLT bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
            /* config filter */
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 4U);
            TIMER_CHCTL2(timer_dev->periph) = chctl2;
            break;
        case TIMER_TRIGGER_SOURCE_CI1FE1:
            chctl2 = TIMER_CHCTL2(timer_dev->periph);
            /* reset the CH1EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
            /* reset the CH1P and CH1NP bits */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
            /* config polarity */
            TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_polarity) << 4U);
            /* reset the CH1CAPFLT bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
            /* config filter */
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 12U);
            TIMER_CHCTL2(timer_dev->periph) = chctl2;
            break;
        case TIMER_TRIGGER_SOURCE_CI0FED:
            chctl2 = TIMER_CHCTL2(timer_dev->periph);
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            /* reset the CH0CAPFLT bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
            /* config filter */
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_slavemode->trigger_filter) << 4U);
            TIMER_CHCTL2(timer_dev->periph) = chctl2;
            break;
        case TIMER_TRIGGER_SOURCE_ETIFP:
            /* external clock mode 1 disabled */
            TIMER_SMCFG(timer_dev->periph) &= ~(uint32_t)TIMER_SMCFG_SMC1;
            /* configure TIMER external trigger input */
            timer_external_trigger_config(timer_dev->periph, timer_slavemode->trigger_prescaler, 
            timer_slavemode->trigger_polarity, timer_slavemode->trigger_filter);
            break;
        default:
            HAL_DEBUGW("parameter [timer_slavemode->trigger_selection] value is undefine");
            break;
    }
    /* enable the TIMER interrupt */
    timer_interrupt_enable(timer_dev->periph, TIMER_INT_TRG);
    /* disable the TIMER DMA trigger request */
    timer_dma_disable(timer_dev->periph, TIMER_DMA_TRGD);
    return HAL_ERR_NONE;
}
/*!
    \brief      TIMER clock source config 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_clock: TIMER clock source config struct
                  clock_source:
                    the argument could be selected from enumeration <hal_timer_clock_source_enum>
                  clock_polarity:
                    the argument could be selected from enumeration <hal_timer_clock_trigger_polarity_enum>
                  clock_prescaler:
                    only one parameter can be selected which is shown as below:
      \arg          TIMER_CLOCK_PRESCALER_DIV1: clock input source no divided for ETI
      \arg          TIMER_CLOCK_PRESCALER_DIV2: clock input source divided by 2 for ETI
      \arg          TIMER_CLOCK_PRESCALER_DIV4: clock input source divided by 4 for ETI
      \arg          TIMER_CLOCK_PRESCALER_DIV8: clock input source divided by 8 for ETI
                  clock_filter: 0x0~0xF
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_clock_source_config(hal_timer_dev_struct *timer_dev, hal_timer_clocksource_struct *timer_clock)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == timer_clock){
        HAL_DEBUGE("pointer [timer_clock] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    switch(timer_clock->clock_source){
        case TIMER_CLOCK_SOURCE_CK_TIMER:
            /* configure TIMER internal clock mode */
            timer_internal_clock_config(timer_dev->periph);
            break;
        case TIMER_CLOCK_SOURCE_ITI0:
            /* configure TIMER the internal trigger as external clock input */
            timer_internal_trigger_as_external_clock_config(timer_dev->periph,TIMER_SMCFG_TRGSEL_ITI0);
            break;
        case TIMER_CLOCK_SOURCE_ITI1:
            /* configure TIMER the internal trigger as external clock input */
            timer_internal_trigger_as_external_clock_config(timer_dev->periph,TIMER_SMCFG_TRGSEL_ITI1);
            break;
        case TIMER_CLOCK_SOURCE_ITI2:
            /* configure TIMER the internal trigger as external clock input */
            timer_internal_trigger_as_external_clock_config(timer_dev->periph,TIMER_SMCFG_TRGSEL_ITI2);
            break;
        case TIMER_CLOCK_SOURCE_ITI3:
            /* configure TIMER the internal trigger as external clock input */
            timer_internal_trigger_as_external_clock_config(timer_dev->periph,TIMER_SMCFG_TRGSEL_ITI3);
            break;
        case TIMER_CLOCK_SOURCE_CI0FE0:
            /* configure TIMER the external trigger as external clock input */
            timer_external_trigger_as_external_clock_config(timer_dev->periph,TIMER_SMCFG_TRGSEL_CI0FE0,
            timer_clock->clock_polarity,timer_clock->clock_filter);
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            break;
        case TIMER_CLOCK_SOURCE_CI1FE1:
            /* configure TIMER the external trigger as external clock input */
            timer_external_trigger_as_external_clock_config(timer_dev->periph,TIMER_SMCFG_TRGSEL_CI1FE1,
            timer_clock->clock_polarity,timer_clock->clock_filter);
            /* reset the CH1EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);
            break;
        case TIMER_CLOCK_SOURCE_CI0FED:
            /* configure TIMER the external trigger as external clock input */
            timer_external_trigger_as_external_clock_config(timer_dev->periph,TIMER_SMCFG_TRGSEL_CI0F_ED,
            timer_clock->clock_polarity,timer_clock->clock_filter);
            /* reset the CH0EN bit */ 
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);
            break;
        case TIMER_CLOCK_SOURCE_ETIMODE0:
            /* configure TIMER the external clock mode0 */
            timer_external_clock_mode0_config(timer_dev->periph,timer_clock ->clock_prescaler,
            timer_clock->clock_polarity,timer_clock->clock_filter);
            break;
        case TIMER_CLOCK_SOURCE_ETIMODE1:
            /* configure TIMER the external clock mode1 */
            timer_external_clock_mode1_config(timer_dev->periph,timer_clock ->clock_prescaler,
            timer_clock->clock_polarity,timer_clock->clock_filter);
            break;
        default:
            HAL_DEBUGW("parameter [timer_clock->clock_source] value is undefine");
            break;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER break config 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  break_cofig: TIMER break parameter struct
                    runoffstate: TIMER_ROS_STATE_ENABLE,TIMER_ROS_STATE_DISABLE
                    ideloffstate: TIMER_IOS_STATE_ENABLE,TIMER_IOS_STATE_DISABLE
                    deadtime: 0~255
                    breakpolarity: TIMER_BREAK_POLARITY_LOW,TIMER_BREAK_POLARITY_HIGH
                    outputautostate: TIMER_OUTAUTO_ENABLE,TIMER_OUTAUTO_DISABLE
                    protectmode: TIMER_CCHP_PROT_OFF,TIMER_CCHP_PROT_0,TIMER_CCHP_PROT_1,TIMER_CCHP_PROT_2
                    breakstate: TIMER_BREAK_ENABLE,TIMER_BREAK_DISABLE
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_break_config(hal_timer_dev_struct *timer_dev, hal_timer_breakconfig_struct *break_cofig)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == break_cofig){
        HAL_DEBUGE("pointer [break_cofig] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* configure TIMER break function */
    timer_break_config(timer_dev->periph, break_cofig);
    
    return HAL_ERR_NONE;
}



/*!
    \brief      TIMER OxCPRE signal clear config 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  oc_clear: TIMER output compare clear struct
                  ocpre_clear_source: TIMER_OCPRE_CLEAR_SOURCE_DISABLE,TIMER_OCPRE_CLEAR_SOURCE_CLR,TIMER_OCPRE_CLEAR_SOURCE_ETIF
                  ocpre_clear_state: ENABLE or DISABLE 
                  trigger_polarity: TIMER_ETP_FALLING,TIMER_ETP_RISING 
                  trigger_prescaler: TIMER_EXT_TRI_PSC_OFF,TIMER_EXT_TRI_PSC_DIV2,TIMER_EXT_TRI_PSC_DIV4,TIMER_EXT_TRI_PSC_DIV8
                  trigger_filter:0~15
    \param[in]  channel: TIMER_CH_0,TIMER_CH_1,TIMER_CH_2,TIMER_CH_3
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_timer_ocpre_clear_config(hal_timer_dev_struct *timer_dev,hal_timer_ocpre_clear_struct *oc_clear, uint32_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == oc_clear){
        HAL_DEBUGE("pointer [oc_clear] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if((TIMER_CH_0 != channel) && (TIMER_CH_1 != channel) && \
       (TIMER_CH_2 != channel) && (TIMER_CH_3 != channel) ){
        HAL_DEBUGE("parameter channel is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(oc_clear->ocpre_clear_source){
        case TIMER_OCPRE_CLEAR_SOURCE_DISABLE:
            /* disable configure TIMER OCPRE clear source */
            TIMER_SMCFG(timer_dev->periph) &= ~(TIMER_SMCFG_OCRC | TIMER_SMCFG_ETFC | TIMER_SMCFG_ETPSC | TIMER_SMCFG_SMC1 | TIMER_SMCFG_ETP);
            break;
        case TIMER_OCPRE_CLEAR_SOURCE_CLR:
            /* configure TIMER OCPRE clear source selection */
            timer_ocpre_clear_source_config(timer_dev->periph, TIMER_OCPRE_CLEAR_SOURCE_CLR) ;
            break; 
        case TIMER_OCPRE_CLEAR_SOURCE_ETIF:
            /* configure TIMER external trigger input */
            timer_external_trigger_config(timer_dev->periph,oc_clear->trigger_prescaler,oc_clear->trigger_polarity,oc_clear->trigger_filter);
            /* configure TIMER OCPRE clear source selection */
            timer_ocpre_clear_source_config(timer_dev->periph, TIMER_OCPRE_CLEAR_SOURCE_ETIF) ;
            break;
        default:
            HAL_DEBUGW("parameter [oc_clear->ocpre_clear_source] value is undefine");
            break;
    }
    
    /* configure TIMER channel output clear function */
    if(ENABLE == (oc_clear->ocpre_clear_state)){
        timer_channel_output_clear_config(timer_dev->periph,channel,TIMER_OC_CLEAR_ENABLE);
    }else{
        timer_channel_output_clear_config(timer_dev->periph,channel,TIMER_OC_CLEAR_DISABLE);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER CI0 trigger input selection 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  ci0_select:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CI0_CH0IN: the TIMERx_CH0 pin input is selected as channel 0 trigger input
      \arg        TIMER_CI0_XOR_CH012: the result of combinational XOR of TIMERx_CH0,CH1 and CH2 pins is selected as channel 0 trigger input
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_timer_ci0_input_selection(hal_timer_dev_struct *timer_dev, uint32_t ci0_select)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if((TIMER_CI0_CH0IN != ci0_select) && (TIMER_CI0_XOR_CH012 != ci0_select)){
        HAL_DEBUGE("parameter ci0_select is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER hall sensor mode */
    timer_hall_mode_config(timer_dev->periph,ci0_select);
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMERx remap 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  remap:
                only one parameter can be selected which is shown as below:
      \arg        TIMER13_CI0_RMP_GPIO: timer13 channel 0 input is connected to GPIO(TIMER13_CH0)
      \arg        TIMER13_CI0_RMP_RTCCLK: timer13 channel 0 input is connected to the RTCCLK
      \arg        TIMER13_CI0_RMP_HXTAL_DIV32: timer13 channel 0 input is connected to HXTAL/32 clock
      \arg        TIMER13_CI0_RMP_CKOUTSEL: timer13 channel 0 input is connected to CKOUTSEL
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_remap_config(hal_timer_dev_struct *timer_dev,uint32_t remap)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* configure TIMER channel remap function */
    if(TIMER13 == timer_dev->periph){
        timer_channel_remap_config(TIMER13,remap);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER generate a software event 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  event_src:
                    the argument could be selected from enumeration <hal_timer_software_event_enum>
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_generate_event(hal_timer_dev_struct *timer_dev, hal_timer_software_event_enum event_src)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* software generate events  */
    timer_event_software_generate(timer_dev->periph, event_src);
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize TIMER output compare mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  timer_compare: TIMER output compare mode struct
                  output_mode: 
                    the argument could be selected from enumeration <hal_timer_output_compare_enum>
                  output_pulse:0~65535
                  output_fastmode: TIMER_OC_FAST_ENABLE, TIMER_OC_FAST_DISABLE
                  output_polarity: TIMER_OC_POLARITY_HIGH, TIMER_OC_POLARITY_LOW
                  outputn_polarity: TIMER_OCN_POLARITY_HIGH, TIMER_OCN_POLARITY_LOW
                  output_idlestate: TIMER_OC_IDLE_STATE_LOW, TIMER_OC_IDLE_STATE_HIGH
                  outputn_idlestate: TIMER_OCN_IDLE_STATE_LOW, TIMER_OCN_IDLE_STATE_HIGH
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_outputcompare_init(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_outputcompare_struct *timer_compare)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == timer_compare){
        HAL_DEBUGE("pointer [timer_compare] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */


    switch(channel){
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        /* configure TIMER channel enable state */
        timer_channel_output_state_config(timer_dev->periph,TIMER_CH_0,TIMER_CCX_DISABLE);
        if((TIMER0 == timer_dev->periph) || (TIMER14 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)){
            /* configure TIMER channel complementary output enable state */
            timer_channel_complementary_output_state_config(timer_dev->periph,TIMER_CH_0,TIMER_CCXN_DISABLE);
            timer_channel_complementary_output_polarity_config(timer_dev->periph, TIMER_CH_0, timer_compare->outputn_polarity);
            /* reset the ISO0 bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO0);
            /* set the ISO0 bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)timer_compare->output_idlestate;
            /* reset the ISO0N bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO0N);
            /* set the ISO0N bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)timer_compare->outputn_idlestate;
        }
        TIMER_CHCTL0(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL0_CH0MS;
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        /* configure TIMER channel enable state */
        timer_channel_output_state_config(timer_dev->periph,TIMER_CH_1,TIMER_CCX_DISABLE);
        if((TIMER0 == timer_dev->periph) || (TIMER14 == timer_dev->periph)){
            if(TIMER0 == timer_dev->periph){
                /* configure TIMER channel complementary output enable state */
                timer_channel_complementary_output_state_config(timer_dev->periph,TIMER_CH_1,TIMER_CCXN_DISABLE);
                timer_channel_complementary_output_polarity_config(timer_dev->periph, TIMER_CH_1, timer_compare->outputn_polarity);
                /* reset the ISO1N bit */
                TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO1N);
                /* set the ISO1N bit */
                TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->outputn_idlestate) << 2U);
            }
            /* reset the ISO1 bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO1);
            /* set the ISO1 bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->output_idlestate) << 2U);

        }
        TIMER_CHCTL0(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL0_CH1MS;
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        /* configure TIMER channel enable state */
        timer_channel_output_state_config(timer_dev->periph,TIMER_CH_2,TIMER_CCX_DISABLE);
        if(TIMER0 == timer_dev->periph){
            /* configure TIMER channel complementary output enable state */
            timer_channel_complementary_output_state_config(timer_dev->periph,TIMER_CH_2,TIMER_CCXN_DISABLE);
            timer_channel_complementary_output_polarity_config(timer_dev->periph, TIMER_CH_2, timer_compare->outputn_polarity);

            /* reset the ISO2 bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO2);
            /* set the ISO2 bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->output_idlestate) << 4U);
            /* reset the ISO2N bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO2N);
            /* set the ISO2N bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->outputn_idlestate) << 4U);
        }
        TIMER_CHCTL1(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL1_CH2MS;
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        /* configure TIMER channel enable state */
        timer_channel_output_state_config(timer_dev->periph,TIMER_CH_3,TIMER_CCX_DISABLE);
        if(TIMER0 == timer_dev->periph){
            /* reset the ISO3 bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO3);
            /* set the ISO3 bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->output_idlestate) << 6U);
        }
        TIMER_CHCTL1(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL1_CH3MS;
        break;
    default:
        break;
    }

    /* configure TIMER channel output compare mode */
    timer_channel_output_mode_config(timer_dev->periph, channel,timer_compare->output_mode);
    /* configure TIMER channel output pulse value */
    timer_channel_output_pulse_value_config(timer_dev->periph, channel, timer_compare->output_pulse);
    /* configure TIMER channel output polarity  */
    timer_channel_output_polarity_config(timer_dev->periph, channel, timer_compare->output_polarity);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER output compare mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_outputcompare(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER output compare mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_outputcompare(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER output compare mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_outputcompare_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    switch (channel){
    case TIMER_CH_0:
        /* enable the TIMER compare 0 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* enable the TIMER compare 1 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* enable the TIMER compare 2 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        break;

    case TIMER_CH_3:
        /* enable the TIMER compare 3 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH3);
        break;

    default:
        break;
    }
    
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER output compare mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_outputcompare_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    switch (channel){
    case TIMER_CH_0:
        /* enable the TIMER compare 0 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* enable the TIMER compare 1 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* enable the TIMER compare 2 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        break;

    case TIMER_CH_3:
        /* enable the TIMER compare 3 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH3);
        break;

    default:
        break;
    }

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER output compare mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  buffer: the source memory buffer address
    \param[in]  blength: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_outputcompare_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, uint32_t *buffer, uint16_t blength)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
    dma_irq.half_finish_handle = NULL;
    dma_irq.error_handle = _timer_dmaerror_callback;
    
    switch (channel){
    case TIMER_CH_0:
        /* enable the DMA CH0 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], (uint32_t)buffer,TIMER_CH0CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;

    case TIMER_CH_1:
        /* enable the DMA CH1 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], (uint32_t)buffer,TIMER_CH1CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;

    case TIMER_CH_2:
        /* enable the DMA CH2 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH2], (uint32_t)buffer,TIMER_CH2CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;

    case TIMER_CH_3:
        /* enable the DMA CH3 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH3], (uint32_t)buffer,TIMER_CH3CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
        break;

    default:
        break;
    }

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER output compare mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_outputcompare_dma(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch (channel){
    case TIMER_CH_0:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
        break;

    case TIMER_CH_1:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
        break;

    case TIMER_CH_2:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH2]);
        break;

    case TIMER_CH_3:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH3]);
        break;

    default:
        break;
    }

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary output compare mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_outputcompare_negtive(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary output compare mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_outputcompare_negtive(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary output compare mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_outputcompare_negtive_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    switch (channel){
    case TIMER_CH_0:
        /* enable the TIMER compare 0 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* enable the TIMER compare 1 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* enable the TIMER compare 2 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        break;

    default:
        break;
    }
    
    /* enable the TIMER interrupt */
    timer_interrupt_enable(timer_dev->periph, TIMER_INT_BRK);
    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary output compare mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_outputcompare_negtive_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    switch (channel){
    case TIMER_CH_0:
        /* disable the TIMER compare 0 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* disable the TIMER compare 1 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* disable the TIMER compare 2 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        break;

    default:
        break;
    }

    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    if(RESET == (TIMER_CHCTL2(timer_dev->periph) & (TIMER_CHCTL2_CH0NEN | TIMER_CHCTL2_CH1NEN | TIMER_CHCTL2_CH2NEN))){
        /* disable the TIMER interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_BRK);
    }
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary output compare mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
    \param[in]  buffer: the source memory buffer address
    \param[in]  blength: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_outputcompare_negtive_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, uint32_t *buffer, uint16_t blength)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
    dma_irq.half_finish_handle = NULL;
    dma_irq.error_handle = _timer_dmaerror_callback;
    
    switch (channel){
    case TIMER_CH_0:
        /* enable the DMA CH0 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], (uint32_t)buffer,TIMER_CH0CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;

    case TIMER_CH_1:
        /* enable the DMA CH1 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], (uint32_t)buffer,TIMER_CH1CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;

    case TIMER_CH_2:
        /* enable the DMA CH2 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH2], (uint32_t)buffer,TIMER_CH2CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;

    default:
        break;
    }

    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary output compare mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_outputcompare_negtive_dma(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch (channel){
    case TIMER_CH_0:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
        break;

    case TIMER_CH_1:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
        break;

    case TIMER_CH_2:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH2]);
        break;

    default:
        break;
    }

    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      initialize TIMER pwm mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  timer_compare: TIMER output compare mode struct
                  output_mode: 
                    the argument could be selected from enumeration <hal_timer_output_compare_enum>
                  output_pulse:0~65535
                  output_fastmode: TIMER_OC_FAST_ENABLE, TIMER_OC_FAST_DISABLE
                  output_polarity: TIMER_OC_POLARITY_HIGH, TIMER_OC_POLARITY_LOW
                  outputn_polarity: TIMER_OCN_POLARITY_HIGH, TIMER_OCN_POLARITY_LOW
                  output_idlestate: TIMER_OC_IDLE_STATE_LOW, TIMER_OC_IDLE_STATE_HIGH
                  outputn_idlestate: TIMER_OCN_IDLE_STATE_LOW, TIMER_OCN_IDLE_STATE_HIGH
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwm_init(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_outputcompare_struct *timer_compare)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == timer_compare){
        HAL_DEBUGE("pointer [timer_compare] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */


    switch(channel){
    /* configure TIMER_CH_0 */
    case TIMER_CH_0:
        /* configure TIMER channel enable state */
        timer_channel_output_state_config(timer_dev->periph,TIMER_CH_0,TIMER_CCX_DISABLE);
        if((TIMER0 == timer_dev->periph) || (TIMER14 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)){
            /* configure TIMER channel complementary output enable state */
            timer_channel_complementary_output_state_config(timer_dev->periph,TIMER_CH_0,TIMER_CCXN_DISABLE);
            timer_channel_complementary_output_polarity_config(timer_dev->periph, TIMER_CH_0, timer_compare->outputn_polarity);
            /* reset the ISO0 bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO0);
            /* set the ISO0 bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)timer_compare->output_idlestate;
            /* reset the ISO0N bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO0N);
            /* set the ISO0N bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)timer_compare->outputn_idlestate;
        }
        TIMER_CHCTL0(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL0_CH0MS;
        TIMER_CHCTL0(timer_dev->periph) |= TIMER_CHCTL0_CH0COMSEN;
        timer_channel_output_fast_config(timer_dev->periph, channel, timer_compare->output_fastmode);
        break;
    /* configure TIMER_CH_1 */
    case TIMER_CH_1:
        /* configure TIMER channel enable state */
        timer_channel_output_state_config(timer_dev->periph,TIMER_CH_1,TIMER_CCX_DISABLE);
        if((TIMER0 == timer_dev->periph) || (TIMER14 == timer_dev->periph)){
            if(TIMER0 == timer_dev->periph){
                /* configure TIMER channel complementary output enable state */
                timer_channel_complementary_output_state_config(timer_dev->periph,TIMER_CH_1,TIMER_CCXN_DISABLE);
                timer_channel_complementary_output_polarity_config(timer_dev->periph, TIMER_CH_1, timer_compare->outputn_polarity);
                /* reset the ISO1N bit */
                TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO1N);
                /* set the ISO1N bit */
                TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->outputn_idlestate) << 2U);
            }
            /* reset the ISO1 bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO1);
            /* set the ISO1 bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->output_idlestate) << 2U);

        }
        TIMER_CHCTL0(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL0_CH1MS;
        TIMER_CHCTL0(timer_dev->periph) |= TIMER_CHCTL0_CH1COMSEN;
        timer_channel_output_fast_config(timer_dev->periph, channel, timer_compare->output_fastmode);
        break;
    /* configure TIMER_CH_2 */
    case TIMER_CH_2:
        /* configure TIMER channel enable state */
        timer_channel_output_state_config(timer_dev->periph,TIMER_CH_2,TIMER_CCX_DISABLE);
        if(TIMER0 == timer_dev->periph){
            /* configure TIMER channel complementary output enable state */
            timer_channel_complementary_output_state_config(timer_dev->periph,TIMER_CH_2,TIMER_CCXN_DISABLE);
            timer_channel_complementary_output_polarity_config(timer_dev->periph, TIMER_CH_2, timer_compare->outputn_polarity);

            /* reset the ISO2 bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO2);
            /* set the ISO2 bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->output_idlestate) << 4U);
            /* reset the ISO2N bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO2N);
            /* set the ISO2N bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->outputn_idlestate) << 4U);
        }
        TIMER_CHCTL1(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL1_CH2MS;
        TIMER_CHCTL1(timer_dev->periph) |= TIMER_CHCTL1_CH2COMSEN;
        timer_channel_output_fast_config(timer_dev->periph, channel, timer_compare->output_fastmode);
        break;
    /* configure TIMER_CH_3 */
    case TIMER_CH_3:
        /* configure TIMER channel enable state */
        timer_channel_output_state_config(timer_dev->periph,TIMER_CH_3,TIMER_CCX_DISABLE);
        if(TIMER0 == timer_dev->periph){
            /* reset the ISO3 bit */
            TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO3);
            /* set the ISO3 bit */
            TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(timer_compare->output_idlestate) << 6U);
        }
        TIMER_CHCTL1(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL1_CH3MS;
        TIMER_CHCTL1(timer_dev->periph) |= TIMER_CHCTL1_CH3COMSEN;
        timer_channel_output_fast_config(timer_dev->periph, channel, timer_compare->output_fastmode);
        break;
    default:
        break;
    }

    /* configure TIMER channel output compare mode */
    timer_channel_output_mode_config(timer_dev->periph, channel,timer_compare->output_mode);
    /* configure TIMER channel output pulse value */
    timer_channel_output_pulse_value_config(timer_dev->periph, channel, timer_compare->output_pulse);
    /* configure TIMER channel output polarity */
    timer_channel_output_polarity_config(timer_dev->periph, channel, timer_compare->output_polarity);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER pwm mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_pwm(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER pwm mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_pwm(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER pwm mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_pwm_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    switch (channel){
    case TIMER_CH_0:
        /* enable the TIMER compare 0 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* enable the TIMER compare 1 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* enable the TIMER compare 2 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        break;

    case TIMER_CH_3:
        /* enable the TIMER compare 3 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH3);
        break;

    default:
        break;
    }
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER pwm mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_pwm_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch (channel){
    case TIMER_CH_0:
        /* enable the TIMER compare 0 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* enable the TIMER compare 1 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* enable the TIMER compare 2 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        break;

    case TIMER_CH_3:
        /* enable the TIMER compare 3 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH3);
        break;

    default:
        break;
    }
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER pwm mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  buffer: the source memory buffer address
    \param[in]  blength: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_pwm_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, uint32_t *buffer, uint16_t blength)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
    dma_irq.half_finish_handle = NULL;
    dma_irq.error_handle = _timer_dmaerror_callback;
    
    switch (channel){
    case TIMER_CH_0:
        /* enable the DMA CH0 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], (uint32_t)buffer,TIMER_CH0CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;

    case TIMER_CH_1:
        /* enable the DMA CH1 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], (uint32_t)buffer,TIMER_CH1CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;

    case TIMER_CH_2:
        /* enable the DMA CH2 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH2], (uint32_t)buffer,TIMER_CH2CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;

    case TIMER_CH_3:
        /* enable the DMA CH3 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH3], (uint32_t)buffer,TIMER_CH3CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
        break;

    default:
        break;
    }

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);


    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER pwm mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_pwm_dma(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch (channel){
    case TIMER_CH_0:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
        break;

    case TIMER_CH_1:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
        break;

    case TIMER_CH_2:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH2]);
        break;

    case TIMER_CH_3:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH3]);
        break;

    default:
        break;
    }

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    
    return ret_val;
}

/*!
    \brief      start TIMER complementary pwm mode 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_pwm_negtive(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary pwm mode 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_pwm_negtive(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary pwm mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_pwm_negtive_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch (channel){
    case TIMER_CH_0:
        /* enable the TIMER compare 0 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* enable the TIMER compare 1 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* enable the TIMER compare 2 interrupt */
        timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
        break;

    default:
        break;
    }
    
    /* enable the TIMER interrupt */
    timer_interrupt_enable(timer_dev->periph, TIMER_INT_BRK);
    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary pwm mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_pwm_negtive_interrupt(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch (channel){
    case TIMER_CH_0:
        /* disable the TIMER compare 0 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
        break;

    case TIMER_CH_1:
        /* disable the TIMER compare 1 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
        break;

    case TIMER_CH_2:
        /* disable the TIMER compare 2 interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
        break;

    default:
        break;
    }
    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    if(RESET == (TIMER_CHCTL2(timer_dev->periph) & (TIMER_CHCTL2_CH0NEN | TIMER_CHCTL2_CH1NEN | TIMER_CHCTL2_CH2NEN))){
        /* disable the TIMER interrupt */
        timer_interrupt_disable(timer_dev->periph, TIMER_INT_BRK);
    }
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementary pwm mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[in]  buffer: the source memory buffer address
    \param[in]  blength: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_pwm_negtive_dma(hal_timer_dev_struct *timer_dev, uint16_t channel, uint32_t *buffer, uint16_t blength)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
    dma_irq.half_finish_handle = NULL;
    dma_irq.error_handle = _timer_dmaerror_callback;
    
    switch (channel){
    case TIMER_CH_0:
        /* enable the DMA CH0 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], (uint32_t)buffer,TIMER_CH0CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
        break;

    case TIMER_CH_1:
        /* enable the DMA CH1 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], (uint32_t)buffer,TIMER_CH1CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
        break;

    case TIMER_CH_2:
        /* enable the DMA CH2 interrupt */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH2], (uint32_t)buffer,TIMER_CH2CV_ADDRESS(timer_dev->periph),blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
        break;

    default:
        break;
    }
    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementary pwm mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,14..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_pwm_negtive_dma(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch (channel){
    case TIMER_CH_0:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
        break;

    case TIMER_CH_1:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
        break;

    case TIMER_CH_2:
        /* disbale the DMA CH0 interrupt */
        timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
        /* stop DMA transfer */
        hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH2]);
        break;

    default:
        break;
    }
    /* configure TIMER channel complementary output enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, channel, TIMER_CCXN_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      initialize TIMER input capture mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  timer_capture: TIMER channel input capture parameter struct
                  icpolarity: TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_FALLING, TIMER_IC_POLARITY_BOTH_EDGE
                  icselection: TIMER_IC_SELECTION_DIRECTTI, TIMER_IC_SELECTION_INDIRECTTI, TIMER_IC_SELECTION_ITS
                  icprescaler: TIMER_IC_PSC_DIV1, TIMER_IC_PSC_DIV2, TIMER_IC_PSC_DIV4, TIMER_IC_PSC_DIV8
                  icfilter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_inputcapture_init(hal_timer_dev_struct *timer_dev, uint16_t channel, hal_timer_inputcapture_struct *timer_capture)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check the parameters */
    if(NULL == timer_capture){
        HAL_DEBUGE("pointer [timer_capture] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER input capture parameter */
    timer_input_capture_config(timer_dev->periph, channel, timer_capture);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER input capture mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_inputcapture(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER input capture mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_inputcapture(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER input capture mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_inputcapture_interrupt(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* enable the TIMER interrupt */
            timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
            break;
        case TIMER_CH_1:
            /* enable the TIMER interrupt */
            timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH1);
            break;
        case TIMER_CH_2:
            /* enable the TIMER interrupt */
            timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH2);
            break;
        case TIMER_CH_3:
            /* enable the TIMER interrupt */
            timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH3);
            break;
        default:
            HAL_DEBUGE("parameter [channel] value is invalid");
            return HAL_ERR_VAL;
    }
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER input capture mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_inputcapture_interrupt(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* disable the TIMER interrupt */
            timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
            break;
        case TIMER_CH_1:
            /* disable the TIMER interrupt */
            timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH1);
            break;
        case TIMER_CH_2:
            /* disable the TIMER interrupt */
            timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH2);
            break;
        case TIMER_CH_3:
            /* disable the TIMER interrupt */
            timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH3);
            break;
        default:
            HAL_DEBUGE("parameter [channel] value is invalid");
            return HAL_ERR_VAL;
    }
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER channel input capture and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  buffer: the source memory buffer address
    \param[in]  blength: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_inputcapture_dma(hal_timer_dev_struct *timer_dev, uint32_t channel, uint32_t *buffer, uint16_t blength)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
        /* channel DMA config */
        dma_irq.full_finish_handle = _channel_capture_dmatc_callback;
        dma_irq.half_finish_handle = NULL;
        dma_irq.error_handle = _timer_dmaerror_callback;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], TIMER_CH0CV_ADDRESS(timer_dev->periph), (uint32_t)buffer, blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
            break;
        case TIMER_CH_1:
        /* channel DMA config */
        dma_irq.full_finish_handle = _channel_capture_dmatc_callback;
        dma_irq.half_finish_handle = NULL;
        dma_irq.error_handle = _timer_dmaerror_callback;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], TIMER_CH1CV_ADDRESS(timer_dev->periph), (uint32_t)buffer, blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
            break;
        case TIMER_CH_2:
        /* channel DMA config */
        dma_irq.full_finish_handle = _channel_capture_dmatc_callback;
        dma_irq.half_finish_handle = NULL;
        dma_irq.error_handle = _timer_dmaerror_callback;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH2],  TIMER_CH2CV_ADDRESS(timer_dev->periph), (uint32_t)buffer, blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
            break;
        case TIMER_CH_3:
        /* channel DMA config */
        dma_irq.full_finish_handle = _channel_capture_dmatc_callback;
        dma_irq.half_finish_handle = NULL;
        dma_irq.error_handle = _timer_dmaerror_callback;
        /* start DMA interrupt mode transfer */
        hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH3], TIMER_CH3CV_ADDRESS(timer_dev->periph), (uint32_t)buffer,  blength, &dma_irq);
        timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
            break;
        default:
            HAL_DEBUGE("parameter [channel] value is invalid");
            return HAL_ERR_VAL;
    }
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER channel input capture and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_inputcapture_dma(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* disable the TIMER DMA */
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
            break;
        case TIMER_CH_1:
            /* disable the TIMER DMA */
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
            break;
        case TIMER_CH_2:
            /* disable the TIMER DMA */
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH2]);
            break;
        case TIMER_CH_3:
            /* disable the TIMER DMA */
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH3]);
            break;
        default:
            HAL_DEBUGE("parameter [channel] value is invalid");
            return HAL_ERR_VAL;
    }
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, channel, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      read TIMER channel capture compare register value
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel: 
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     channel capture compare register value
*/
uint32_t hal_timer_read_capture_value(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    uint32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* read TIMER channel capture compare register value */
    ret_val = timer_channel_capture_value_register_read(timer_dev->periph, channel);
    return ret_val;
}

/*!
    \brief      initialize TIMER single pulse mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_singlepulse:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_SP_MODE_SINGLE: single pulse mode
      \arg        TIMER_SP_MODE_REPETITIVE: repetitive pulse mode
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_singlepulse_mode_init(hal_timer_dev_struct *timer_dev, uint32_t timer_singlepulse)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER single pulse mode */
    timer_single_pulse_mode_config(timer_dev->periph, timer_singlepulse);
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER single pulse mode configure
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  singlepulse: TIMER single pulse mode struct
                  sp_ocmode:
                    the argument could be selected from enumeration <hal_timer_output_compare_enum>
                sp_pulse:0~65535
                sp_ocpolarity:TIMER_OC_POLARITY_HIGH,TIMER_OC_POLARITY_LOW
                sp_ocnpolarity:TIMER_OCN_POLARITY_HIGH,TIMER_OCN_POLARITY_LOW
                sp_ocidlestate:TIMER_OC_IDLE_STATE_LOW,TIMER_OC_IDLE_STATE_HIGH
                sp_ocnidlestate:TIMER_OCN_IDLE_STATE_LOW,TIMER_OCN_IDLE_STATE_HIGH
                sp_ci_selection:TIMER_IC_SELECTION_DIRECTTI,TIMER_IC_SELECTION_INDIRECTTI,TIMER_IC_SELECTION_ITS
                sp_ci_polarity:TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING,TIMER_IC_POLARITY_BOTH_EDGE
                sp_ci_filter:0~15
    \param[in]  channel_out: TIMER output channel.The channel will output single pulse
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[in]  channel_in: TIMER input channel.If the channel input a active signal,TIMER will start count.
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_singlepulse_channel_config(hal_timer_dev_struct *timer_dev,hal_timer_singlepulse_struct *singlepulse,uint32_t channel_out,uint32_t channel_in)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(NULL == singlepulse){
        HAL_DEBUGE("pointer [singlepulse] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(channel_out != channel_in){
        switch(channel_out){
            case TIMER_CH_0:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph,TIMER_CH_0,TIMER_CCX_DISABLE);
            if((TIMER0 == timer_dev->periph) || (TIMER14 == timer_dev->periph) || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)){
                /* configure TIMER channel complementary output enable state */
                timer_channel_complementary_output_state_config(timer_dev->periph,TIMER_CH_0,TIMER_CCXN_DISABLE);
                timer_channel_complementary_output_polarity_config(timer_dev->periph, TIMER_CH_0, singlepulse->sp_ocnpolarity);
                /* reset the ISO0 bit */
                TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO0);
                /* set the ISO0 bit */
                TIMER_CTL1(timer_dev->periph) |= (uint32_t)singlepulse->sp_ocidlestate;
                /* reset the ISO0N bit */
                TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO0N);
                /* set the ISO0N bit */
                TIMER_CTL1(timer_dev->periph) |= (uint32_t)singlepulse->sp_ocnidlestate;
            }
            TIMER_CHCTL0(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL0_CH0MS;
                break;
            case TIMER_CH_1:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph,TIMER_CH_1,TIMER_CCX_DISABLE);
            if((TIMER0 == timer_dev->periph) || (TIMER14 == timer_dev->periph)){
                if(TIMER0 == timer_dev->periph){
                    /* configure TIMER channel complementary output enable state */
                    timer_channel_complementary_output_state_config(timer_dev->periph,TIMER_CH_1,TIMER_CCXN_DISABLE);
                    timer_channel_complementary_output_polarity_config(timer_dev->periph, TIMER_CH_1, singlepulse->sp_ocnpolarity);
                    /* reset the ISO1N bit */
                    TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO1N);
                    /* set the ISO1N bit */
                    TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(singlepulse->sp_ocnidlestate) << 2U);
                }
                /* reset the ISO1 bit */
                TIMER_CTL1(timer_dev->periph) &= (~(uint32_t)TIMER_CTL1_ISO1);
                /* set the ISO1 bit */
                TIMER_CTL1(timer_dev->periph) |= (uint32_t)((uint32_t)(singlepulse->sp_ocidlestate) << 2U);

            }
            TIMER_CHCTL0(timer_dev->periph) &= ~(uint32_t)TIMER_CHCTL0_CH1MS;
                break;
            default:
                HAL_DEBUGE("parameter [channel_out] value is invalid");
                return HAL_ERR_VAL;
        }
        /* configure TIMER channel output compare mode */
        timer_channel_output_mode_config(timer_dev->periph, channel_out,singlepulse->sp_ocmode);
        /* configure TIMER channel output pulse value */
        timer_channel_output_pulse_value_config(timer_dev->periph, channel_out, singlepulse->sp_pulse);
        /* configure TIMER channel output polarity */
        timer_channel_output_polarity_config(timer_dev->periph, channel_out, singlepulse->sp_ocpolarity);
        
        switch(channel_in){
            case TIMER_CH_0:
            /* reset the CH0EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);

            /* reset the CH0P and CH0NP bits */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
            TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)(singlepulse->sp_ci_polarity);
            if(((TIMER0 == timer_dev->periph) || (TIMER2 == timer_dev->periph) || (TIMER14 == timer_dev->periph))){
                /* reset the CH0MS bit */
                TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
                TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)(singlepulse->sp_ci_selection);
            }else
            {
                /* reset the CH0MS bit */
                TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
                TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)TIMER_IC_SELECTION_DIRECTTI;
            }
            /* reset the CH0CAPFLT bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(singlepulse->sp_ci_filter) << 4U);
            
            /* configure TIMER channel input capture prescaler value */
            timer_channel_input_capture_prescaler_config(timer_dev->periph, channel_in,TIMER_IC_PSC_DIV1);
            /* select TIMER input trigger source */
            timer_input_trigger_source_select(timer_dev->periph,TIMER_SMCFG_TRGSEL_CI0FE0);
            /* select TIMER slave mode */
            timer_slave_mode_select(timer_dev->periph, TIMER_SLAVE_MODE_EVENT);

                break;
            case TIMER_CH_1:
            /* reset the CH1EN bit */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH1EN);

            /* reset the CH1P and CH1NP bits */
            TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH1P | TIMER_CHCTL2_CH1NP));
            TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)((uint32_t)(singlepulse->sp_ci_polarity) << 4U);
            /* reset the CH1MS bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH1MS);
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(singlepulse->sp_ci_selection) << 8U);
            /* reset the CH1CAPFLT bit */
            TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH1CAPFLT);
            TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(singlepulse->sp_ci_filter) << 12U);
            
            /* configure TIMER channel input capture prescaler value */
            timer_channel_input_capture_prescaler_config(timer_dev->periph, channel_in,TIMER_IC_PSC_DIV1);
            /* select TIMER input trigger source */
            timer_input_trigger_source_select(timer_dev->periph,TIMER_SMCFG_TRGSEL_CI1FE1);
            /* select TIMER slave mode */
            timer_slave_mode_select(timer_dev->periph, TIMER_SLAVE_MODE_EVENT);
                break;
            default:
                HAL_DEBUGE("parameter [channel_in] value is invalid");
                return HAL_ERR_VAL;
        }
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_VAL;
    }
}

/*!
    \brief      start TIMER single pulse mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  out_channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_singlepulse(hal_timer_dev_struct *timer_dev, uint32_t out_channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph,TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(timer_dev->periph,TIMER_CH_1, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER single pulse mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  out_channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_singlepulse(hal_timer_dev_struct *timer_dev, uint32_t out_channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph,TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(timer_dev->periph,TIMER_CH_1, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER single pulse mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  out_channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_singlepulse_interrupt(hal_timer_dev_struct *timer_dev, uint32_t out_channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* enable the TIMER interrupt */
    timer_interrupt_enable(timer_dev->periph,TIMER_INT_CH0);
    timer_interrupt_enable(timer_dev->periph,TIMER_INT_CH1);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph,TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(timer_dev->periph,TIMER_CH_1, TIMER_CCX_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER single pulse mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  out_channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_singlepulse_interrupt(hal_timer_dev_struct *timer_dev, uint32_t out_channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* disable the TIMER interrupt */
    timer_interrupt_disable(timer_dev->periph,TIMER_INT_CH0);
    timer_interrupt_disable(timer_dev->periph,TIMER_INT_CH1);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph,TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(timer_dev->periph,TIMER_CH_1, TIMER_CCX_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementay channel single pulse mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  out_channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_negtive_singlepulse(hal_timer_dev_struct *timer_dev, uint16_t out_channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, out_channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementay channel single pulse mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  out_channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_negtive_singlepulse(hal_timer_dev_struct *timer_dev, uint16_t out_channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, out_channel, TIMER_CCXN_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER complementay channel single pulse mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  out_channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_negtive_singlepulse_interrupt(hal_timer_dev_struct *timer_dev, uint16_t out_channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* enable the TIMER interrupt */
    timer_interrupt_enable(timer_dev->periph,TIMER_INT_CH0);
    timer_interrupt_enable(timer_dev->periph,TIMER_INT_CH1);
    /* configure TIMER channel enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, out_channel, TIMER_CCXN_ENABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, ENABLE);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER complementay channel single pulse mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  out_channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_negtive_singlepulse_interrupt(hal_timer_dev_struct *timer_dev, uint16_t out_channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* disable the TIMER interrupt */
    timer_interrupt_disable(timer_dev->periph,TIMER_INT_CH0);
    timer_interrupt_disable(timer_dev->periph,TIMER_INT_CH1);
    /* configure TIMER channel enable state */
    timer_channel_complementary_output_state_config(timer_dev->periph, out_channel, TIMER_CCXN_DISABLE);
    /* enable or disable TIMER primary output */
    _timer_primary_output_config(timer_dev, DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      initialize TIMER quadrature decoder mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  decode:
                  decodemode:TIMER_QUADRATURE_DECODER_MODE0, TIMER_QUADRATURE_DECODER_MODE1, TIMER_QUADRATURE_DECODER_MODE2
                  ci0_polarity/ci1_polarity: TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING
                  ci0_selection/ci1_selection: TIMER_IC_SELECTION_DIRECTTI,TIMER_IC_SELECTION_INDIRECTTI,TIMER_IC_SELECTION_ITS
                  ci0_prescaler/ci1_prescaler: TIMER_IC_PSC_DIV1,TIMER_IC_PSC_DIV2,TIMER_IC_PSC_DIV4,TIMER_IC_PSC_DIV8
                  ci0_filter/ci1_filter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_quadrature_decoder_init(hal_timer_dev_struct *timer_dev, hal_timer_quadrature_decoder_struct *decode)
{
    timer_ic_parameter_struct ic0_capture;
    timer_ic_parameter_struct ic1_capture;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    TIMER_SMCFG(timer_dev->periph) &= (~(uint32_t)TIMER_SMCFG_SMC);
    TIMER_SMCFG(timer_dev->periph) |= (uint32_t)decode->decodemode;

    /* channel 0 input parameter struct */
    ic0_capture.icfilter = decode->ci0_filter;
    ic0_capture.icpolarity = decode->ci0_polarity;
    ic0_capture.icprescaler = decode->ci0_prescaler;
    ic0_capture.icselection = decode->ci0_selection;
    /* channel 1 input parameter struct */
    ic1_capture.icfilter = decode->ci1_filter;
    ic1_capture.icpolarity = decode->ci1_polarity;
    ic1_capture.icprescaler = decode->ci1_prescaler;
    ic1_capture.icselection = decode->ci1_selection;
    /* configure TIMER input capture parameter */
    timer_input_capture_config(timer_dev->periph, TIMER_CH_0, &ic0_capture);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    /* configure TIMER input capture parameter */
    timer_input_capture_config(timer_dev->periph, TIMER_CH_1, &ic1_capture);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER quadrature decoder mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_quadrature_decoder(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* configure TIMER channel 0 enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
            break;
        case TIMER_CH_1:
            /* configure TIMER channel 1 enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
            break;
        case TIMER_CH_0_1:
            /* configure TIMER channel 0 and channel 1 enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
            break;
        default:
            HAL_DEBUGW("parameter [channel] value is undefine");
            break;
    }
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER quadrature decoder mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_quadrature_decoder(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* configure TIMER channel 0 enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
            break;
        case TIMER_CH_1:
            /* configure TIMER channel 1 enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
            break;
        case TIMER_CH_0_1:
            /* configure TIMER channel 0 and channel 1 enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
            break;
        default:
            HAL_DEBUGW("parameter [channel] value is undefine");
            break;
    }
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER quadrature decoder mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_quadrature_decoder_interrupt(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* configure TIMER channel 0 enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
            /* enable the TIMER interrupt */
            timer_interrupt_enable(timer_dev->periph, TIMER_CH_0);
            break;
        case TIMER_CH_1:
            /* configure TIMER channel 1 enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
            /* enable the TIMER interrupt */
            timer_interrupt_enable(timer_dev->periph, TIMER_CH_1);
            break;
        case TIMER_CH_0_1:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
            /* enable the TIMER interrupt */
            timer_interrupt_enable(timer_dev->periph, TIMER_CH_0);
            timer_interrupt_enable(timer_dev->periph, TIMER_CH_1);
            break;
        default:
            HAL_DEBUGW("parameter [channel] value is undefine");
            break;
    }
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER quadrature decoder mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_quadrature_decoder_interrupt(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
            /* disable the TIMER interrupt */
            timer_interrupt_disable(timer_dev->periph, TIMER_CH_0);
            break;
        case TIMER_CH_1:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
            /* disable the TIMER interrupt */
            timer_interrupt_disable(timer_dev->periph, TIMER_CH_1);
            break;
        case TIMER_CH_0_1:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
            /* disable the TIMER interrupt */
            timer_interrupt_disable(timer_dev->periph, TIMER_CH_0);
            timer_interrupt_disable(timer_dev->periph, TIMER_CH_1);
            break;
        default:
            HAL_DEBUGW("parameter [channel] value is undefine");
            break;
    }
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    
    return ret_val;
}

/*!
    \brief      start TIMER quadrature decoder mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[in]  buffer0: the source memory buffer address for channel0
    \param[in]  buffer1: the source memory buffer address for channel1
    \param[in]  blength: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_quadrature_decoder_dma(hal_timer_dev_struct *timer_dev, uint32_t channel,
                                    uint32_t *buffer0, uint32_t *buffer1, uint16_t blength)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* channel DMA config */
            dma_irq.full_finish_handle = _channel_capture_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], TIMER_CH0CV_ADDRESS(timer_dev->periph), (uint32_t)buffer0, blength, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
            /* enable a TIMER */
            timer_enable(timer_dev->periph);
            break;
        case TIMER_CH_1:
            /* channel DMA config */
            dma_irq.full_finish_handle = _channel_capture_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], TIMER_CH1CV_ADDRESS(timer_dev->periph), (uint32_t)buffer1, blength, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);
            /* enable a TIMER */
            timer_enable(timer_dev->periph);
            break;
        case TIMER_CH_0_1:
            /* channel DMA config */
            dma_irq.full_finish_handle = _channel_capture_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], TIMER_CH0CV_ADDRESS(timer_dev->periph), (uint32_t)buffer0, blength, &dma_irq);
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], TIMER_CH1CV_ADDRESS(timer_dev->periph), (uint32_t)buffer1, blength, &dma_irq);

            /* enable the TIMER DMA */
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_ENABLE);

            /* enable a TIMER */
            timer_enable(timer_dev->periph);
            break;
        default:
            HAL_DEBUGW("parameter [channel] value is undefine");
            break;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER quadrature decoder mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_0_1: TIMER channel0 and TIMER channel1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_quadrature_decoder_dma(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
        case TIMER_CH_0:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
            break;
        case TIMER_CH_1:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
            break;
        case TIMER_CH_0_1:
            /* configure TIMER channel enable state */
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
            timer_channel_output_state_config(timer_dev->periph, TIMER_CH_1, TIMER_CCX_DISABLE);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
            break;
        default:
            HAL_DEBUGW("parameter [channel] value is undefine");
            break;
    }
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER DMA mode for writing data to TIMER
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dma_startaddr:
                    the argument could be selected from enumeration <hal_timer_dma_start_address_enum>
    \param[in]  dma_count: 
                only one parameter can be selected which is shown as below:
       \arg        TIMER_DMACFG_DMATC_xTRANSFER(x=1..18): DMA transfer x time
    \param[in]  dma_reqsrc: 
                    the argument could be selected from enumeration <hal_timer_dma_request_source_enum>
    \param[in]  srcbuffer: the source memory buffer address
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_dmatransfer_write(hal_timer_dev_struct *timer_dev, hal_timer_dma_start_address_enum dma_startaddr, uint32_t dma_count,\
                            hal_timer_dma_request_source_enum dma_reqsrc, uint32_t *srcbuffer)
{
    hal_dma_irq_struct dma_irq;
    uint32_t data_length = 0;
    data_length = (uint32_t)((dma_count>>8U) + 1); 
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if((NULL == srcbuffer) && (data_length > 0U))
    {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(dma_reqsrc){
        case TIMER_DMA_UPD:
            dma_irq.full_finish_handle = _update_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_UP], (uint32_t)srcbuffer,TIMER_DMATB_ADDRESS(timer_dev->periph),data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_UPD);
            break;
        case TIMER_DMA_CH0D:
            dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], (uint32_t)srcbuffer,TIMER_DMATB_ADDRESS(timer_dev->periph),data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
            break;
        case TIMER_DMA_CH1D:
            dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], (uint32_t)srcbuffer,TIMER_DMATB_ADDRESS(timer_dev->periph),data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
            break;
        case TIMER_DMA_CH2D:
            dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH2], (uint32_t)srcbuffer,TIMER_DMATB_ADDRESS(timer_dev->periph),data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
            break;
        case TIMER_DMA_CH3D:
            dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH3], (uint32_t)srcbuffer,TIMER_DMATB_ADDRESS(timer_dev->periph),data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
            break;
        case TIMER_DMA_CMTD:
            dma_irq.full_finish_handle = _commutation_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CMT], (uint32_t)srcbuffer,TIMER_DMATB_ADDRESS(timer_dev->periph),data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CMTD);
            break;
        case TIMER_DMA_TRGD:
            dma_irq.full_finish_handle = _trigger_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_TRG], (uint32_t)srcbuffer,TIMER_DMATB_ADDRESS(timer_dev->periph),data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_TRGD);
            break;
        default:
            HAL_DEBUGW("parameter [dma_reqsrc] value is undefine");
            break;
    }
    /* configure the TIMER DMA transfer */
    timer_dma_transfer_config(timer_dev->periph, dma_startaddr,dma_count);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER DMA mode for writing data to TIMER
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dma_reqsrc: 
                    the argument could be selected from enumeration <hal_timer_dma_request_source_enum>
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_dmatransfer_write(hal_timer_dev_struct *timer_dev, hal_timer_dma_request_source_enum dma_reqsrc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* DMA config for request */
    switch(dma_reqsrc){
        case TIMER_DMA_UPD:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_UP]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_UPD);
            break;
        case TIMER_DMA_CH0D:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
            break;
        case TIMER_DMA_CH1D:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
            break;
        case TIMER_DMA_CH2D:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH2]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
            break;
        case TIMER_DMA_CH3D:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH3]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
            break;
        case TIMER_DMA_CMTD:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CMT]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CMTD);
            break;
        case TIMER_DMA_TRGD:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_TRG]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_TRGD);
            break;
        default:
            HAL_DEBUGW("parameter [dma_reqsrc] value is undefine");
            break;
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER DMA mode for read data from TIMER
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dma_startaddr:
                    the argument could be selected from enumeration <hal_timer_dma_start_address_enum>
    \param[in]  dma_count: 
                only one parameter can be selected which is shown as below:
       \arg        TIMER_DMACFG_DMATC_xTRANSFER(x=1..18): DMA transfer x time
    \param[in]  dma_reqsrc: 
                    the argument could be selected from enumeration <hal_timer_dma_request_source_enum>
    \param[in]  destbuffer: the destinating memory buffer address
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_dmatransfer_read(hal_timer_dev_struct *timer_dev, hal_timer_dma_start_address_enum dma_startaddr, uint32_t dma_count,\
                            hal_timer_dma_request_source_enum dma_reqsrc, uint32_t *destbuffer)
{
    hal_dma_irq_struct dma_irq;
    uint32_t data_length = 0;
    data_length = (uint32_t)((dma_count>>8U) + 1); 
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if((NULL == destbuffer) && (data_length > 0U))
    {
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* DMA config for request */
    switch(dma_reqsrc){
        case TIMER_DMA_UPD:
            dma_irq.full_finish_handle = _update_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_UP], TIMER_DMATB_ADDRESS(timer_dev->periph),(uint32_t)destbuffer,data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_UPD);
            break;
        case TIMER_DMA_CH0D:
            dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], TIMER_DMATB_ADDRESS(timer_dev->periph), (uint32_t)destbuffer,data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
            break;
        case TIMER_DMA_CH1D:
            dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH1], TIMER_DMATB_ADDRESS(timer_dev->periph) ,(uint32_t)destbuffer,data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH1D);
            break;
        case TIMER_DMA_CH2D:
            dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH2], TIMER_DMATB_ADDRESS(timer_dev->periph),(uint32_t)destbuffer,data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH2D);
            break;
        case TIMER_DMA_CH3D:
            dma_irq.full_finish_handle = _channel_compare_pwm_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH3], TIMER_DMATB_ADDRESS(timer_dev->periph),(uint32_t)destbuffer,data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CH3D);
            break;
        case TIMER_DMA_CMTD:
            dma_irq.full_finish_handle = _commutation_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CMT], TIMER_DMATB_ADDRESS(timer_dev->periph),(uint32_t)destbuffer,data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_CMTD);
            break;
        case TIMER_DMA_TRGD:
            dma_irq.full_finish_handle = _trigger_dmatc_callback;
            dma_irq.half_finish_handle = NULL;
            dma_irq.error_handle = _timer_dmaerror_callback;
            /* start DMA interrupt mode transfer */
            hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_TRG], TIMER_DMATB_ADDRESS(timer_dev->periph),(uint32_t)destbuffer,data_length, &dma_irq);
            timer_dma_enable(timer_dev->periph, TIMER_DMA_TRGD);
            break;
        default:
            HAL_DEBUGW("parameter [dma_reqsrc] value is undefine");
            break;
    }
    /* configure the TIMER DMA transfer */
    timer_dma_transfer_config(timer_dev->periph, dma_startaddr,dma_count);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER DMA mode for read data from TIMER
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  dma_reqsrc: 
                    the argument could be selected from enumeration <hal_timer_dma_request_source_enum>
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_dmatransfer_read(hal_timer_dev_struct *timer_dev, hal_timer_dma_request_source_enum dma_reqsrc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(dma_reqsrc){
        case TIMER_DMA_UPD:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_UP]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_UPD);
            break;
        case TIMER_DMA_CH0D:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
            break;
        case TIMER_DMA_CH1D:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH1]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH1D);
            break;
        case TIMER_DMA_CH2D:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH2]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH2D);
            break;
        case TIMER_DMA_CH3D:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH3]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CH3D);
            break;
        case TIMER_DMA_CMTD:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CMT]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_CMTD);
            break;
        case TIMER_DMA_TRGD:
            /* stop DMA transfer */
            hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_TRG]);
            timer_dma_disable(timer_dev->periph, TIMER_DMA_TRGD);
            break;
        default:
            HAL_DEBUGW("parameter [dma_reqsrc] value is undefine");
            break;
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize TIMER HALL mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  hall_init: 
                  cmt_delay; 0x0-0xFFFF
                  hall_polarity: TIMER_IC_POLARITY_RISING,TIMER_IC_POLARITY_FALLING,TIMER_IC_POLARITY_BOTH_EDGE
                  hall_prescaler: TIMER_IC_PSC_DIV1,TIMER_IC_PSC_DIV2,TIMER_IC_PSC_DIV4,TIMER_IC_PSC_DIV8
                  hall_filter: 0~15
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_hall_init(hal_timer_dev_struct *timer_dev,hal_timer_hall_struct *hall_init)
{
    hal_timer_outputcompare_struct hall_oc1;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(NULL == hall_init){
        HAL_DEBUGE("pointer [hall_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* reset the CH0EN bit */
    TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL2_CH0EN);

    /* reset the CH0P and CH0NP bits */
    TIMER_CHCTL2(timer_dev->periph) &= (~(uint32_t)(TIMER_CHCTL2_CH0P | TIMER_CHCTL2_CH0NP));
    TIMER_CHCTL2(timer_dev->periph) |= (uint32_t)(hall_init->hall_polarity);
    /* reset the CH0MS bit */
    TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0MS);
    TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)(TIMER_IC_SELECTION_ITS);
    /* reset the CH0CAPFLT bit */
    TIMER_CHCTL0(timer_dev->periph) &= (~(uint32_t)TIMER_CHCTL0_CH0CAPFLT);
    TIMER_CHCTL0(timer_dev->periph) |= (uint32_t)((uint32_t)(hall_init->hall_filter) << 4U);
    /* configure TIMER channel input capture prescaler value */
    timer_channel_input_capture_prescaler_config(timer_dev->periph, TIMER_CH_0, (uint16_t)(hall_init->hall_prescaler));
    
    /* configure TIMER hall sensor mode*/
    timer_hall_mode_config(timer_dev->periph, TIMER_HALLINTERFACE_ENABLE);
    /* select TIMER input trigger source */
    timer_input_trigger_source_select(timer_dev->periph,TIMER_SMCFG_TRGSEL_CI0F_ED);
    
    /* select TIMER slave mode */
    timer_slave_mode_select(timer_dev->periph,TIMER_SLAVE_MODE_RESTART);

    /* TIMER output compare mode struct config */
    hall_oc1.output_pulse = hall_init->cmt_delay;
    hall_oc1.output_mode = TIMER_OUTPUT_PWM1_MODE;
    hall_oc1.output_fastmode = TIMER_OC_FAST_DISABLE;
    hall_oc1.output_idlestate = TIMER_OC_IDLE_STATE_LOW;
    hall_oc1.outputn_idlestate = TIMER_OC_IDLE_STATE_LOW;
    hall_oc1.outputn_polarity = TIMER_OC_POLARITY_HIGH;
    hall_oc1.output_polarity = TIMER_OC_POLARITY_HIGH;
    /* initialize TIMER output compare mode */
    hal_timer_outputcompare_init(timer_dev,TIMER_CH_1,&hall_oc1);
    
    /* select TIMER master mode output trigger source */
    timer_master_output_trigger_source_select(timer_dev->periph, TIMER_TRI_OUT_SRC_O1CPRE);

    return HAL_ERR_NONE;
}

/*!
    \brief      start TIMER HALL mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_hall(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER HALL mode
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_hall(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER HALL mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_hall_interrupt(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* enable the TIMER interrupt */
    timer_interrupt_enable(timer_dev->periph, TIMER_INT_CH0);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER HALL mode and channel interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_hall_interrupt(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* disable the TIMER interrupt */
    timer_interrupt_disable(timer_dev->periph, TIMER_INT_CH0);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      start TIMER HALL mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  buffer: the source memory buffer address
    \param[in]  blength: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_start_hall_dma(hal_timer_dev_struct *timer_dev, uint32_t *buffer, uint16_t blength)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    dma_irq.full_finish_handle = _channel_capture_dmatc_callback;
    dma_irq.error_handle = _timer_dmaerror_callback;
    dma_irq.half_finish_handle = NULL;
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(timer_dev->hdma[TIM_DMA_ID_CH0], TIMER_CH0CV_ADDRESS(timer_dev->periph), (uint32_t)buffer, blength, &dma_irq);
    timer_dma_enable(timer_dev->periph, TIMER_DMA_CH0D);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_ENABLE);
    /* enable a TIMER */
    timer_enable(timer_dev->periph);
    return HAL_ERR_NONE;
}

/*!
    \brief      stop TIMER HALL mode and channel dma request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_stop_hall_dma(hal_timer_dev_struct *timer_dev)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    timer_dma_disable(timer_dev->periph, TIMER_DMA_CH0D);
    /* configure TIMER channel enable state */
    timer_channel_output_state_config(timer_dev->periph, TIMER_CH_0, TIMER_CCX_DISABLE);
    /* stop DMA transfer */
    hal_dma_stop(timer_dev->hdma[TIM_DMA_ID_CH0]);
    /* disable TIMER */
    ret_val = _timer_disable(timer_dev);
    return ret_val;
}

/*!
    \brief      config TIMER commutation_event
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  trigger_source:
       \arg        TIMER_TRIGGER_SOURCE_ITI0: trigger input source selection:ITI0 
       \arg        TIMER_TRIGGER_SOURCE_ITI1: trigger input source selection:ITI1 
       \arg        TIMER_TRIGGER_SOURCE_ITI2: trigger input source selection:ITI2 
       \arg        TIMER_TRIGGER_SOURCE_ITI3: trigger input source selection:ITI3 
       \arg        TIMER_TRIGGER_SOURCE_DISABLE: trigger input source selection:none 
    \param[in]  com_source:
                only one parameter can be selected which is shown as below:
       \arg        TIMER_UPDATECTL_CCU: the shadow registers are updated when CMTG bit is set
       \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers are updated when CMTG bit is set or an rising edge of TRGI occurs 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_commutation_event_config(hal_timer_dev_struct *timer_dev,uint32_t trigger_source, uint32_t com_source)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* trigger source config */
    if((TIMER_TRIGGER_SOURCE_ITI0 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI1 == trigger_source)
        || (TIMER_TRIGGER_SOURCE_ITI2 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI3 == trigger_source)){
        TIMER_SMCFG(timer_dev->periph) &= ~TIMER_SMCFG_TRGS;
        TIMER_SMCFG(timer_dev->periph) |= trigger_source;
    }
    /* enable channel capture/compare control shadow register */
    timer_channel_control_shadow_config(timer_dev->periph, ENABLE);
    /* configure TIMER channel control shadow register update control */
    timer_channel_control_shadow_update_config(timer_dev->periph, com_source);
    return HAL_ERR_NONE;
}

/*!
    \brief      config TIMER commutation_event and enable CMT interrupt
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  trigger_source:
       \arg        TIMER_TRIGGER_SOURCE_ITI0: trigger input source selection:ITI0 
       \arg        TIMER_TRIGGER_SOURCE_ITI1: trigger input source selection:ITI1 
       \arg        TIMER_TRIGGER_SOURCE_ITI2: trigger input source selection:ITI2 
       \arg        TIMER_TRIGGER_SOURCE_ITI3: trigger input source selection:ITI3 
       \arg        TIMER_TRIGGER_SOURCE_DISABLE: trigger input source selection:none 
    \param[in]  com_source:
                only one parameter can be selected which is shown as below:
       \arg        TIMER_UPDATECTL_CCU: the shadow registers are updated when CMTG bit is set
       \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers are updated when CMTG bit is set or an rising edge of TRGI occurs 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_commutation_event_interrupt_config(hal_timer_dev_struct *timer_dev,uint32_t trigger_source, uint32_t com_source)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* trigger source config */
    if((TIMER_TRIGGER_SOURCE_ITI0 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI1 == trigger_source)
        || (TIMER_TRIGGER_SOURCE_ITI2 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI3 == trigger_source)){
        TIMER_SMCFG(timer_dev->periph) &= ~TIMER_SMCFG_TRGS;
        TIMER_SMCFG(timer_dev->periph) |= trigger_source;
    }
    /* enable channel capture/compare control shadow register */
    timer_channel_control_shadow_config(timer_dev->periph, ENABLE);
    /* configure TIMER channel control shadow register update control */
    timer_channel_control_shadow_update_config(timer_dev->periph, com_source);
    
    /* enable the TIMER interrupt */
    timer_interrupt_enable(timer_dev->periph, TIMER_INT_CMT);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      config TIMER commutation_event and enable CMT DMA request
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  trigger_source:
       \arg        TIMER_TRIGGER_SOURCE_ITI0: trigger input source selection:ITI0 
       \arg        TIMER_TRIGGER_SOURCE_ITI1: trigger input source selection:ITI1 
       \arg        TIMER_TRIGGER_SOURCE_ITI2: trigger input source selection:ITI2 
       \arg        TIMER_TRIGGER_SOURCE_ITI3: trigger input source selection:ITI3 
       \arg        TIMER_TRIGGER_SOURCE_DISABLE: trigger input source selection:none 
    \param[in]  com_source:
                only one parameter can be selected which is shown as below:
       \arg        TIMER_UPDATECTL_CCU: the shadow registers are updated when CMTG bit is set
       \arg        TIMER_UPDATECTL_CCUTRI: the shadow registers are updated when CMTG bit is set or an rising edge of TRGI occurs 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_commutation_event_dma_config(hal_timer_dev_struct *timer_dev,uint32_t trigger_source, uint32_t com_source)
{
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* trigger source config */
    if((TIMER_TRIGGER_SOURCE_ITI0 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI1 == trigger_source)
        || (TIMER_TRIGGER_SOURCE_ITI2 == trigger_source) || (TIMER_TRIGGER_SOURCE_ITI3 == trigger_source)){
        TIMER_SMCFG(timer_dev->periph) &= ~TIMER_SMCFG_TRGS;
        TIMER_SMCFG(timer_dev->periph) |= trigger_source;
    }
    /* enable channel capture/compare control shadow register */
    timer_channel_control_shadow_config(timer_dev->periph, ENABLE);
    /* configure TIMER channel control shadow register update control */
    timer_channel_control_shadow_update_config(timer_dev->periph, com_source);
    
    /* DMA config for CMT dma request */
    dma_irq.full_finish_handle = _commutation_dmatc_callback;
    dma_irq.half_finish_handle = NULL;
    dma_irq.error_handle = _timer_dmaerror_callback;
    (timer_dev->hdma[TIM_DMA_ID_CMT])->dma_irq = dma_irq;
    timer_dma_enable(timer_dev->periph, TIMER_DMA_CMTD);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  user_func: point to TIMER interrupt callback functions structure
                  please refer to hal_timer_irq_struct
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_irq_handle_set(hal_timer_dev_struct *timer_dev, hal_timer_irq_struct *user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(NULL == user_func){
        HAL_DEBUGE("pointer [user_func] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* TIMER break interrupt call */
    timer_dev->timer_irq.break_usercb = (hal_irq_handle_cb)(user_func->break_usercb);
    /* channel interrupt for input capture */
    timer_dev->timer_irq.channelx_capture_usercb = (hal_irq_handle_cb)(user_func->channelx_capture_usercb);
    /* channel interrupt for compare output */
    timer_dev->timer_irq.channelx_compare_usercb = (hal_irq_handle_cb)(user_func->channelx_compare_usercb);
    /* channel interrupt for PWM output */
    timer_dev->timer_irq.channelx_pwm_usercb = (hal_irq_handle_cb)(user_func->channelx_pwm_usercb);
    /* TIMER commutation interrupt call */
    timer_dev->timer_irq.commutation_usercb = (hal_irq_handle_cb)(user_func->commutation_usercb);
    /* TIMER trigger interrupt call */
    timer_dev->timer_irq.trigger_usercb = (hal_irq_handle_cb)(user_func->trigger_usercb);
    /* TIMER update interrupt call */
    timer_dev->timer_irq.update_usercb = (hal_irq_handle_cb)(user_func->update_usercb);
    
    /* DMA channel error interrupt user callback */
    timer_dev->timer_irq.dmaerror_usercb = (hal_irq_handle_cb)(user_func->dmaerror_usercb);
    /* DMA transmission complete interrupt(TC) user callback for update dma request */
    timer_dev->timer_irq.dmatc_update_usercb = (hal_irq_handle_cb)(user_func->dmatc_update_usercb);
    /* DMA transmission complete interrupt(TC) user callback for channelx compare output and pwm output dma request */
    timer_dev->timer_irq.dmatc_chx_cmppwm_usercb = (hal_irq_handle_cb)(user_func->dmatc_chx_cmppwm_usercb);          
    /* DMA transmission complete interrupt(TC) user callback for channelx input capture dma request */
    timer_dev->timer_irq.dmatc_chx_capture_usercb = (hal_irq_handle_cb)(user_func->dmatc_chx_capture_usercb);         
    /* DMA transmission complete interrupt(TC) user callback for commutation DMA request */
    timer_dev->timer_irq.dmatc_commutation_usercb = (hal_irq_handle_cb)(user_func->dmatc_commutation_usercb);         
    /* DMA transmission complete interrupt(TC) user callback for trigger DMA request */
    timer_dev->timer_irq.dmatc_trigger_usercb = (hal_irq_handle_cb)(user_func->dmatc_trigger_usercb);             
    
    return HAL_ERR_NONE;
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_irq_handle_all_reset(hal_timer_dev_struct *timer_dev)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* TIMER interrupt handler reset */
    timer_dev->timer_irq.break_usercb = NULL;
    timer_dev->timer_irq.channelx_capture_usercb = NULL;
    timer_dev->timer_irq.channelx_compare_usercb = NULL;
    timer_dev->timer_irq.channelx_pwm_usercb = NULL;
    timer_dev->timer_irq.commutation_usercb = NULL;
    timer_dev->timer_irq.trigger_usercb = NULL;
    timer_dev->timer_irq.update_usercb = NULL;
    timer_dev->timer_irq.dmaerror_usercb = NULL;
    timer_dev->timer_irq.dmatc_chx_capture_usercb = NULL;
    timer_dev->timer_irq.dmatc_chx_cmppwm_usercb = NULL;
    timer_dev->timer_irq.dmatc_commutation_usercb = NULL;
    timer_dev->timer_irq.dmatc_trigger_usercb = NULL;
    timer_dev->timer_irq.dmatc_update_usercb = NULL;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      TIMER interrupt handler content function,which is merely used in timer_handler
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_timer_irq(hal_timer_dev_struct *timer_dev)
{
    /* check whether the update interrupt is set or not */
    if( SET == (timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_UP))){
        timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_UP);
        /* update interrupt handle */
        if(NULL != (timer_dev->timer_irq.update_usercb)){
            timer_dev->timer_irq.update_usercb(timer_dev);
        }
    }
    
    /* check whether the channel 0 capture/compare interrupt is set or not */
    if( SET == (timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CH0))){
        timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH0);
        timer_dev->service_channel = TIMER_SERVICE_CHANNEL_0;
        /* channel 0 capture interrupt handle */
        if( 0 != (TIMER_CHCTL0(timer_dev->periph) & TIMER_CHCTL0_CH0MS) ){
            if(NULL != (timer_dev->timer_irq.channelx_capture_usercb)){
                timer_dev->timer_irq.channelx_capture_usercb(timer_dev);
            }
        }else{
            /* channel 0 compare interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_compare_usercb)){
                timer_dev->timer_irq.channelx_compare_usercb(timer_dev);
            }
            /* channel 0 pwm interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_pwm_usercb)){
                timer_dev->timer_irq.channelx_pwm_usercb(timer_dev);
            }
        }
        /* clear service channel */
        timer_dev->service_channel = TIMER_SERVICE_CHANNEL_NONE;
    }
    /* check whether the channel 1 capture/compare interrupt is set or not */
    if( SET == (timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CH1))){
        timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH1);
        timer_dev->service_channel = TIMER_SERVICE_CHANNEL_1;
        /* channel 1 capture interrupt handle */
        if( 0 != (TIMER_CHCTL0(timer_dev->periph) & TIMER_CHCTL0_CH1MS) ){
            if(NULL != (timer_dev->timer_irq.channelx_capture_usercb)){
                timer_dev->timer_irq.channelx_capture_usercb(timer_dev);
            }
        }else{
            /* channel 1 compare interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_compare_usercb)){
                timer_dev->timer_irq.channelx_compare_usercb(timer_dev);
            }
            /* channel 1 pwm interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_pwm_usercb)){
                timer_dev->timer_irq.channelx_pwm_usercb(timer_dev);
            }
        }
        /* clear service channel */
        timer_dev->service_channel = TIMER_SERVICE_CHANNEL_NONE;
    }
    
    /* check whether the channel 2 capture/compare interrupt is set or not */
    if( SET == (timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CH2)))
    {
        timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH2);
        timer_dev->service_channel = TIMER_SERVICE_CHANNEL_2;
        /* channel 2 capture interrupt handle */
        if( 0 != (TIMER_CHCTL1(timer_dev->periph) & TIMER_CHCTL1_CH2MS) ){
            if(NULL != (timer_dev->timer_irq.channelx_capture_usercb)){
                timer_dev->timer_irq.channelx_capture_usercb(timer_dev);
            }
        }else{
            /* channel 2 compare interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_compare_usercb)){
                timer_dev->timer_irq.channelx_compare_usercb(timer_dev);
            }
            /* channel 2 pwm interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_pwm_usercb)){
                timer_dev->timer_irq.channelx_pwm_usercb(timer_dev);
            }
        }
        /* clear service channel */
        timer_dev->service_channel = TIMER_SERVICE_CHANNEL_NONE;
    }
    
    /* check whether the channel 3 capture/compare interrupt is set or not */
    if( SET == (timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CH3))){
        timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CH3);
        timer_dev->service_channel = TIMER_SERVICE_CHANNEL_3;
        /* channel 3 capture interrupt handle */
        if( 0 != (TIMER_CHCTL1(timer_dev->periph) & TIMER_CHCTL1_CH3MS) ){
            if(NULL != (timer_dev->timer_irq.channelx_capture_usercb)){
                timer_dev->timer_irq.channelx_capture_usercb(timer_dev);
            }
        }else{
            /* channel 3 compare interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_compare_usercb)){
                timer_dev->timer_irq.channelx_compare_usercb(timer_dev);
            }
            /* channel 3 pwm interrupt handle */
            if(NULL != (timer_dev->timer_irq.channelx_pwm_usercb)){
                timer_dev->timer_irq.channelx_pwm_usercb(timer_dev);
            }
        }
        /* clear service channel */
        timer_dev->service_channel = TIMER_SERVICE_CHANNEL_NONE;
    }
    /* check whether the commutation interrupt is set or not */
    if( SET == (timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_CMT))){
        timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_CMT);
        /* commutation interrupt handle */
        if(NULL != (timer_dev->timer_irq.commutation_usercb)){
            timer_dev->timer_irq.commutation_usercb(timer_dev);
        }
    }
    /* check whether the trigger interrupt is set or not */
    if( SET == (timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_TRG))){
        timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_TRG);
        /* trigger interrupt handle */
        if(NULL != (timer_dev->timer_irq.trigger_usercb)){
            timer_dev->timer_irq.trigger_usercb(timer_dev);
        }
    }
    /* check whether the break interrupt is set or not */
    if( SET == (timer_interrupt_flag_get(timer_dev->periph, TIMER_INT_FLAG_BRK))){
        timer_interrupt_flag_clear(timer_dev->periph, TIMER_INT_FLAG_BRK);
        /* break interrupt handle */
        if(NULL != (timer_dev->timer_irq.break_usercb)){
            timer_dev->timer_irq.break_usercb(timer_dev);
        }
    }
}

/*!
    \brief      write the current counter value, TIMERx_CNT
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  counter_value: the counter value,0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS,HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_write_counter_value(hal_timer_dev_struct *timer_dev, uint32_t counter_value)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(counter_value > UINT16_T_MAX){
        HAL_DEBUGE("parameter [counter_value] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure TIMER counter register value */
    TIMER_CNT(timer_dev->periph) = (uint32_t)counter_value;
    return HAL_ERR_NONE;
}

/*!
    \brief      read the current counter value, TIMERx_CNT
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the current counter value
*/
uint32_t hal_timer_read_counter_value(hal_timer_dev_struct *timer_dev)
{
    uint32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* get TIMER counter register value */
    ret_val = TIMER_CNT(timer_dev->periph);
    return ret_val;
}

/*!
    \brief      write counter auto reload value, TIMERx_CAR
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  autoreload: counter auto reload value,0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS,HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_write_autoreload_value(hal_timer_dev_struct *timer_dev, uint32_t autoreload)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(autoreload > UINT16_T_MAX){
        HAL_DEBUGE("parameter [autoreload] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    TIMER_CAR(timer_dev->periph) = (uint32_t)autoreload;
    return HAL_ERR_NONE;
}

/*!
    \brief      read counter auto reload value, TIMERx_CAR
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the counter auto reload value
*/
uint32_t hal_timer_read_autoreload_value(hal_timer_dev_struct *timer_dev)
{
    uint32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* get TIMER counter register value */
    ret_val = TIMER_CAR(timer_dev->periph);
    return ret_val;
}

/*!
    \brief      write counter repetition value, TIMERx_CREP
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  repetition_value: counter repetition value,0~255
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS,HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_write_repetition_value(hal_timer_dev_struct *timer_dev, uint32_t repetition_value)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(repetition_value > UINT8_T_MAX){
        HAL_DEBUGE("parameter [repetition_value] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    TIMER_CREP(timer_dev->periph) = (uint32_t)repetition_value;
    return HAL_ERR_NONE;
}

/*!
    \brief      read counter repetition value, TIMERx_CREP
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the counter repetition value
*/
uint32_t hal_timer_read_repetition_value(hal_timer_dev_struct *timer_dev)
{
    uint32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* get TIMER counter register value */
    ret_val = TIMER_CREP(timer_dev->periph);
    return ret_val;
}

/*!
    \brief      write prescaler value of the counter clock, TIMERx_PSC
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  prescaler: prescaler value of the counter clock,0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS,HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_write_prescaler_value(hal_timer_dev_struct *timer_dev, uint32_t prescaler)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(prescaler > UINT16_T_MAX){
        HAL_DEBUGE("parameter [prescaler] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    TIMER_PSC(timer_dev->periph) = (uint32_t)prescaler;
    return HAL_ERR_NONE;
}

/*!
    \brief      read prescaler value of the counter clock, TIMERx_PSC
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the prescaler value of the counter clock
*/
uint32_t hal_timer_read_prescaler_value(hal_timer_dev_struct *timer_dev)
{
    uint32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* get TIMER counter register value */
    ret_val = TIMER_PSC(timer_dev->periph);
    return ret_val;
}

/*!
    \brief      write channely capture/compare value, TIMERx_CHyCV
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  channel_value: capture or compare value of channel,0~65535
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS,HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_write_channelx_value(hal_timer_dev_struct *timer_dev, uint32_t channel, uint32_t channel_value)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(channel_value > UINT16_T_MAX){
        HAL_DEBUGE("parameter [channel_value] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
    /* configure capture or compare value of channel 0 */
    case TIMER_CH_0:
        TIMER_CH0CV(timer_dev->periph) = (uint32_t)channel_value;
        break;
    /* configure capture or compare value of channel 1 */
    case TIMER_CH_1:
        TIMER_CH1CV(timer_dev->periph) = (uint32_t)channel_value;
        break;
    /* configure capture or compare value of channel 2 */
    case TIMER_CH_2:
        TIMER_CH2CV(timer_dev->periph) = (uint32_t)channel_value;
        break;
    /* configure capture or compare value of channel 3 */
    case TIMER_CH_3:
         TIMER_CH3CV(timer_dev->periph) = (uint32_t)channel_value;
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        return HAL_ERR_VAL;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      read channely capture/compare value, TIMERx_CHyCV
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     capture or compare value of channel
*/
uint32_t hal_timer_read_channelx_value(hal_timer_dev_struct *timer_dev, uint32_t channel)
{
    uint32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(channel){
    /* get capture or compare value of channel 0 */
    case TIMER_CH_0:
        ret_val = TIMER_CH0CV(timer_dev->periph);
        break;
    /* get capture or compare value of channel 1 */
    case TIMER_CH_1:
        ret_val = TIMER_CH1CV(timer_dev->periph);
        break;
    /* get capture or compare value of channel 2 */
    case TIMER_CH_2:
        ret_val = TIMER_CH2CV(timer_dev->periph);
        break;
    /* get capture or compare value of channel 3 */
    case TIMER_CH_3:
         ret_val = TIMER_CH3CV(timer_dev->periph);
        break;
    default:
        HAL_DEBUGE("parameter [channel] value is invalid");
        break;
    }
    return ret_val;
}

/*!
    \brief      DMA error callback TIMER all dma request
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _timer_dmaerror_callback(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *p_timer;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct*)dma;
    p_timer = (hal_timer_dev_struct*)(p_dma->p_periph);
    /* call DMA error handle */
    if(NULL != (p_timer->timer_irq.dmaerror_usercb)){
        p_timer->timer_irq.dmaerror_usercb(p_timer);
    }
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER update dma request
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _update_dmatc_callback(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *p_timer;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct*)dma;
    p_timer = (hal_timer_dev_struct*)(p_dma->p_periph);
    /* call DMA transmission complete(TC) handle for TIMER update dma request */
    if(NULL != (p_timer->timer_irq.dmatc_update_usercb)){
        p_timer->timer_irq.dmatc_update_usercb(p_timer);
    }
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER compare and pwm dma request
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _channel_compare_pwm_dmatc_callback(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *p_timer;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct*)dma;
    p_timer = (hal_timer_dev_struct*)(p_dma->p_periph);
    /* service channel assignment */
    if (dma == p_timer->hdma[TIM_DMA_ID_CH0]){
        p_timer->service_channel = TIMER_SERVICE_CHANNEL_0;
    }else if (dma == p_timer->hdma[TIM_DMA_ID_CH1]){
        p_timer->service_channel = TIMER_SERVICE_CHANNEL_1;
    }else if (dma == p_timer->hdma[TIM_DMA_ID_CH2]){
        p_timer->service_channel = TIMER_SERVICE_CHANNEL_2;
    }else if (dma == p_timer->hdma[TIM_DMA_ID_CH3]){
        p_timer->service_channel = TIMER_SERVICE_CHANNEL_3;
    }
    /* call DMA transmission complete(TC) handle for TIMER compare and pwm dma request */
    if(NULL != (p_timer->timer_irq.dmatc_chx_cmppwm_usercb)){
        p_timer->timer_irq.dmatc_chx_cmppwm_usercb(p_timer);
    }
    /* clear service channel */
    p_timer->service_channel = TIMER_SERVICE_CHANNEL_NONE;
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER channel input capture dma request
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _channel_capture_dmatc_callback(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *p_timer;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct*)dma;
    p_timer = (hal_timer_dev_struct*)(p_dma->p_periph);
    /* service channel assignment */
    if (dma == p_timer->hdma[TIM_DMA_ID_CH0]){
        p_timer->service_channel = TIMER_SERVICE_CHANNEL_0;
    }else if (dma == p_timer->hdma[TIM_DMA_ID_CH1]){
        p_timer->service_channel = TIMER_SERVICE_CHANNEL_1;
    }else if (dma == p_timer->hdma[TIM_DMA_ID_CH2]){
        p_timer->service_channel = TIMER_SERVICE_CHANNEL_2;
    }else if (dma == p_timer->hdma[TIM_DMA_ID_CH3]){
        p_timer->service_channel = TIMER_SERVICE_CHANNEL_3;
    }
    /* call DMA transmission complete(TC) handle for TIMER input capture dma request */
    if(NULL != (p_timer->timer_irq.dmatc_chx_capture_usercb)){
        p_timer->timer_irq.dmatc_chx_capture_usercb(p_timer);
    }
    p_timer->service_channel = TIMER_SERVICE_CHANNEL_NONE;
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER commutation dma request
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _commutation_dmatc_callback(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *p_timer;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct*)dma;
    p_timer = (hal_timer_dev_struct*)(p_dma->p_periph);
    /* call DMA transmission complete(TC) handle for TIMER commutation dma request */
    if(NULL != (p_timer->timer_irq.dmatc_commutation_usercb)){
        p_timer->timer_irq.dmatc_commutation_usercb(p_timer);
    }
}

/*!
    \brief      DMA transmission complete(TC) callback for TIMER trigger dma request
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _trigger_dmatc_callback(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_timer_dev_struct *p_timer;
    /* parameter assignment */
    p_dma = (hal_dma_dev_struct*)dma;
    p_timer = (hal_timer_dev_struct*)(p_dma->p_periph);
    /* call DMA transmission complete(TC) handle for TIMER trigger dma request */
    if(NULL != (p_timer->timer_irq.dmatc_trigger_usercb)){
        p_timer->timer_irq.dmatc_trigger_usercb(p_timer);
    }
}

/*!
    \brief      disable TIMER
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
static int32_t _timer_disable(hal_timer_dev_struct *timer_dev)
{
    uint32_t chctl2;
    chctl2 = TIMER_CHCTL2(timer_dev->periph);
    /* determine whether channel is disabled */
    if( 0 == (chctl2 & TIMER_CHX_EN_MASK) ){
        if( 0 == (chctl2 & TIMER_CHNX_EN_MASK) ){
            /* complementay channel is disabled */
            timer_disable(timer_dev->periph);
            return HAL_ERR_NONE;
        }else{
            /* complementay channel is not disabled */
            return HAL_ERR_NO_SUPPORT;
        }
    }else{
        /* channel is not disabled */
        return HAL_ERR_NO_SUPPORT;
    }
}

/*!
    \brief      enable or disable TIMER primary output
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  state: EBANLE, DISABLE
    \param[out] none
    \retval     none
*/
static void _timer_primary_output_config(hal_timer_dev_struct *timer_dev , ControlStatus state)
{
    uint32_t chctl2;
    chctl2 = TIMER_CHCTL2(timer_dev->periph);
    /* TIMER which has primary output enable(POE) bit */
    if((TIMER0 == timer_dev->periph) || (TIMER14 == timer_dev->periph)
        || (TIMER15 == timer_dev->periph) || (TIMER16 == timer_dev->periph)){
        if(ENABLE == state){
            /* enable TIMER primary output */
            TIMER_CCHP(timer_dev->periph) |= (uint32_t)TIMER_CCHP_POEN;
        }else{
            /* disable TIMER primary output */
            if( 0 == (chctl2 & TIMER_CHX_EN_MASK) ){
                if( 0 == (chctl2 & TIMER_CHNX_EN_MASK) ){
                    TIMER_CCHP(timer_dev->periph) &= (~(uint32_t)TIMER_CCHP_POEN);
                }
            }
        }
    }
}
