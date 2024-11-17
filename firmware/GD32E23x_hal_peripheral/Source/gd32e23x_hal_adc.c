/*!
    \file    gd32e23x_hal_adc.c
    \brief   ADC driver
    
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

/* delaytime for ADC enable and disable, in milliseconds */
#define ADC_ENABLE_DELAYTIME                    ((uint32_t) 2U)
#define ADC_DISABLE_DELAYTIME                   ((uint32_t) 2U)
/* delaytime for temperature sensor stabilization time, in microseconds */
#define ADC_TEMPSENSOR_DELAYTIME                ((uint32_t) 10U)

/* ADC conversion cycles(sampling time + conversion time of 12.5 ADC clock cycles, with resolution 12 bits) */
#define ADC_CONVERCYCLES_SAMT_1POINT5           ((uint32_t) 14U)
#define ADC_CONVERCYCLES_SAMT_7POINT5           ((uint32_t) 20U)
#define ADC_CONVERCYCLES_SAMT_13POINT5          ((uint32_t) 26U)
#define ADC_CONVERCYCLES_SAMT_28POINT5          ((uint32_t) 41U)
#define ADC_CONVERCYCLES_SAMT_41POINT5          ((uint32_t) 54U)
#define ADC_CONVERCYCLES_SAMT_55POINT5          ((uint32_t) 68U)
#define ADC_CONVERCYCLES_SAMT_71POINT5          ((uint32_t) 84U)
#define ADC_CONVERCYCLES_SAMT_239POINT5         ((uint32_t)252U)

/* ADC sampling time register1 mask */
#define ADC_SAMTALLCHS_1MASK2                   ((uint32_t)0x24924924U)
#define ADC_SAMTALLCHS_1MASK1                   ((uint32_t)0x12492492U)
#define ADC_SAMTALLCHS_1MASK0                   ((uint32_t)0x09249249U)
/* ADC sampling time register0 mask */
#define ADC_SAMTALLCHS_0MASK2                   ((uint32_t)0x00900000U)
#define ADC_SAMTALLCHS_0MASK1                   ((uint32_t)0x00480000U)
#define ADC_SAMTALLCHS_0MASK0                   ((uint32_t)0x00240000U)

/* enable ADC */
/* enable the ADC */
static int32_t _adc_enable(hal_adc_dev_struct *adc);
/*disable the ADC */
static int32_t _adc_disable(hal_adc_dev_struct *adc);
/* get the ADC enable state */
static FlagStatus _adc_enable_state_get(void);

/* DMA callback function */
/* ADC DMA transmission complete callback */
static void _adc_dma_transfer_complete(void *dma);
/* ADC DMA error callback */
static void _adc_dma_error(void *dma);

/* get ADC state and error */
/* set ADC state */
static void _adc_state_set(hal_adc_dev_struct *adc, hal_adc_state_enum adc_state);
/* clear ADC state */
static void _adc_state_clear(hal_adc_dev_struct *adc, hal_adc_state_enum adc_state);
/* set ADC error */
static void _adc_error_set(hal_adc_dev_struct *adc, hal_adc_error_enum adc_error);
/* clear ADC error */
static void _adc_error_clear(hal_adc_dev_struct *adc, hal_adc_error_enum adc_error);

/*!
    \brief      ADC init struct deinitialize
    \param[in]  hal_struct_type: the type of the structure
    \param[in]  p_struct: the pointer of the structure
    \param[out] none
    \retval     none
*/
void hal_adc_struct_init(hal_adc_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct){
        HAL_DEBUGE("pointer [*p_struct] value is invalid");
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    switch(hal_struct_type){
    case HAL_ADC_INIT_STRUCT:
        /* initialize ADC initialization structure with the default values */
        ((hal_adc_init_struct*)p_struct)->resolution_select                        = ADC_RESOLUTION_12B;
        ((hal_adc_init_struct*)p_struct)->data_alignment                           = ADC_DATAALIGN_RIGHT;
        ((hal_adc_init_struct*)p_struct)->scan_mode                                = DISABLE;
        ((hal_adc_init_struct*)p_struct)->oversample_config.oversample_mode        = DISABLE;
        ((hal_adc_init_struct*)p_struct)->oversample_config.oversample_ratio       = ADC_OVERSAMPLE_RATIO_MUL2;
        ((hal_adc_init_struct*)p_struct)->oversample_config.oversample_shift       = ADC_OVERSAMPLE_SHIFT_NONE;
        ((hal_adc_init_struct*)p_struct)->oversample_config.oversample_triggermode = ADC_OVERSAMPLE_ALL_CONVERT;
        break;

    case HAL_ADC_OVERSAMPLE_STRUCT:
        /* initialize ADC oversampling structure with the default values */
        ((hal_adc_oversample_struct*)p_struct)->oversample_mode        = DISABLE;
        ((hal_adc_oversample_struct*)p_struct)->oversample_ratio       = ADC_OVERSAMPLE_RATIO_MUL2;
        ((hal_adc_oversample_struct*)p_struct)->oversample_shift       = ADC_OVERSAMPLE_SHIFT_NONE;
        ((hal_adc_oversample_struct*)p_struct)->oversample_triggermode = ADC_OVERSAMPLE_ALL_CONVERT;
        break;
    
    case HAL_ADC_REGULARCH_INIT_STRUCT:
        /* initialize ADC regular channel initialization structure with the default values */
        ((hal_adc_regularch_init_struct*)p_struct)->length                       = 1U;
        ((hal_adc_regularch_init_struct*)p_struct)->exttrigger_select            = ADC_EXTTRIG_REGULAR_NONE;
        ((hal_adc_regularch_init_struct*)p_struct)->continuous_mode              = DISABLE;
        ((hal_adc_regularch_init_struct*)p_struct)->discontinuous_mode           = DISABLE;
        ((hal_adc_regularch_init_struct*)p_struct)->discontinuous_channel_length = 0U;
        break;
    
    case HAL_ADC_REGULARCH_CONFIG_STRUCT:
        /* initialize ADC regular channel configuration structure with the default values */
        ((hal_adc_regularch_config_struct*)p_struct)->regular_channel  = 0U;
        ((hal_adc_regularch_config_struct*)p_struct)->regular_sequence = ADC_REGULAR_SEQUENCE_0;
        ((hal_adc_regularch_config_struct*)p_struct)->sample_time      = ADC_SAMPLETIME_7POINT5; 
        break;

    case HAL_ADC_INSERTEDCH_INIT_STRUCT:
        /* initialize ADC inserted channel initialization structure with the default values */
        ((hal_adc_insertedch_init_struct*)p_struct)->length             = 0U;
        ((hal_adc_insertedch_init_struct*)p_struct)->exttrigger_select  = ADC_EXTTRIG_REGULAR_NONE;
        ((hal_adc_insertedch_init_struct*)p_struct)->auto_convert       = DISABLE;
        ((hal_adc_insertedch_init_struct*)p_struct)->discontinuous_mode = DISABLE;
        break;
    
    case HAL_ADC_INSERTEDCH_CONFIG_STRUCT:
        /* initialize ADC inserted channel configuration structure with the default values */
        ((hal_adc_insertedch_config_struct*)p_struct)->inserted_channel  = 0U;
        ((hal_adc_insertedch_config_struct*)p_struct)->inserted_sequence = ADC_INSERTED_SEQUENCE_0;
        ((hal_adc_insertedch_config_struct*)p_struct)->sample_time       = ADC_SAMPLETIME_7POINT5;
        ((hal_adc_insertedch_config_struct*)p_struct)->inserted_offset   = 0U;
        break;
    
    case HAL_ADC_IRQ_STRUCT:
        /* initialize ADC device interrupt callback function pointer structure with the default values */
        ((hal_adc_irq_struct*)p_struct)->adc_watchdog_handle = NULL;
        ((hal_adc_irq_struct*)p_struct)->adc_eoc_handle      = NULL;
        ((hal_adc_irq_struct*)p_struct)->adc_eoic_handle     = NULL; 
        break;

    case HAL_ADC_DMA_HANDLE_CB_STRUCT:
        /* initialize ADC DMA callback function pointer structure with the default values */
        ((hal_adc_dma_handle_cb_struct*)p_struct)->transcom_handle = NULL;
        ((hal_adc_dma_handle_cb_struct*)p_struct)->error_handle    = NULL;
        break;
    
    case HAL_ADC_WATCHDOG_STRUCT:
        /* initialize ADC watchdog configuration structure with the default values */
        ((hal_adc_watchdog_struct*)p_struct)->watchdog_mode  = ADC_WATCHDOG_NONE;
        ((hal_adc_watchdog_struct*)p_struct)->single_channel = 0U;
        ((hal_adc_watchdog_struct*)p_struct)->low_threshold  = 0U;
        ((hal_adc_watchdog_struct*)p_struct)->high_threshold = 0U;
        break;
    
    case HAL_ADC_DEV_STRUCT:
        /* initialize ADC device information structure with the default values */
        ((hal_adc_dev_struct*)p_struct)->adc_irq.adc_eoc_handle      = NULL;
        ((hal_adc_dev_struct*)p_struct)->adc_irq.adc_eoic_handle     = NULL;
        ((hal_adc_dev_struct*)p_struct)->adc_irq.adc_watchdog_handle = NULL;
        ((hal_adc_dev_struct*)p_struct)->p_dma_adc                   = NULL;
        ((hal_adc_dev_struct*)p_struct)->adc_dma.transcom_handle     = NULL;
        ((hal_adc_dev_struct*)p_struct)->adc_dma.error_handle        = NULL;
        ((hal_adc_dev_struct*)p_struct)->error_state                 = ADC_ERROR_NONE;
        ((hal_adc_dev_struct*)p_struct)->state                       = ADC_STATE_RESET;
        break;

    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize ADC device structure and init structure
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_deinit(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    _adc_state_set(adc, ADC_STATE_BUSY_SYSTEM);
    
    /* if ADC is disabled */
    if(HAL_ERR_NONE == _adc_disable(adc)){
        /* deinit ADC */
        adc_deinit();
        adc->error_state = ADC_ERROR_NONE;
        adc->state = ADC_STATE_RESET;
    }else{
        return HAL_ERR_HARDWARE;
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize ADC
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_init: the pointer of ADC init structure
                  resolution_select: ADC_RESOLUTION_12B, ADC_RESOLUTION_10B, ADC_RESOLUTION_8B, ADC_RESOLUTION_6B
                  data_alignment: ADC_DATAALIGN_RIGHT, ADC_DATAALIGN_LEFT
                  scan_mode: ENABLE, DISABLE
                  oversample_config:
                    the argument could be selected from structure <hal_adc_oversample_struct>
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_init(hal_adc_dev_struct *adc, hal_adc_init_struct *p_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_init)){
        HAL_DEBUGE("pointer [adc] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(ADC_STATE_RESET == (hal_adc_state_get(adc))){
        adc->error_state = ADC_ERROR_NONE;
    }
    
    if((HAL_ERR_NONE == _adc_disable(adc)) && (RESET == ((hal_adc_error_get(adc)) & ADC_ERROR_SYSTEM))){
        /* set ADC state  */
        _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
        _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc, ADC_STATE_BUSY_SYSTEM);

        /* init ADC */
        /* 1.ADC data resolution select */
        /* ADC data resolution select can be updated only when ADC is disabled*/
        if(RESET == _adc_enable_state_get()){
            adc_resolution_config(p_init->resolution_select);
        }
        /* 2.ADC data alignment config */
        adc_data_alignment_config(p_init->data_alignment);

        /* 3.ADC scan mode config */
        adc_special_function_config(ADC_SCAN_MODE, p_init->scan_mode);

        /* 4.ADC oversampling mode config */
        if(ENABLE == p_init->oversample_config.oversample_mode){
            adc_oversample_mode_config(p_init->oversample_config.oversample_triggermode, p_init->oversample_config.oversample_shift,
                                       p_init->oversample_config.oversample_ratio);
            /* enable ADC oversample mode */
            adc_oversample_mode_enable();
        }else{
            if(SET == __HAL_ADC_GET_OVERSAMPLE_ENABLE){
                /* disable ADC oversample mode */
                adc_oversample_mode_disable();
            }
        }
    }else{
        return HAL_ERR_HARDWARE;
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      ADC calibration
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_calibration(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is disabled */
    if(HAL_ERR_NONE == _adc_disable(adc)){
        /* set ADC state  */
        _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
        _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc, ADC_STATE_BUSY_SYSTEM);

        /* enable the ADC */
        _adc_enable(adc);
        /* ADC calibration and reset calibration */
        adc_calibration_enable();
        
        /* set ADC state  */
        _adc_state_clear(adc, ADC_STATE_BUSY_SYSTEM);
        _adc_state_set(adc, ADC_STATE_READY);
    }else{
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize ADC regular channel
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rinit: the pointer of ADC regular group init structure
                  length: 1-16
                  exttrigger_select: ADC_EXTTRIG_REGULAR_T0_CH0, ADC_EXTTRIG_REGULAR_T0_CH1, ADC_EXTTRIG_REGULAR_T0_CH2
                                     ADC_EXTTRIG_REGULAR_T2_TRGO, ADC_EXTTRIG_REGULAR_T14_CH0, ADC_EXTTRIG_REGULAR_EXTI_11,
                                     ADC_EXTTRIG_REGULAR_NONE
                  continuous_mode: ENABLE, DISABLE
                  discontinuous_mode: ENABLE, DISABLE
                  discontinuous_channel_length: 1-8
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_regular_channel_init(hal_adc_dev_struct *adc, hal_adc_regularch_init_struct *p_rinit)
{
    uint32_t reg_temp;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_rinit)){
        HAL_DEBUGE("pointer [adc] or [p_rinit] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(ADC_STATE_RESET == (hal_adc_state_get(adc))){
        adc->error_state = ADC_ERROR_NONE;
    }

    if((HAL_ERR_NONE == _adc_disable(adc)) && (RESET == ((hal_adc_error_get(adc)) & ADC_ERROR_SYSTEM))){
        /* set ADC state  */
        _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
        _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc, ADC_STATE_BUSY_SYSTEM);

        /* init ADC regular channel */
        /* 1.ADC channel length config */
        adc_channel_length_config(ADC_REGULAR_CHANNEL, p_rinit->length);
        /* 2.ADC regular channel external trigger config */
        adc_external_trigger_source_config(ADC_REGULAR_CHANNEL, p_rinit->exttrigger_select); 

        /* 3.ADC continuous mode config */
        adc_special_function_config(ADC_CONTINUOUS_MODE, p_rinit->continuous_mode);
        /* 4.ADC discontinuous mode config */
        if(ENABLE == p_rinit->discontinuous_mode){
            if(DISABLE == p_rinit->continuous_mode){
                reg_temp = ADC_CTL0;
                /* clear DISRC bit and DISNUM bit */
                reg_temp &= ~((uint32_t)ADC_CTL0_DISRC);
                reg_temp &= ~((uint32_t)ADC_CTL0_DISNUM);
                reg_temp |= CTL0_DISNUM(((uint32_t)(p_rinit->discontinuous_channel_length) - 1U));
                reg_temp |= (uint32_t)ADC_CTL0_DISRC;
                ADC_CTL0 = reg_temp;
            }else{
                ADC_CTL0 &= ~((uint32_t)ADC_CTL0_DISRC);
                _adc_error_set(adc, ADC_ERROR_CONFIG);
                _adc_error_set(adc, ADC_ERROR_SYSTEM);
            }
        }
    }else{
        return HAL_ERR_HARDWARE;
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      configure ADC regular channel
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rchannel: the pointer of ADC regular channel configuration structure
                  regular_channel: ADC_CHANNEL_x(x=0..9,16,17)
                  regular_sequence:
                    the argument could be selected from enumeration <hal_adc_regularch_sequence_enum>
                  sample_time: ADC_SAMPLETIME_1POINT5, ADC_SAMPLETIME_7POINT5, ADC_SAMPLETIME_13POINT5, ADC_SAMPLETIME_28POINT5,
                                ADC_SAMPLETIME_41POINT5, ADC_SAMPLETIME_55POINT5, ADC_SAMPLETIME_71POINT5, ADC_SAMPLETIME_239POINT5
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_regular_channel_config(hal_adc_dev_struct *adc, hal_adc_regularch_config_struct *p_rchannel)
{
    __IO uint32_t wait_loop = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_rchannel) ){
        HAL_DEBUGE("pointer [adc] or [p_rchannel] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* configure ADC regular channel */
    adc_regular_channel_config(p_rchannel->regular_sequence, p_rchannel->regular_channel, p_rchannel->sample_time);
    
    /* enable temperature sensor and VREFINT measurement */
    if((ADC_CHANNEL_TEMPSENSOR == p_rchannel->regular_channel) || (ADC_CHANNEL_VREFINT == p_rchannel->regular_channel)){
        if(RESET == __HAL_ADC_GET_TEMPVREF_ENABLE){
            /* enable the temperature sensor and Vrefint channel */
            ADC_CTL1 |= ADC_CTL1_TSVREN;
            if(ADC_CHANNEL_TEMPSENSOR == p_rchannel->regular_channel){
                /* compute number of CPU cycles to wait for */
                wait_loop = (ADC_TEMPSENSOR_DELAYTIME * (SystemCoreClock / 1000000));
                /* delay for temperature sensor stabilization time */
                while(wait_loop != 0){
                    wait_loop--;
                }
            } 
        }
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      start ADC module function
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_start(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
        
    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc)){
        _adc_state_clear(adc, ADC_STATE_READY);
        _adc_state_clear(adc, ADC_STATE_REGULAR_EOC);
        _adc_state_set(adc, ADC_STATE_REGULAR_BUSY);

        /* if ADC configured in automatic inserted mode */
        if(SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV){
            _adc_state_clear(adc, ADC_STATE_INSERTED_EOC);
            _adc_state_set(adc, ADC_STATE_INSERTED_BUSY);
        }
        
        if(SET == ((hal_adc_state_get(adc)) & ADC_STATE_INSERTED_BUSY)){
            _adc_error_clear(adc, ADC_ERROR_DMA);
        }else{
            adc->error_state = ADC_ERROR_NONE;
        }
        
        /* clear ADC flag */
        adc_flag_clear(ADC_FLAG_EOC);

        /* enable conversion of regular group */
        if(ADC_EXTTRIG_REGULAR_NONE == __HAL_ADC_GET_REGULARCH_EXTTRIGGER){
            /* enable software trigger */
            adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
            adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
        }else{
            /* enable external trigger */
            adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
        }
    }else{
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop ADC module function
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_stop(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is disabled */
    if(HAL_ERR_NONE == _adc_disable(adc)){
        _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
        _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc, ADC_STATE_READY);
    }else{
        return HAL_ERR_HARDWARE;
    }
    
    return HAL_ERR_NONE;  
}

/*!
    \brief      polling for ADC regular group conversion, this function is just for single conversion 
                (scan mode disabled or scan mode enabled with the length of regular channel is 1) 
    \param[in]  adc: ADC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: time out duration in milliscond
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_regular_conversion_poll(hal_adc_dev_struct *adc, uint32_t timeout_ms)
{
    uint32_t tick_start = 0;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* if ADC configured in DMA mode */
    if(SET == __HAL_ADC_GET_DMA_MODE){
        _adc_error_set(adc, ADC_ERROR_CONFIG);
        return HAL_ERR_NO_SUPPORT;
    }

    /* set timeout */
    tick_start = hal_basetick_count_get();

    /* if single conversion: scan mode disabled or enabled with length = 1 */
    if((RESET == __HAL_ADC_GET_SCAN_MODE) && (RESET == __HAL_ADC_GET_REGULARCH_LENGTH)){
        /* wait until end of conversion flag is raised */
        while(RESET == adc_flag_get(ADC_FLAG_EOC)){
            /* check for the timeout */
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    _adc_state_set(adc, ADC_STATE_TIMEOUT);
                    /* when timeout occurs, output timeout warning message */
                    HAL_DEBUGW("adc regular conversion poll timeout");
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }else{
        return HAL_ERR_NO_SUPPORT;
    }

    /* clear ADC EOC flag and STRC flag */
    adc_flag_clear(ADC_FLAG_EOC);
    adc_flag_clear(ADC_FLAG_STRC);
    
    /* set ADC states */
    _adc_state_set(adc, ADC_STATE_REGULAR_EOC);
    if((ADC_EXTTRIG_REGULAR_NONE == __HAL_ADC_GET_REGULARCH_EXTTRIGGER) && (DISABLE == __HAL_ADC_GET_CONTINUOUS_MODE)){
        _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
        if(RESET == ((hal_adc_state_get(adc)) & ADC_STATE_INSERTED_BUSY)){
            _adc_state_set(adc, ADC_STATE_READY);
       }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC regular group software trigger
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
/*  */
void hal_adc_regular_software_trigger_enable(hal_adc_dev_struct *adc)
{
    adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
}

/*!
    \brief      start ADC EOC interrupt
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: EOC interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_start_interrupt(hal_adc_dev_struct *adc, hal_adc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_irq)){
        HAL_DEBUGE("pointer [adc] or [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc)){
        /* set ADC state */
        _adc_state_clear(adc, ADC_STATE_READY);
        _adc_state_clear(adc, ADC_STATE_REGULAR_EOC);
        _adc_state_set(adc, ADC_STATE_REGULAR_BUSY);

        /* if ADC configured in automatic inserted mode */
        if(SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV){
            _adc_state_clear(adc, ADC_STATE_INSERTED_EOC);
            _adc_state_set(adc, ADC_STATE_INSERTED_BUSY);
        }
        /* check if an inserted conversion is ongoing */
        if(SET == ((hal_adc_state_get(adc)) & ADC_STATE_INSERTED_BUSY)){
            _adc_error_clear(adc, ADC_ERROR_DMA);
        }else{
            adc->error_state = ADC_ERROR_NONE;
        }

        adc->adc_irq.adc_eoc_handle = p_irq->adc_eoc_handle;
        
        /* clear ADC interrupt flag */
        adc_interrupt_flag_clear(ADC_INT_FLAG_EOC);
        /* enable ADC end of group conversion flag */
        adc_interrupt_enable(ADC_INT_EOC);
        /* enable the conversion of regular group */
        if(ADC_EXTTRIG_REGULAR_NONE == __HAL_ADC_GET_REGULARCH_EXTTRIGGER){
            /* enable software trigger */
            adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
            adc_software_trigger_enable(ADC_REGULAR_CHANNEL);
        }else{
            /* enable external trigger */
            adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
        }
    }else{
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop ADC EOC interrupt
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_stop_interrupt(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is disabled */
    if(HAL_ERR_NONE == _adc_disable(adc)){
        /* end of group conversion interrupt */
        adc_interrupt_disable(ADC_INT_EOC);
        adc->adc_irq.adc_eoc_handle = NULL;
        
        /* set ADC state */
        _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
        _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc, ADC_STATE_READY);
    }else{
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      start ADC EOC DMA request 
    \param[in]  adc: ADC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  pdata: the source memory buffer address
    \param[in]  length: the number of data to be transferred from source to destination
    \param[in]  dmacb: ADC DMA transfer complete interrupt callback function structure
                  transcom_handle: DMA transfer complete interrupt handler 
                  error_handle: ADC DMA error interrupt handler
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_start_dma(hal_adc_dev_struct *adc, uint32_t* pdata, uint32_t length, hal_adc_dma_handle_cb_struct *dmacb)
{
    hal_dma_irq_struct dma_irq;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == pdata) || (NULL == dmacb)){
        HAL_DEBUGE("pointer [adc] or [pdata] or [dmacb] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    if(0U == length){
        HAL_DEBUGE("parameter [length] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    
    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc)){
        /* set ADC state */
        _adc_state_clear(adc, ADC_STATE_READY);
        _adc_state_clear(adc, ADC_STATE_REGULAR_EOC);
        _adc_state_set(adc, ADC_STATE_REGULAR_BUSY);

        /* if conversions on group regular are also triggering group inserted */
        if(SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV){
            _adc_state_clear(adc, ADC_STATE_INSERTED_EOC);
            _adc_state_set(adc, ADC_STATE_INSERTED_BUSY);
        }
        /* check if an inserted conversion is ongoing */
        if(SET == ((hal_adc_state_get(adc)) & ADC_STATE_INSERTED_BUSY)){
            _adc_error_clear(adc, ADC_ERROR_DMA);
        }else{
            adc->error_state = ADC_ERROR_NONE;
        }

        /* clear ADC flag */
        adc_flag_clear(ADC_FLAG_EOC);
        
        adc->adc_dma.transcom_handle = dmacb->transcom_handle;
        adc->adc_dma.error_handle    = dmacb->error_handle;

        /* set the DMA transfer complete callback */
        dma_irq.full_finish_handle = _adc_dma_transfer_complete;
        dma_irq.error_handle       = _adc_dma_error;
        if(NULL != adc->p_dma_adc->dma_irq.half_finish_handle){
            dma_irq.half_finish_handle = adc->p_dma_adc->dma_irq.half_finish_handle;
        }

        /* enable ADC DMA mode */
        adc_dma_mode_enable();
        /* start the DMA channel */
        hal_dma_start_interrupt(adc->p_dma_adc, (uint32_t)&ADC_RDATA, (uint32_t)pdata, length, &dma_irq);

        /* enable the conversion of regular group */
        if(ADC_EXTTRIG_REGULAR_NONE == __HAL_ADC_GET_REGULARCH_EXTTRIGGER){
            /* enable software trigger */
            adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
            adc_software_trigger_enable(ADC_REGULAR_CHANNEL);

        }else{
            /* enable external trigger */
            adc_external_trigger_config(ADC_REGULAR_CHANNEL, ENABLE);
        }
    }else{
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop the conversion of regular group, disable ADC DMA mode and disable ADC
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_stop_dma(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_disable(adc)){
        /* disable ADC DMA mode */
        adc_dma_mode_disable();
        /* disable the DMA channel */
        hal_dma_stop(adc->p_dma_adc);
        
        /* reset DMA callback function interrupt handler */
        adc->adc_dma.transcom_handle = NULL;
        adc->adc_dma.error_handle = NULL;
        
        /* set ADC state */
        _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
        _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
        _adc_state_set(adc, ADC_STATE_READY);
    }else{
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize ADC inserted channel
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_iinit: the pointer of ADC inserted group init structure
                  length: 1-4
                  exttrigger_select: ADC_EXTTRIG_INSERTED_T0_TRGO, ADC_EXTTRIG_INSERTED_T0_CH3, ADC_EXTTRIG_INSERTED_T2_CH3,
                                              ADC_EXTTRIG_INSERTED_T14_TRGO, ADC_EXTTRIG_INSERTED_EXTI_15, ADC_EXTTRIG_INSERTED_NONE,
                  auto_convert: ENABLE, DISABLE
                  discontinuous_mode: ENABLE, DISABLE
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_inserted_channel_init(hal_adc_dev_struct *adc, hal_adc_insertedch_init_struct *p_iinit)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_iinit) ){
        HAL_DEBUGE("pointer [adc] or [p_iinit] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* ADC inserted channel initialize */ 
    /* ADC inserted channel length config */
    adc_channel_length_config(ADC_INSERTED_CHANNEL, p_iinit->length);
    
    /* if ADC is disabled */
    if(RESET == _adc_enable_state_get()){
        /* ADC inserted channel external trigger config */
        adc_external_trigger_source_config(ADC_INSERTED_CHANNEL, p_iinit->exttrigger_select);
        /* enable ADC */
        ADC_CTL1 |= (uint32_t)ADC_CTL1_ADCON;
    }

    /* ADC inserted channel group convert automatically config */
    if(ENABLE == p_iinit->auto_convert){
        if(ADC_EXTTRIG_INSERTED_NONE == p_iinit->exttrigger_select){
            adc_special_function_config(ADC_INSERTED_CHANNEL_AUTO, p_iinit->auto_convert);
        }else{
            _adc_error_set(adc, ADC_ERROR_CONFIG);
            return HAL_ERR_NO_SUPPORT;  
        }
    }
    
    /* ADC discontinuous mode config */
    if(ENABLE == p_iinit->discontinuous_mode){
        /* inserted discontinuous can be enabled only if auto-inserted mode is disabled.   */
        if(DISABLE == p_iinit->auto_convert){
            ADC_CTL0 |= (uint32_t)ADC_CTL0_DISIC;
        }else{
            _adc_error_set(adc, ADC_ERROR_CONFIG);
            return HAL_ERR_NO_SUPPORT;   
        }
    }else{
        ADC_CTL0 &= ~((uint32_t)ADC_CTL0_DISIC);
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      configure ADC inserted channel
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_ichannel: the pointer of ADC inserted channel configuration structure
                  inserted_channel: ADC_CHANNEL_x(x=0..9,16,17)
                  inserted_sequence:
                    the argument could be selected from enumeration <hal_adc_insertedch_sequence_enum>
                  sample_time: ADC_SAMPLETIME_1POINT5, ADC_SAMPLETIME_7POINT5, ADC_SAMPLETIME_13POINT5, ADC_SAMPLETIME_28POINT5,
                                ADC_SAMPLETIME_41POINT5, ADC_SAMPLETIME_55POINT5, ADC_SAMPLETIME_71POINT5, ADC_SAMPLETIME_239POINT5
                  inserted_offset: the offset data(0-0x0FFF)
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_inserted_channel_config(hal_adc_dev_struct *adc, hal_adc_insertedch_config_struct *p_ichannel)
{
    uint32_t reg_temp;
    uint32_t length_temp;
    __IO uint32_t wait_loop = 0;
    uint8_t inserted_length;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_ichannel) ){
        HAL_DEBUGE("pointer [adc] or [p_ichannel] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* ADC inserted channel length */
    length_temp = ((ADC_ISQ & ADC_ISQ_IL) >> 20U) + 1U;
    
    if(RESET == __HAL_ADC_GET_SCAN_MODE){
        if(ADC_INSERTED_SEQUENCE_0 == p_ichannel->inserted_sequence){
           /* ADC inserted channel config */ 
           adc_inserted_channel_config(ADC_INSERTED_SEQUENCE_0, p_ichannel->inserted_channel, p_ichannel->sample_time);
        }else{
            _adc_error_set(adc, ADC_ERROR_CONFIG);
            return HAL_ERR_NO_SUPPORT;   
        }
    }else{
        if(p_ichannel->inserted_sequence <= (length_temp -1U)){
            /* ADC inserted channel config */
           adc_inserted_channel_config(p_ichannel->inserted_sequence, p_ichannel->inserted_channel, p_ichannel->sample_time);
        }else{
            /* clear parameters */
            ADC_ISQ &= ~((uint32_t)ADC_ISQ_IL);

            inserted_length = (uint8_t)(length_temp - 1U);
            reg_temp = ADC_ISQ;
            reg_temp &= ~((uint32_t)(ADC_ISQ_ISQN << (15U - (inserted_length - (p_ichannel->inserted_sequence))*5U)));
            ADC_ISQ = reg_temp;
        }
    }

    /* ADC inserted channel offset config */
    adc_inserted_channel_offset_config(p_ichannel->inserted_sequence, p_ichannel->inserted_offset);
    
    /* enable temperature sensor and VREFINT measurement */
    if((ADC_CHANNEL_TEMPSENSOR == p_ichannel->inserted_channel) || (ADC_CHANNEL_VREFINT == p_ichannel->inserted_channel)){
        if(RESET == __HAL_ADC_GET_TEMPVREF_ENABLE){
            ADC_CTL1 |= ADC_CTL1_TSVREN;
            if(ADC_CHANNEL_TEMPSENSOR == p_ichannel->inserted_channel){
                /* delay for temperature sensor stabilization time */
                /* compute number of CPU cycles to wait for */
                wait_loop = (ADC_TEMPSENSOR_DELAYTIME * (SystemCoreClock / 1000000));
                while(wait_loop != 0){
                    wait_loop--;
                }
            } 
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC and start the conversion of inserted group
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_inserted_start(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc)){
        /* set ADC state  */
        _adc_state_clear(adc, ADC_STATE_READY);
        _adc_state_clear(adc, ADC_STATE_INSERTED_EOC);
        _adc_state_set(adc, ADC_STATE_INSERTED_BUSY);
        /* check if a regular conversion is ongoing */
        if(RESET == ((hal_adc_state_get(adc)) & ADC_STATE_REGULAR_BUSY)){
            adc->error_state = ADC_ERROR_NONE;
        }
        
        /* clear ADC flag */
        adc_flag_clear(ADC_FLAG_EOIC);

        if(RESET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV){
            if(ADC_EXTTRIG_INSERTED_NONE == __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER){
                /* enable software trigger */
                adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
                adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
            }else{
                /* enable external trigger */
                adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
            }
        }
    }else{
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop the conversion of inserted group and disable ADC
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_inserted_stop(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* regular group is not busy and automatic convert is set */   
    if((RESET ==((hal_adc_state_get(adc)) & ADC_STATE_REGULAR_BUSY)) && (SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV)){
        /* if ADC is disabled */
        if(HAL_ERR_NONE == _adc_disable(adc)){
            /* set ADC state  */
            _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
            _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
            _adc_state_set(adc, ADC_STATE_READY);
        }
    }else{
        _adc_error_set(adc, ADC_ERROR_CONFIG);
        return HAL_ERR_NO_SUPPORT;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      polling for ADC inserted group conversion, this function is just for single conversion 
                (scan mode disabled or scan mode enabled with the length of inserted channel is 1).
    \param[in]  adc: ADC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: time out duration in milliscond
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_inserted_conversion_poll(hal_adc_dev_struct *adc, uint32_t timeout_ms)
{
    uint32_t tick_start = 0;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* set timeout */
    tick_start = hal_basetick_count_get();

    /* if single conversion: scan mode disabled or enabled with length = 1 */
    if((RESET == __HAL_ADC_GET_SCAN_MODE) &&(RESET == __HAL_ADC_GET_INSERTEDCH_LENGTH)){
        /* wait until end of inserted channel conversion flag is raised */
        while(RESET == adc_flag_get(ADC_FLAG_EOIC)){
            /* check for the timeout */
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    _adc_state_set(adc, ADC_STATE_TIMEOUT);
                    /* when timeout occurs, output timeout warning message */
                    HAL_DEBUGW("adc inserted conversion poll timeout");
                    return HAL_ERR_TIMEOUT;
                }            
            }
        }
    }else{
        return HAL_ERR_NO_SUPPORT;
    }

    /* clear ADC EOIC/STIC/EOC flag */
    adc_flag_clear(ADC_FLAG_EOIC);
    adc_flag_clear(ADC_FLAG_STIC);
    adc_flag_clear(ADC_FLAG_EOC);

    /* set ADC state */
    _adc_state_set(adc, ADC_STATE_INSERTED_EOC);
    
    if((ADC_EXTTRIG_INSERTED_NONE == __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER) ||((RESET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) \
        && ((ADC_EXTTRIG_REGULAR_NONE == __HAL_ADC_GET_REGULARCH_EXTTRIGGER) && (RESET == __HAL_ADC_GET_CONTINUOUS_MODE)))){
       /* set ADC state */
       _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
        if(RESET == ((hal_adc_state_get(adc)) & ADC_STATE_REGULAR_BUSY)){
           _adc_state_set(adc, ADC_STATE_READY);
       }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC inserted group software trigger
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
/*  */
void hal_adc_inserted_software_trigger_enable(hal_adc_dev_struct *adc)
{
    adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
}

/*!
    \brief      start ADC EOIC interrupt
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: EOIC interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_inserted_start_interrupt(hal_adc_dev_struct *adc, hal_adc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_irq) ){
        HAL_DEBUGE("pointer [adc] or [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* if ADC is enabled */
    if(HAL_ERR_NONE == _adc_enable(adc)){
        _adc_state_clear(adc, ADC_STATE_READY);
        _adc_state_clear(adc, ADC_STATE_INSERTED_EOC);
        _adc_state_set(adc, ADC_STATE_INSERTED_BUSY);
        
        /* check if an inserted conversion is ongoing */
        if(RESET == ((hal_adc_state_get(adc)) & ADC_STATE_REGULAR_BUSY)){
            adc->error_state = ADC_ERROR_NONE;
        }

        adc->adc_irq.adc_eoic_handle = p_irq->adc_eoic_handle;
        /* clear ADC interrupt flag */
        adc_interrupt_flag_clear(ADC_INT_FLAG_EOIC);
        /* enable ADC end of group conversion flag */
        adc_interrupt_enable(ADC_INT_EOIC);
        
        /* check if inserted group external trigger is enable*/
        if(RESET == ((ADC_CTL1) & (ADC_CTL1_ETEIC))){
            /* enable software trigger */
            adc_external_trigger_config(ADC_INSERTED_CHANNEL, ENABLE);
        }
        if(RESET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV){
            if(ADC_EXTTRIG_INSERTED_NONE == __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER){
                /* enable software trigger */
                adc_software_trigger_enable(ADC_INSERTED_CHANNEL);
            }
        }
    }else{
        return HAL_ERR_HARDWARE;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      stop the conversion of inserted group and disable ADC
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_inserted_stop_interrupt(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* regular group is not busy and automatic convert is set */   
    if((RESET ==((hal_adc_state_get(adc)) & ADC_STATE_REGULAR_BUSY)) && (SET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV)){
        /* if ADC is disabled */
        if(HAL_ERR_NONE == _adc_disable(adc)){
            /* disable end of inserted group conversion interrupt */
            adc_interrupt_disable(ADC_INT_EOIC);
            adc->adc_irq.adc_eoic_handle = NULL;

            /* set ADC state */;
            _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
            _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
            _adc_state_set(adc, ADC_STATE_READY);
        }
    }else{
        _adc_error_set(adc, ADC_ERROR_CONFIG);
        return HAL_ERR_NO_SUPPORT;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      adc watchdog config
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_watchdog: the pointer of ADC watchdog configuration structure
                  watchdog_mode:
                    the argument could be selected from enumeration <hal_adc_watchdog_mode_enum>
                  single_channel: ADC_CHANNEL_x(x=0..9,16,17)
                  low_threshold: analog watchdog low threshold, 0..4095
                  high_threshold: analog watchdog high threshold, 0..4095
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_watchdog_config(hal_adc_dev_struct *adc, hal_adc_watchdog_struct *p_watchdog)
{
    uint32_t mode_temp, reg_temp;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_watchdog)){
        HAL_DEBUGE("pointer [adc] or [p_watchdog] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    reg_temp = ADC_CTL0;
    reg_temp &= (uint32_t)~(ADC_CTL0_RWDEN | ADC_CTL0_IWDEN | ADC_CTL0_WDSC);
    mode_temp = p_watchdog->watchdog_mode;

    /* select watchdog mode mode */ 
    if((ADC_WATCHDOG_SINGLE_REGULAR == mode_temp) || (ADC_WATCHDOG_SINGLE_INSERTED == mode_temp)|| (ADC_WATCHDOG_SINGLE_REGINS == mode_temp)){
        /* set watchdog single channel */ 
        reg_temp &= (uint32_t)(~ADC_CTL0_WDCHSEL);
        reg_temp |= (uint32_t)(p_watchdog->single_channel);
        reg_temp |= (uint32_t)mode_temp;
    }else{
        reg_temp |= (uint32_t)mode_temp;
    }
    ADC_CTL0 = reg_temp;

    /* ADC analog watchdog low threshold */
    ADC_WDLT = (uint32_t)WDLT_WDLT(p_watchdog->low_threshold);
    /* ADC analog watchdog high threshold */
    ADC_WDHT = (uint32_t)WDHT_WDHT(p_watchdog->high_threshold);

    return HAL_ERR_NONE;
}

/*!
    \brief      enable ADC watchdog interrupt
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: watchdog interrupt callback function structure
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_watchdog_interrupt_enable(hal_adc_dev_struct *adc, hal_adc_irq_struct *p_irq)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == adc) || (NULL == p_irq)){
        HAL_DEBUGE("pointer [adc] or [p_irq] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* ADC analog watchdog interrupt handle config */
    adc->adc_irq.adc_watchdog_handle = p_irq->adc_watchdog_handle;
    /* clear ADC watchdog interrupt flag */
    adc_interrupt_flag_clear(ADC_INT_FLAG_WDE);
    /* enable ADC watchdog interrupt */
    adc_interrupt_enable(ADC_INT_WDE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      disable ADC watchdog interrupt
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_watchdog_interrupt_disable(hal_adc_dev_struct *adc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* disable ADC watchdog interrupt */
    adc_interrupt_disable(ADC_INT_WDE);
    /* ADC analog watchdog interrupt handle config */
    adc->adc_irq.adc_watchdog_handle = NULL;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      polling for watchdog event
    \param[in]  adc: ADC device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: time out duration in milliscond
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_adc_watchdog_event_poll(hal_adc_dev_struct *adc, uint32_t timeout_ms)
{
    uint32_t tick_start = 0;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    /* set timeout */
    tick_start = hal_basetick_count_get();

    /* check watchdog event flag */
    while(RESET == adc_flag_get(ADC_FLAG_WDE)){
        /* check for the timeout */
        if(HAL_TIMEOUT_FOREVER != timeout_ms){
            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                _adc_state_set(adc, ADC_STATE_TIMEOUT);
                /* when timeout occurs, output timeout warning message */
                HAL_DEBUGW("adc watchdog event poll timeout");
                return HAL_ERR_TIMEOUT;
            }
        }
    }
    
    /* set ADC state */
    _adc_state_set(adc, ADC_STATE_WATCHDOG);
    
    /* clear ADC analog watchdog event flag */
    adc_flag_clear(ADC_FLAG_WDE);

    return HAL_ERR_NONE;
}

/*!
    \brief      ADC interrupt handler content function, which is merely used in ADC_CMP_IRQHandler
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_adc_irq(hal_adc_dev_struct *adc)
{
    /* EOC interrupt handler */
    if(SET == (adc_interrupt_flag_get(ADC_INT_FLAG_EOC))){
        /* clear EOC flag */
        adc_interrupt_flag_clear(ADC_INT_FLAG_EOC);
            if(NULL != (adc->adc_irq.adc_eoc_handle)){
            adc->adc_irq.adc_eoc_handle(adc);
        }
        /* if not in error state */
        if(SET == ((hal_adc_error_get(adc)) & ADC_ERROR_SYSTEM)){
            _adc_state_set(adc, ADC_STATE_REGULAR_EOC);
        }

        if((ADC_EXTTRIG_REGULAR_NONE == __HAL_ADC_GET_REGULARCH_EXTTRIGGER)&&(RESET == __HAL_ADC_GET_CONTINUOUS_MODE)){
            adc_interrupt_disable(ADC_INT_EOC);
            /* set ADC state */
            _adc_state_clear(adc, ADC_STATE_REGULAR_BUSY);
            if(RESET == ((hal_adc_state_get(adc)) & ADC_STATE_INSERTED_BUSY)){
                _adc_state_set(adc, ADC_STATE_READY);
            }
        }
    }
    /* EOIC interrupt handler */
    if(SET == (adc_interrupt_flag_get(ADC_INT_FLAG_EOIC))){
        /* clear EOIC flag */
        adc_interrupt_flag_clear(ADC_INT_FLAG_EOIC);
        if(NULL != (adc->adc_irq.adc_eoic_handle)){
            adc->adc_irq.adc_eoic_handle(adc);
        }
        /* if not in error state */
        if(SET == ((hal_adc_error_get(adc)) & ADC_ERROR_SYSTEM)){
            _adc_state_set(adc, ADC_STATE_INSERTED_EOC);
        }

        if((ADC_EXTTRIG_INSERTED_NONE == __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER) ||((RESET == __HAL_ADC_GET_INSERTEDCH_AUTOCONV) \
            && ((ADC_EXTTRIG_REGULAR_NONE == __HAL_ADC_GET_REGULARCH_EXTTRIGGER) && (RESET == __HAL_ADC_GET_CONTINUOUS_MODE)))){
            adc_interrupt_disable(ADC_INT_EOIC);
            /* set ADC state */
            _adc_state_clear(adc, ADC_STATE_INSERTED_BUSY);
            if(RESET == ((hal_adc_state_get(adc)) & ADC_STATE_REGULAR_BUSY)){
                _adc_state_set(adc, ADC_STATE_READY);
            }
        }
    }
    /* watchdog interrupt handler */
    if(SET == (adc_interrupt_flag_get(ADC_INT_FLAG_WDE))){
        /* clear watchdog flag */
        adc_interrupt_flag_clear(ADC_INT_FLAG_WDE);
        if(NULL != (adc->adc_irq.adc_watchdog_handle)){
            adc->adc_irq.adc_watchdog_handle(adc);
        }
        /* set ADC state */
        _adc_state_set(adc, ADC_STATE_WATCHDOG);
    }
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to ADC interrupt callback functions structure
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_adc_irq_handle_set(hal_adc_dev_struct *adc, hal_adc_irq_struct *p_irq)
{
    /* EOC interrupt handler set */
    if(NULL != p_irq->adc_eoc_handle){
        adc->adc_irq.adc_eoc_handle = p_irq->adc_eoc_handle;
        adc_interrupt_enable(ADC_INT_EOC);
    }else{
        adc->adc_irq.adc_eoc_handle = NULL;
        adc_interrupt_enable(ADC_INT_EOC);
    }
    /* EOIC interrupt handler set */
    if(NULL != p_irq->adc_eoic_handle){
        adc->adc_irq.adc_eoic_handle = p_irq->adc_eoic_handle;
        adc_interrupt_enable(ADC_INT_EOIC);
    }else{
        adc->adc_irq.adc_eoic_handle = NULL;
        adc_interrupt_disable(ADC_INT_EOIC);
    }
    /* watchdog interrupt handler set */
    if(NULL != p_irq->adc_watchdog_handle){
        adc->adc_irq.adc_watchdog_handle = p_irq->adc_watchdog_handle;
        adc_interrupt_enable(ADC_INT_WDE);
    }else{
        adc->adc_irq.adc_watchdog_handle = NULL;
        adc_interrupt_disable(ADC_INT_WDE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_adc_irq_handle_all_reset(hal_adc_dev_struct *adc)
{
    /* ADC interrupt handler reset */
    adc->adc_irq.adc_watchdog_handle = NULL;
    adc->adc_irq.adc_eoc_handle      = NULL;
    adc->adc_irq.adc_eoic_handle     = NULL;
}

/*!
    \brief      get ADC regular group conversion result
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the regular group conversion result(0-0xFFFF)
*/
uint16_t hal_adc_regular_value_get(hal_adc_dev_struct *adc)
{
    /* return ADC regular group converted value */ 
    return ((uint16_t)ADC_RDATA);
}

/*!
    \brief      get ADC inserted group conversion result
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the inserted group conversion result(0-0xFFFF)
*/
uint16_t hal_adc_inserted_value_get(hal_adc_dev_struct *adc, uint8_t inschannel_sequence)
{
    uint16_t idata_temp = 0;

    /* clear ADC interrupt flag */
    adc_flag_clear(ADC_FLAG_EOIC);

    /* read ADC converted value */ 
    idata_temp = adc_inserted_data_read(inschannel_sequence);
    
    /* return ADC converted value */ 
    return idata_temp;
}

/*!
    \brief      get ADC error
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the error state
*/
uint32_t hal_adc_error_get(hal_adc_dev_struct *adc)
{
    /* return ADC error */
    return (adc->error_state);
}

/*!
    \brief      get ADC state
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the state
*/
uint32_t hal_adc_state_get(hal_adc_dev_struct *adc)
{
    /* return ADC state */
    return (adc->state);
}

/*!
    \brief      enable the ADC
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
static int32_t _adc_enable(hal_adc_dev_struct *adc)
{
    uint32_t tick_start = 0;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* if ADC is disabled */
    if(RESET == _adc_enable_state_get()){
        /* enable ADC */
        adc_enable();
        /* wait for ADC enable */
        hal_basetick_delay_ms(1U);
        
        tick_start = hal_basetick_count_get();
        
        /* wait for ADC actually enabled */
        while(RESET == _adc_enable_state_get()){
            if(SET == hal_basetick_timeout_check(tick_start, ADC_ENABLE_DELAYTIME)){
                _adc_error_set(adc, ADC_ERROR_SYSTEM);
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      disable the ADC
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
static int32_t _adc_disable(hal_adc_dev_struct *adc)
{
    uint32_t tick_start = 0;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == adc){
        HAL_DEBUGE("pointer [adc] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    /* if ADC is not already disabled */
    if(RESET != _adc_enable_state_get()){
        /* disable ADC */
        adc_disable();
        
        /* wait for ADC disable */
        tick_start = hal_basetick_count_get();
        
        /* wait for ADC actually disabled */
        while(RESET != _adc_enable_state_get()){
            if(SET == hal_basetick_timeout_check(tick_start, ADC_ENABLE_DELAYTIME)){
                _adc_error_set(adc, ADC_ERROR_SYSTEM);
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      get ADC enable state
    \param[in]  adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the enable state: SET or RESET
*/
static FlagStatus _adc_enable_state_get(void)
{
    if(0U !=(ADC_CTL1 & ADC_CTL1_ADCON)){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      ADC DMA transmission complete callback
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _adc_dma_transfer_complete(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_adc_dev_struct *p_adc;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_adc = (hal_adc_dev_struct*)(p_dma->p_periph);

    if(RESET == ((hal_adc_error_get(p_adc)) & (ADC_ERROR_DMA | ADC_ERROR_SYSTEM))){
        /* set ADC state */
        _adc_state_set(p_adc, ADC_STATE_REGULAR_EOC);

        if((ADC_EXTTRIG_REGULAR_NONE == __HAL_ADC_GET_REGULARCH_EXTTRIGGER)&&(DISABLE == __HAL_ADC_GET_CONTINUOUS_MODE)){
          /* set ADC state */
          _adc_state_clear(p_adc, ADC_STATE_REGULAR_BUSY);
          if(RESET == ((hal_adc_state_get(p_adc)) & ADC_STATE_INSERTED_BUSY)){
              _adc_state_set(p_adc, ADC_STATE_READY);
          }
        }

        if(NULL != (p_adc->adc_dma.transcom_handle)){
            p_adc->adc_dma.transcom_handle(p_adc);
        }
    }
}

/*!
    \brief      ADC DMA error callback
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _adc_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_adc_dev_struct *p_adc;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_adc = (hal_adc_dev_struct*)p_dma->p_periph;
    
    if(NULL != (p_adc->adc_dma.error_handle)){
        p_adc->adc_dma.error_handle(p_adc);
    }
    
    /* set ADC state and error */
    _adc_error_set(p_adc, ADC_ERROR_DMA);
}

/*!
    \brief      set ADC state
    \param[in]  d_adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  adc_state: the state of ADC
                  the argument could be selected from enumeration <hal_adc_state_enum>
    \param[out] none
    \retval     the ADC state
*/
static void _adc_state_set(hal_adc_dev_struct *d_adc, hal_adc_state_enum adc_state)
{
    /* set ADC state */
    d_adc->state |= (adc_state);
}

/*!
    \brief      clear ADC state
    \param[in]  d_adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  adc_state: the state of ADC
                  the argument could be selected from enumeration <hal_adc_state_enum>
    \param[out] none
    \retval     the ADC state
*/
static void _adc_state_clear(hal_adc_dev_struct *d_adc, hal_adc_state_enum adc_state)
{
    /* clear ADC state */
    d_adc->state &= ~(adc_state);
}

/*!
    \brief      set ADC error
    \param[in]  d_adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  adc_state: the state of ADC
                  the argument could be selected from enumeration <hal_adc_error_enum>
    \param[out] none
    \retval     the ADC error
*/
static void _adc_error_set(hal_adc_dev_struct *d_adc, hal_adc_error_enum adc_error)
{
    /* set ADC error */
    d_adc->error_state |= (adc_error);
}

/*!
    \brief      clear ADC error
    \param[in]  d_adc: ADC device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  adc_state: the state of ADC
                  the argument could be selected from enumeration <hal_adc_error_enum>
    \param[out] none
    \retval     the ADC error
*/
static void _adc_error_clear(hal_adc_dev_struct *d_adc, hal_adc_error_enum adc_error)
{
    /* clear ADC error */
    d_adc->error_state &= ~(adc_error);
}
