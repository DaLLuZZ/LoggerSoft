/*!
    \file    gd32e23x_hal_cmp.c
    \brief   CMP driver

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
    \brief      comparator init struct deinitialize
    \param[in]  hal_struct_type: the type of the structure
    \param[in]  p_struct: the pointer of the structure
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL,
                details refer to gd32e23x_hal.h
*/
int32_t hal_cmp_struct_init(hal_cmp_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == p_struct){
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(hal_struct_type){
        case HAL_CMP_INIT_STRUCT:
            /* initialize comparator initialization structure with the default values */
            ((hal_cmp_init_struct*)p_struct)->inverting_input_select = CMP_1_4VREFINT;
            ((hal_cmp_init_struct*)p_struct)->noninverting_input_select = CMP_PA1;
            ((hal_cmp_init_struct*)p_struct)->output_select = CMP_OUTPUT_NONE;
            ((hal_cmp_init_struct*)p_struct)->output_polarity = CMP_OUTPUT_POLARITY_NOINVERTED;
            ((hal_cmp_init_struct*)p_struct)->mode = CMP_HIGHSPEED;
            ((hal_cmp_init_struct*)p_struct)->hysteresis_level = CMP_HYSTERESIS_NO;
            ((hal_cmp_init_struct*)p_struct)->exti_mode = CMP_EXTI_NONE;

            break;

        case HAL_CMP_DEV_STRUCT:
            /* initialize comparator device information structure with the default values */
            ((hal_cmp_dev_struct*)p_struct)->cmp_irq_handle = NULL;
            ((hal_cmp_dev_struct*)p_struct)->state = CMP_STATE_RESET;
            ((hal_cmp_dev_struct*)p_struct)->output_level = RESET;
        
            break;
        
        default:
            HAL_DEBUGW("parameter value is undefine");
            return HAL_ERR_VAL;
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      deinitialize comparator device structure and reset the peripheral
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_LOCK,
                details refer to gd32e23x_hal.h
*/
int32_t hal_cmp_deinit(hal_cmp_dev_struct *cmp)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check whether the comparator is locked or not */
    if(CMP_STATE_LOCKED != cmp->state){
        hal_cmp_struct_init(HAL_CMP_DEV_STRUCT, cmp);
        CMP_CS = ((uint32_t)0x00000000U);
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_LOCK;
    }
}

/*!
    \brief      initialize the comparator
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_init: the pointer of comparator init structure
                  inverting_input_select:
                    the argument could be selected from enumeration <inverting_input_enum>
                  noninverting_input_select:
                    the argument could be selected from enumeration <noninverting_input_enum>
                  output_select:
                    the argument could be selected from enumeration <cmp_output_enum>
                  output_polarity: CMP_OUTPUT_POLARITY_NOINVERTED, CMP_OUTPUT_POLARITY_INVERTED
                  mode:
                    the argument could be selected from enumeration <operating_mode_enum>
                  hysteresis_level:
                    the argument could be selected from enumeration <cmp_hysteresis_enum>
                  exti_mode:
                    the argument could be selected from enumeration <external_trigger_mode_enum>
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT,
                details refer to gd32e23x_hal.h
*/
int32_t hal_cmp_init(hal_cmp_dev_struct *cmp, hal_cmp_init_struct *p_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == cmp)||(NULL == p_init)){
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* initialize the comparator */
    if(CMP_STATE_RESET == cmp->state){
        CMP_CS &= 0U;
        CMP_CS |= (CS_CMPMSEL(p_init->inverting_input_select) | \
                   CS_CMPOSEL(p_init->output_select) | \
                   CS_CMPM(p_init->mode) | \
                   CS_CMPHST(p_init->hysteresis_level));

        /* configure the output polarity */
        if(CMP_OUTPUT_POLARITY_INVERTED == p_init->output_polarity){
            CMP_CS |= CMP_CS_CMPPL;
        }else{
            CMP_CS &= ~CMP_CS_CMPPL; 
        }

        /* configure the noninverting input port */
        if(CMP_PA4_SWCLOSE == p_init->noninverting_input_select){
            CMP_CS |= CMP_CS_CMPSW;
        }else{
            CMP_CS &= ~CMP_CS_CMPSW;
        }

        /* enable external event or interrupt */
        if(CMP_EXTI_NONE != p_init->exti_mode){
            hal_exti_internal_init(EXTI_LINE_21_CMP_OUTPUT, (hal_exti_type_enum)(p_init->exti_mode));
        }

        cmp->state = CMP_STATE_READY;

        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_NO_SUPPORT;
    }
}

/*!
    \brief      start cmp module function with event or not
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK,
                details refer to gd32e23x_hal.h
*/
int32_t hal_cmp_start(hal_cmp_dev_struct *cmp)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(CMP_STATE_LOCKED == cmp->state){
        /* the comparator is locked */
        HAL_DEBUGE("comparator is locked");
        return HAL_ERR_LOCK;
    }else if(CMP_STATE_READY == cmp->state){
        cmp_enable();

        /* wait for comparator startup */
        hal_basetick_delay_ms(1U);

        cmp->state = CMP_STATE_RUN;

        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_NO_SUPPORT;
    }
}

/*!
    \brief      stop cmp module function with event or not
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK,
                details refer to gd32e23x_hal.h
*/
int32_t hal_cmp_stop(hal_cmp_dev_struct *cmp)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(CMP_STATE_LOCKED == cmp->state){
        /* the comparator is locked */
        HAL_DEBUGE("comparator is locked");
        return HAL_ERR_LOCK;
    }else if(CMP_STATE_RUN == cmp->state){
        cmp_disable();

        cmp->state = CMP_STATE_READY;

        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_NO_SUPPORT;
    }
    
}

/*!
    \brief      start the comparator with interrupt
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK,
                details refer to gd32e23x_hal.h
*/
int32_t hal_cmp_start_interrupt(hal_cmp_dev_struct *cmp, hal_cmp_irq_handle_cb irq_handler)
{
    int32_t state;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }

    if(NULL == irq_handler){
        HAL_DEBUGE("irq_handler is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* configure interrupt callback function */
    cmp->cmp_irq_handle = irq_handler;

    state = hal_cmp_start(cmp);

    return state;
}

/*!
    \brief      stop the comparator with interrupt
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, HAL_ERR_LOCK,
                details refer to gd32e23x_hal.h
*/
int32_t hal_cmp_stop_interrupt(hal_cmp_dev_struct *cmp)
{
    int32_t state;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* reset interrupt callback function */
    cmp->cmp_irq_handle = NULL;

    state = hal_cmp_stop(cmp);

    return state;
}

/*!
    \brief      set the comparator external trigger interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  hal_cmp_irq_handle_cb: the pointer of the comparator interrupt callback
    \param[out] none
    \retval     none
*/
void hal_cmp_irq_handle_set(hal_cmp_dev_struct *cmp, hal_cmp_irq_handle_cb irq_handler)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
    }

    if(NULL == irq_handler){
        HAL_DEBUGE("callback function is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* configure interrupt callback function */
    cmp->cmp_irq_handle = irq_handler;
}

/*!
    \brief      reset the comparator external trigger interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_cmp_irq_handle_all_reset(hal_cmp_dev_struct *cmp)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* reset interrupt callback function */
    cmp->cmp_irq_handle = NULL;
}

/*!
    \brief      cmp external trigger interrupt handler content function, 
                which is merely used in ADC_CMP_IRQHandler
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_cmp_irq(hal_cmp_dev_struct *cmp)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(SET == exti_interrupt_flag_get(EXTI_21)){
        exti_interrupt_flag_clear(EXTI_21);
        if(NULL != cmp->cmp_irq_handle){
            cmp->cmp_irq_handle(cmp);
        }
    }
}

/*!
    \brief      lock the comparator
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_ALREADY_DONE,
                details refer to gd32e23x_hal.h
*/
int32_t hal_cmp_lock(hal_cmp_dev_struct *cmp)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    if(CMP_STATE_LOCKED != cmp->state){
        cmp_lock_enable();
        cmp->state = CMP_STATE_LOCKED;
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_ALREADY_DONE;
    }
}

/*!
    \brief      get output level of comparator
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the output level
*/
uint32_t hal_cmp_output_level_get(hal_cmp_dev_struct *cmp)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    cmp->output_level = cmp_output_level_get();

    return (cmp->output_level);
}

/*!
    \brief      get the state of comparator
    \param[in]  cmp: comparator device information structure
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     hal_cmp_state_enum: CMP_STATE_NONE, CMP_STATE_RESET, CMP_STATE_READY,
                                    CMP_STATE_LOCKED, CMP_STATE_RUN
*/
hal_cmp_state_enum hal_cmp_state_get(hal_cmp_dev_struct *cmp)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == cmp){
        HAL_DEBUGE("pointer address is invalid");
        return CMP_STATE_NONE;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    return (cmp->state);
}
