/*!
    \file    gd32e23x_hal_smartcard.c
    \brief   SMARTCARD driver
    
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

/* smartcard private function */
static FlagStatus _smartcard_error_flag_get(hal_smartcard_dev_struct *smartcard);
static void _smartcard_transmit_complete_interrupt(void *smartcard);
static void _smartcard_transmit_interrupt(void *smartcard);
static void _smartcard_receive_interrupt(void *smartcard);
static void _smartcard_transmit_dma(void *dma);
static void _smartcard_receive_dma(void *dma);
static void _smartcard_dma_error(void *dma);

/*!
    \brief      initialize the smartcard structure with the default values
    \param[in]  hal_struct_type: smartcard structure type
      \arg        HAL_SMARTCARD_INIT_STRUCT: initialization structure
      \arg        HAL_SMARTCARD_INIT_EX_STRUCT: initialization extend structure
      \arg        HAL_SMARTCARD_DEV_STRUCT: device information structure
      \arg        HAL_SMARTCARD_USER_CALLBCAK_STRUCT: user callback structure
      \arg        HAL_SMARTCARD_IRQ_INIT_STRUCT: interrupt callback initialization structure
    \param[in]  p_struct: init structure pointer
    \param[out] none
    \retval     none
*/
void hal_smartcard_struct_init(hal_smartcard_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1U == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    switch(hal_struct_type){
    case HAL_SMARTCARD_INIT_STRUCT:
        /* initialize smartcard initialization structure with the default values */
        ((hal_smartcard_init_struct*)p_struct)->baudrate = 115200U;
        ((hal_smartcard_init_struct*)p_struct)->parity = SMARTCARD_PARITY_EVEN;
        ((hal_smartcard_init_struct*)p_struct)->word_length = SMARTCARD_WORD_LENGTH_9BIT;
        ((hal_smartcard_init_struct*)p_struct)->stop_bit = SMARTCARD_STOP_BIT_1_5;
        ((hal_smartcard_init_struct*)p_struct)->direction = SMARTCARD_DIRECTION_RX_TX;
        ((hal_smartcard_init_struct*)p_struct)->clock_polarity = SMARTCARD_CLOCK_POLARITY_LOW;
        ((hal_smartcard_init_struct*)p_struct)->clock_phase = SMARTCARD_CLOCK_PHASE_1CK;
        ((hal_smartcard_init_struct*)p_struct)->clock_length_lastbit = SMARTCARD_LAST_BIT_NOT_OUTPUT;
        ((hal_smartcard_init_struct*)p_struct)->prescaler = 10U;
        ((hal_smartcard_init_struct*)p_struct)->guard_time = 0U;
        ((hal_smartcard_init_struct*)p_struct)->nack_state = SMARTCARD_NACK_ENABLE;
        ((hal_smartcard_init_struct*)p_struct)->timeout_enable = SMARTCARD_TIMEOUT_DISABLE;
        ((hal_smartcard_init_struct*)p_struct)->timeout_value = 0U;
        ((hal_smartcard_init_struct*)p_struct)->block_length = 0U;
        ((hal_smartcard_init_struct*)p_struct)->sample_method = SMARTCARD_THREE_SAMPLE_BIT;
        ((hal_smartcard_init_struct*)p_struct)->auto_retry_count = 0U;
        break;
    
    case HAL_SMARTCARD_INIT_EX_STRUCT:
        /* initialize smartcard extend initialization structure with the default values */
        ((hal_smartcard_init_ex_struct*)p_struct)->data_bit_invert = DISABLE;
        ((hal_smartcard_init_ex_struct*)p_struct)->first_bit_msb = DISABLE;
        ((hal_smartcard_init_ex_struct*)p_struct)->overrun_disable = DISABLE;
        ((hal_smartcard_init_ex_struct*)p_struct)->rx_error_dma_stop = DISABLE;
        ((hal_smartcard_init_ex_struct*)p_struct)->rx_level_invert = DISABLE;
        ((hal_smartcard_init_ex_struct*)p_struct)->tx_level_invert = DISABLE;
        ((hal_smartcard_init_ex_struct*)p_struct)->tx_rx_swap = DISABLE;
        break;

    case HAL_SMARTCARD_DEV_STRUCT:
        /* initialize smartcard device information structure with the default values */
        ((hal_smartcard_dev_struct*)p_struct)->periph = 0U;
        ((hal_smartcard_dev_struct*)p_struct)->smartcard_irq.error_handle = NULL;
        ((hal_smartcard_dev_struct*)p_struct)->smartcard_irq.receive_complete_handle = NULL;
        ((hal_smartcard_dev_struct*)p_struct)->smartcard_irq.transmit_complete_handle = NULL;
        ((hal_smartcard_dev_struct*)p_struct)->smartcard_irq.transmit_ready_handle = NULL;
        ((hal_smartcard_dev_struct*)p_struct)->p_dma_rx = NULL;
        ((hal_smartcard_dev_struct*)p_struct)->p_dma_tx = NULL;
        ((hal_smartcard_dev_struct*)p_struct)->txbuffer.buffer = NULL;
        ((hal_smartcard_dev_struct*)p_struct)->txbuffer.length = 0U;
        ((hal_smartcard_dev_struct*)p_struct)->txbuffer.pos = 0U;
        ((hal_smartcard_dev_struct*)p_struct)->rxbuffer.buffer = NULL;
        ((hal_smartcard_dev_struct*)p_struct)->rxbuffer.length = 0U;
        ((hal_smartcard_dev_struct*)p_struct)->rxbuffer.pos = 0U;
        ((hal_smartcard_dev_struct*)p_struct)->error_state = HAL_SMARTCARD_ERROR_NONE;
        ((hal_smartcard_dev_struct*)p_struct)->tx_state = SMARTCARD_STATE_FREE;
        ((hal_smartcard_dev_struct*)p_struct)->rx_state = SMARTCARD_STATE_FREE;
        ((hal_smartcard_dev_struct*)p_struct)->last_error = HAL_SMARTCARD_ERROR_NONE;
        ((hal_smartcard_dev_struct*)p_struct)->priv = NULL;
        break;

    case HAL_SMARTCARD_USER_CALLBCAK_STRUCT:
        /* initialize user callback structure with the default values */
        ((hal_smartcard_user_callback_struct*)p_struct)->complete_func = NULL;
        ((hal_smartcard_user_callback_struct*)p_struct)->error_func = NULL;
        break;

    case HAL_SMARTCARD_IRQ_INIT_STRUCT:
        /* initialize interrupt callback structure with the default values */
        ((hal_smartcard_irq_struct*)p_struct)->error_handle = NULL;
        ((hal_smartcard_irq_struct*)p_struct)->receive_complete_handle = NULL;
        ((hal_smartcard_irq_struct*)p_struct)->transmit_complete_handle = NULL;
        ((hal_smartcard_irq_struct*)p_struct)->transmit_ready_handle = NULL;
        break;

    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize smartcard
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_deinit(hal_smartcard_dev_struct *smartcard)
{
    uint32_t periph;
    periph = smartcard->periph;

    if((USART0 == periph)){
        /* deinitialize the peripheral */
        usart_deinit(periph);
        /* initialize smartcard extend initialization structure */
        hal_smartcard_struct_init(HAL_SMARTCARD_DEV_STRUCT, smartcard);
        smartcard->periph = periph;
    }else{
        HAL_DEBUGE("parameter [smartcard->periph] value is invalid");
    }
}

/*!
    \brief      initialize smartcard
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is initialized
                    this parameter can only be USART0 for gd32e23x
    \param[in]  p_init: the initialization data needed to initialize smartcard
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_init(hal_smartcard_dev_struct *smartcard, uint32_t periph, \
                           hal_smartcard_init_struct *p_init)
{
    uint32_t reg_temp;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer and p_init address */
    if((NULL == smartcard) || (NULL == p_init)){
        HAL_DEBUGE("pointer [smartcard] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check periph parameter */
    if(USART0 != periph){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }

    /* check periph value from smartcard device struct */
    if(0U != smartcard->periph){
        HAL_DEBUGI("periph value from smartcard device struct has been rewrite");
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    smartcard->periph = periph;

    /* disable the peripheral */
    usart_disable(periph);

    /* CTL0 register configure */
    reg_temp = USART_CTL0(periph);
    reg_temp &= ~(USART_CTL0_PM | USART_CTL0_PCEN | USART_CTL0_WL | USART_CTL0_OVSMOD | \
                  USART_CTL0_REN | USART_CTL0_TEN);
    reg_temp |= (p_init->direction | p_init->parity | p_init->word_length);
    USART_CTL0(periph) = reg_temp;

    /* CTL1 register configure */
    reg_temp = USART_CTL1(periph);
    reg_temp &= ~(USART_CTL1_RTEN | USART_CTL1_CLEN | USART_CTL1_CPH | USART_CTL1_CPL| \
                  USART_CTL1_CKEN | USART_CTL1_STB);
    /* Synchronous mode is activated by default */
    reg_temp |= (USART_CTL1_CKEN | p_init->clock_polarity | p_init->clock_phase | \
                p_init->clock_length_lastbit | p_init->timeout_enable | p_init->stop_bit);
    USART_CTL1(periph) = reg_temp;

    /* CTL2 register configure */
    reg_temp = USART_CTL2(periph);
    reg_temp &= ~(USART_CTL2_OSB | USART_CTL2_NKEN |USART_CTL2_SCRTNUM);
    reg_temp |= (p_init->sample_method | p_init->nack_state | (p_init->auto_retry_count << 17U));
    USART_CTL2(periph) = reg_temp;

    /* GP register configure */
    reg_temp = USART_GP(periph);
    reg_temp &= ~(USART_GP_PSC | USART_GP_GUAT);
    reg_temp |= ((p_init->guard_time << 8U) | p_init->prescaler);
    USART_GP(periph) = reg_temp;

    /* RT register configure */
    reg_temp = USART_RT(periph);
    reg_temp &= ~(USART_RT_RT | USART_RT_BL);
    reg_temp |= ((p_init->block_length << 24U));
    if(SMARTCARD_TIMEOUT_ENABLE == p_init->timeout_enable) {
        reg_temp |= p_init->timeout_value;
    }
    USART_RT(periph) = reg_temp;

    /* baud rate configure */
    usart_baudrate_set(periph, p_init->baudrate);

    /* clear LMEN, HDEN, IREN */
    usart_halfduplex_disable(periph);
    usart_irda_mode_disable(periph);
    usart_lin_mode_disable(periph);

    /* enable SMARTCARD */
    usart_smartcard_mode_enable(periph);

    /* initialize Tx and Rx state */
    smartcard->tx_state = SMARTCARD_STATE_FREE;
    smartcard->rx_state = SMARTCARD_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize extended smartcard
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is initialized
                    this parameter can only be USART0 for gd32e23x
    \param[in]  p_init: the extended initialization data needed to initialize smartcard
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_init_ex(hal_smartcard_dev_struct *smartcard, uint32_t periph, \
                              hal_smartcard_init_ex_struct *p_init)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer and p_init address */
    if((NULL == smartcard) || (NULL == p_init)){
        HAL_DEBUGE("pointer [smartcard] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check periph parameter */
    if(USART0 != periph){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    smartcard->periph = periph;

    /* disable the peripheral */
    usart_disable(periph);

    /* configure data inversion */
    if(ENABLE == p_init->data_bit_invert){
        USART_CTL1(periph) |= USART_CTL1_DINV;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_DINV);
    }

    /* configure Rx pin active level inversion */
    if(ENABLE == p_init->rx_level_invert){
        USART_CTL1(periph) |= USART_CTL1_RINV;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_RINV);
    }

    /* configure Tx pin active level inversion */
    if(ENABLE == p_init->tx_level_invert){
        USART_CTL1(periph) |= USART_CTL1_TINV;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_TINV);
    }

    /* configure Rx/Tx pins swap */
    if(ENABLE == p_init->tx_rx_swap){
        USART_CTL1(periph) |= USART_CTL1_STRP;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_STRP);
    }

    /* configure MSB first on communication line */
    if(ENABLE == p_init->first_bit_msb){
        USART_CTL1(periph) |= USART_CTL1_MSBF;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_MSBF);
    }

    /* configure Rx overrun detection disabling */
    if(ENABLE == p_init->overrun_disable){
        USART_CTL2(periph) |= USART_CTL2_OVRD;
    }else{
        USART_CTL2(periph) &= ~(USART_CTL2_OVRD);
    }

    /* configure DMA disabling on reception error */
    if(ENABLE == p_init->rx_error_dma_stop){
        USART_CTL2(periph) |= USART_CTL2_DDRE;
    }else{
        USART_CTL2(periph) &= ~(USART_CTL2_DDRE);
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      smartcard interrupt handler content function
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_irq(hal_smartcard_dev_struct *smartcard)
{
    /* if no error occurs */
    if(RESET == _smartcard_error_flag_get(smartcard)){
        /* if USART is in receiver mode */
        if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_RBNE)){
            if(NULL != smartcard->smartcard_irq.receive_complete_handle){
                smartcard->smartcard_irq.receive_complete_handle(smartcard);
            }
            return;
        }

    /* if some errors occur */
    }else{
        /* smartcard parity error interrupt occurred */
        if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_PERR)){
            usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_PERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_PERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_PERR;
        }

        /* smartcard noise error interrupt occurred */
        if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_ERR_NERR)){
            usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_NERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_NERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_NERR;
        }

        /* smartcard frame error interrupt occurred */
        if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_ERR_FERR)){
            usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_FERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_FERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_FERR;
        }

        /* smartcard overrun interrupt occurred */
        if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_ERR_ORERR)){
            usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_ORERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_ORERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_ORERR;
        }

        /* smartcard read data buffer not empty and overrun error overrun interrupt occurred */
        if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_RBNE_ORERR)){
            usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_RBNE_ORERR);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_ORERR;
            smartcard->last_error = HAL_SMARTCARD_ERROR_ORERR;
        }

        /* smartcard receiver timeout interrupt occurred */
        if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_RT)){
            usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_RT);
            smartcard->error_state |= HAL_SMARTCARD_ERROR_RTF;
            smartcard->last_error = HAL_SMARTCARD_ERROR_RTF;
        }

        if(HAL_SMARTCARD_ERROR_NONE != smartcard->error_state){
            /* call error callback  */
            if(NULL != smartcard->smartcard_irq.error_handle){
                smartcard->smartcard_irq.error_handle(smartcard);
                smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
            }
            return;
        }
    }

    /* transmitter buffer empty interrupt handle */
    if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_TBE)){
        if(smartcard->smartcard_irq.transmit_ready_handle != NULL){
            smartcard->smartcard_irq.transmit_ready_handle(smartcard);
        }

        return;
    }

    /* transmission complete interrupt handle */
    if(RESET != usart_interrupt_flag_get(smartcard->periph, USART_INT_FLAG_TC)){
        usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_TC);

        if(NULL != smartcard->smartcard_irq.transmit_complete_handle){
            smartcard->smartcard_irq.transmit_complete_handle(smartcard);
        }

        return;
    }
}

/*!
    \brief      start smartcard module
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_start(hal_smartcard_dev_struct *smartcard)
{
    usart_enable(smartcard->periph);
}

/*!
    \brief      stop smartcard module
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_stop(hal_smartcard_dev_struct *smartcard)
{
    usart_disable(smartcard->periph);
}

/*!
    \brief      set user-defined interrupt callback function
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_irq: smartcard interrupt callback function pointer
    \param[out] none
    \retval     none
*/
void hal_smartcard_irq_handle_set(hal_smartcard_dev_struct *smartcard, hal_smartcard_irq_struct *p_irq)
{
    /* initialize smartcard error callback */
    if(NULL != p_irq->error_handle){
        smartcard->smartcard_irq.error_handle = p_irq->error_handle;
        usart_interrupt_enable(smartcard->periph, USART_INT_ERR);
        usart_interrupt_enable(smartcard->periph, USART_INT_PERR);
    }else{
        smartcard->smartcard_irq.error_handle = NULL;
        usart_interrupt_disable(smartcard->periph, USART_INT_ERR);
        usart_interrupt_disable(smartcard->periph, USART_INT_PERR);
    }

    /* initialize smartcard receive completed callback */
    if(NULL != p_irq->receive_complete_handle){
        smartcard->smartcard_irq.receive_complete_handle = p_irq->receive_complete_handle;
        usart_interrupt_enable(smartcard->periph, USART_INT_RBNE);
    }else{
        smartcard->smartcard_irq.receive_complete_handle = NULL;
        usart_interrupt_disable(smartcard->periph, USART_INT_RBNE);
    }

    /* initialize smartcard transmit completed callback */
    if(NULL != p_irq->transmit_complete_handle){
        smartcard->smartcard_irq.transmit_complete_handle = p_irq->transmit_complete_handle;
        usart_interrupt_enable(smartcard->periph, USART_INT_TC);
    }else{
        smartcard->smartcard_irq.transmit_complete_handle = NULL;
        usart_interrupt_disable(smartcard->periph, USART_INT_TC);
    }

    /* initialize smartcard transmit ready callback */
    if(NULL != p_irq->transmit_ready_handle){
        smartcard->smartcard_irq.transmit_ready_handle = p_irq->transmit_ready_handle;
        usart_interrupt_enable(smartcard->periph, USART_INT_TBE);
    }else{
        smartcard->smartcard_irq.transmit_ready_handle = NULL;
        usart_interrupt_disable(smartcard->periph, USART_INT_TBE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_irq_handle_all_reset(hal_smartcard_dev_struct *smartcard)
{
    /* configure interrupt callback function to NULL */
    smartcard->smartcard_irq.error_handle = NULL;
    smartcard->smartcard_irq.receive_complete_handle = NULL;
    smartcard->smartcard_irq.transmit_complete_handle = NULL;
    smartcard->smartcard_irq.transmit_ready_handle = NULL;
}

/*!
    \brief      transmit amounts of data by poll method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_transmit_poll(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                    uint32_t length, uint32_t timeout_ms)
{
    uint32_t tick_start;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Tx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->tx_state){
        HAL_DEBUGE("usart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* disable receive and enable transmit */
    usart_receive_config(smartcard->periph, USART_RECEIVE_DISABLE);
    usart_command_enable(smartcard->periph, USART_CMD_RXFCMD);
    usart_transmit_config(smartcard->periph, USART_TRANSMIT_ENABLE);

    /* initialize transmit parameter */
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    smartcard->txbuffer.buffer = (uint8_t *)p_buffer; 
    smartcard->txbuffer.length = length;
    smartcard->txbuffer.pos = 0U;
    smartcard->tx_state = SMARTCARD_STATE_BUSY;

    /* configure timeout */
    tick_start = hal_basetick_count_get();

    while(smartcard->txbuffer.pos < length){
        /* wait for transmit buffer empty */
        while(RESET == usart_flag_get(smartcard->periph, USART_FLAG_TBE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("smartcard transmit timeout");
                    smartcard->tx_state = SMARTCARD_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* write the data to be transmitted */
        usart_data_transmit(smartcard->periph, (*smartcard->txbuffer.buffer & (uint8_t)0xFFU));
        smartcard->txbuffer.buffer++;

        /* change the transmit pointer */
        smartcard->txbuffer.pos++;
    }

    /* wait for transmit complete */
    while(RESET == usart_flag_get(smartcard->periph, USART_FLAG_TC)){
        if(HAL_TIMEOUT_FOREVER != timeout_ms){
            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                HAL_DEBUGW("smartcard transmit timeout");
                smartcard->tx_state = SMARTCARD_STATE_FREE;
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* re-enable receive */
    usart_receive_config(smartcard->periph, USART_RECEIVE_ENABLE);

    /* change the Tx state to free */
    smartcard->tx_state = SMARTCARD_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by poll method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_receive_poll(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                   uint32_t length, uint32_t timeout_ms)
{
    uint32_t tick_start;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Rx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->rx_state){
        HAL_DEBUGE("usart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* initialize receive parameter */
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    smartcard->rxbuffer.buffer = (uint8_t *)p_buffer; 
    smartcard->rxbuffer.length = length;
    smartcard->rxbuffer.pos = 0U;
    smartcard->rx_state = SMARTCARD_STATE_BUSY;

    /* configure timeout */
    tick_start = hal_basetick_count_get();

    while(smartcard->rxbuffer.pos < length){
        /* wait for read data buffer not empty */
        while(RESET == usart_flag_get(smartcard->periph, USART_FLAG_RBNE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("usart receive timeout");
                    smartcard->rx_state = SMARTCARD_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }

        /* read data from data register */
        *smartcard->rxbuffer.buffer = (uint8_t)(usart_data_receive(smartcard->periph) & 0xFFU);
        smartcard->rxbuffer.buffer++;

        /* change the receive pointer */
        smartcard->rxbuffer.pos++;
    }

    /* change the Rx state to free */
    smartcard->rx_state = SMARTCARD_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_transmit_interrupt(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                         uint32_t length, hal_smartcard_user_cb p_user_func)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Tx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->tx_state){
        HAL_DEBUGE("usart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* disable receive and enable transmit */
    usart_receive_config(smartcard->periph, USART_RECEIVE_DISABLE);
    usart_command_enable(smartcard->periph, USART_CMD_RXFCMD);
    usart_transmit_config(smartcard->periph, USART_TRANSMIT_ENABLE);

    /* initialize transmit parameter */
    smartcard->tx_state = SMARTCARD_STATE_BUSY;
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    smartcard->txbuffer.buffer = (uint8_t *)p_buffer; 
    smartcard->txbuffer.length = length;
    smartcard->txbuffer.pos = 0U;
    smartcard->tx_callback = (void *)p_user_func;
    smartcard->smartcard_irq.transmit_ready_handle = _smartcard_transmit_interrupt;
    smartcard->smartcard_irq.transmit_complete_handle = _smartcard_transmit_complete_interrupt;

    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_TC);

    /* enable ERR(frame error) and TBE interrupt */
    usart_interrupt_enable(smartcard->periph, USART_INT_ERR);
    usart_interrupt_enable(smartcard->periph, USART_INT_TBE);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method 
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be sent received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_receive_interrupt(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                        uint32_t length, hal_smartcard_user_cb p_user_func)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Rx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->rx_state){
        HAL_DEBUGE("usart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* initialize receive parameter */
    smartcard->rx_state = SMARTCARD_STATE_BUSY;
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    smartcard->rxbuffer.buffer = (uint8_t *)p_buffer; 
    smartcard->rxbuffer.length = length;
    smartcard->rxbuffer.pos = 0U;
    smartcard->rx_callback = (void *)p_user_func;
    smartcard->smartcard_irq.receive_complete_handle = _smartcard_receive_interrupt;

    /* enable interrupt */
    usart_interrupt_enable(smartcard->periph, USART_INT_PERR);
    usart_interrupt_enable(smartcard->periph, USART_INT_ERR);
    usart_interrupt_enable(smartcard->periph, USART_INT_RBNE);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by dma method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_transmit_dma(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                              uint32_t length, hal_smartcard_user_callback_struct *p_user_func)
{
    hal_dma_irq_struct dma_irq;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Tx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->tx_state){
        HAL_DEBUGE("usart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* check whether the pointer of DMA Tx is NULL */
    if(NULL == smartcard->p_dma_tx){
        HAL_DEBUGE("parameter [smartcard->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* disable receive and enable transmit */
    usart_receive_config(smartcard->periph, USART_RECEIVE_DISABLE);
    usart_command_enable(smartcard->periph, USART_CMD_RXFCMD);
    usart_transmit_config(smartcard->periph, USART_TRANSMIT_ENABLE);

    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);

    /* initialize transmit parameter */
    smartcard->tx_state = SMARTCARD_STATE_BUSY;
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    smartcard->txbuffer.buffer = (uint8_t *)p_buffer; 
    smartcard->txbuffer.length = length;
    smartcard->txbuffer.pos = 0U;
    if(NULL != p_user_func){
        smartcard->tx_callback = (void *)p_user_func->complete_func;
        smartcard->smartcard_irq.error_handle = (hal_irq_handle_cb)p_user_func->error_func;
    }else{
        smartcard->tx_callback = NULL;
        smartcard->smartcard_irq.error_handle = NULL;
    }
    smartcard->smartcard_irq.transmit_complete_handle = _smartcard_transmit_complete_interrupt;

    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _smartcard_transmit_dma;
    dma_irq.error_handle = _smartcard_dma_error;
    if(NULL != smartcard->p_dma_tx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = smartcard->p_dma_tx->dma_irq.half_finish_handle;
    }

    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(smartcard->p_dma_tx, (uint32_t)smartcard->txbuffer.buffer, \
                            (smartcard->periph + 0x28U), length, &dma_irq);

    /* clear DMA global interrupt flag */
    dma_flag_clear(smartcard->p_dma_tx->channel, DMA_FLAG_G);

    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_TC);

    /* enable ERR interrupt(frame error) */
    usart_interrupt_enable(smartcard->periph, USART_INT_ERR);

    /* DMA enable for smartcard transmission */
    usart_dma_transmit_config(smartcard->periph, USART_DENT_ENABLE);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by dma method
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be sent received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_receive_dma(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                  uint32_t length, hal_smartcard_user_callback_struct *p_user_func)
{
    hal_dma_irq_struct dma_irq;

#if (1U == HAL_PARAMETER_CHECK)
    /* check smartcard pointer, p_buffer address and length */
    if((NULL == smartcard) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [usart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* check whether the Rx state is in ready */
    if(SMARTCARD_STATE_BUSY == smartcard->rx_state){
        HAL_DEBUGE("usart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* check whether the pointer of DMA Rx is NULL */
    if(NULL == smartcard->p_dma_rx){
        HAL_DEBUGE("parameter [smartcard->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* initialize DMA interrupt callback function structure with default value*/
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);

    /* initialize receive parameter */
    smartcard->rx_state = SMARTCARD_STATE_BUSY;
    smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    smartcard->rxbuffer.buffer = (uint8_t *)p_buffer;
    smartcard->rxbuffer.length = length;
    smartcard->rxbuffer.pos = 0U;
    if(NULL != p_user_func){
        smartcard->rx_callback = (void *)p_user_func->complete_func;
        smartcard->smartcard_irq.error_handle = (hal_irq_handle_cb)p_user_func->error_func;
    }else{
        smartcard->rx_callback = NULL;
        smartcard->smartcard_irq.error_handle = NULL;
    }

    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _smartcard_receive_dma;
    dma_irq.error_handle = _smartcard_dma_error;
    if(NULL != smartcard->p_dma_rx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = smartcard->p_dma_rx->dma_irq.half_finish_handle;
    }

    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(smartcard->p_dma_rx, (smartcard->periph + 0x24U), \
                           (uint32_t)smartcard->rxbuffer.buffer, length, &dma_irq);

    /* clear DMA global interrupt flag */
    dma_flag_clear(smartcard->p_dma_tx->channel, DMA_FLAG_G);

    /* enable the usart parity error interrupt */
    usart_interrupt_enable(smartcard->periph, USART_INT_PERR);

    /* enable the usart error interrupt: (frame error, noise error, overrun error) */
    usart_interrupt_enable(smartcard->periph, USART_INT_ERR);

    /* DMA enable for smartcard reception */
    usart_dma_receive_config(smartcard->periph, USART_DENR_ENABLE);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop smartcard transmit transfer
                the function is blocking
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_transmit_stop(hal_smartcard_dev_struct *smartcard)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == smartcard){
        HAL_DEBUGE("parameter [smartcard] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* disable the TBE and TC interrupt */
    usart_interrupt_disable(smartcard->periph, USART_INT_TBE);
    usart_interrupt_disable(smartcard->periph, USART_INT_TC);

    /* check receive state, if free then disable ERR interrupt */
    if(SMARTCARD_STATE_FREE == smartcard->rx_state){
        usart_interrupt_disable(smartcard->periph, USART_INT_ERR);
    }

    /* disable DMA transimt and stop DMA */
    usart_dma_transmit_config(smartcard->periph, USART_DENT_DISABLE);
    hal_dma_stop(smartcard->p_dma_tx);

    /* reset the position and state */
    smartcard->txbuffer.pos = 0U;
    smartcard->tx_state = SMARTCARD_STATE_FREE;

    /* clear interrupt error flags */
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_FERR);

    return HAL_ERR_NONE;
}

/*!
    \brief      stop smartcard receive transfer
                the function is blocking
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_smartcard_receive_stop(hal_smartcard_dev_struct *smartcard)
{
#if (1U == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == smartcard){
        HAL_DEBUGE("parameter [smartcard] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1U == HAL_PARAMETER_CHECK */

    /* disable the RBNE, PERR, ERR, EB and RT interrupt */
    usart_interrupt_disable(smartcard->periph, USART_INT_RBNE);
    usart_interrupt_disable(smartcard->periph, USART_INT_PERR);
    usart_interrupt_disable(smartcard->periph, USART_INT_EB);
    usart_interrupt_disable(smartcard->periph, USART_INT_RT);

    /* check transmit state, if free then disable ERR interrupt */
    if(SMARTCARD_STATE_FREE == smartcard->tx_state){
        usart_interrupt_disable(smartcard->periph, USART_INT_ERR);
    }

    /* disable DMA receive and stop DMA */
    usart_dma_receive_config(smartcard->periph, USART_DENR_DISABLE);
    hal_dma_stop(smartcard->p_dma_rx);

    /* reset the position and state */
    smartcard->rxbuffer.pos = 0U;
    smartcard->rx_state = SMARTCARD_STATE_FREE;

    /* clear interrupt error flags */
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_PERR);
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_FERR);
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_NERR);
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_ERR_ORERR);
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_EB);
    usart_interrupt_flag_clear(smartcard->periph, USART_INT_FLAG_RT);

    return HAL_ERR_NONE;
}

/*!
    \brief      dynamic update the smartcard block length
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  block_length: 0x00000000-0x000000FF
    \param[out] none
    \retval     none
*/
void hal_smartcard_block_length_set(hal_smartcard_dev_struct *smartcard, uint32_t block_length)
{
    USART_RT(smartcard->periph) &= ~(USART_RT_BL);
    USART_RT(smartcard->periph) |= (USART_RT_BL & ((block_length) << 24U));
}

/*!
    \brief      dynamic update the smartcard receiver timeout value
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  rtimeout: 0x00000000-0x00FFFFFF, receiver timeout value in terms of number of baud clocks
    \param[out] none
    \retval     none
*/
void hal_smartcard_receiver_timeout_set(hal_smartcard_dev_struct *smartcard, uint32_t rtimeout)
{
    USART_RT(smartcard->periph) &= ~(USART_RT_RT);
    USART_RT(smartcard->periph) |= rtimeout;
}

/*!
    \brief      enable the smartcard receiver timeout
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_receiver_timeout_enable(hal_smartcard_dev_struct *smartcard)
{
    USART_CTL1(smartcard->periph) |= USART_CTL1_RTEN;
}

/*!
    \brief      disable the smartcard receiver timeout
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smartcard_receiver_timeout_disable(hal_smartcard_dev_struct *smartcard)
{
    USART_CTL1(smartcard->periph) &= ~(USART_CTL1_RTEN);
}

/*!
    \brief      get smartcard error status
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error status: RESET or SET
*/
static FlagStatus _smartcard_error_flag_get(hal_smartcard_dev_struct *smartcard)
{
    if(0U == (USART_STAT(smartcard->periph) & (uint32_t)(USART_STAT_PERR | USART_STAT_FERR | \
             USART_STAT_ORERR | USART_STAT_NERR | USART_STAT_RTF))){
        return RESET;
    }else{
        return SET;
    }
}

/*!
    \brief      smartcard transmit complete interrupt handler
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smartcard_transmit_complete_interrupt(void *smartcard)
{
    hal_smartcard_dev_struct *p_smartcard = smartcard;
    hal_smartcard_user_cb p_func = (hal_smartcard_user_cb)p_smartcard->tx_callback;

    /* disable usart transmit complete interrupt */
    usart_interrupt_disable(p_smartcard->periph, USART_INT_TC);

    /* check receive state, if free then disable ERR interrupt */
    if(SMARTCARD_STATE_FREE == p_smartcard->rx_state){
        usart_interrupt_disable(p_smartcard->periph, USART_INT_ERR);
    }

    /* clear transmit complete callback pointer */
    p_smartcard->smartcard_irq.transmit_complete_handle = NULL;

    /* re-enable receive */
    usart_receive_config(p_smartcard->periph, USART_RECEIVE_ENABLE);

    /* change the Tx state to free */
    p_smartcard->tx_state = SMARTCARD_STATE_FREE;

    if(NULL != p_func){
        p_func(p_smartcard);
    }
}

/*!
    \brief      smartcard transmit interrupt handler
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smartcard_transmit_interrupt(void *smartcard)
{
    hal_smartcard_dev_struct *p_smartcard = smartcard;
    uint32_t length = p_smartcard->txbuffer.length;

    /* Check the remain data to be transmitted */
    if(p_smartcard->txbuffer.pos < length){
        usart_data_transmit(p_smartcard->periph, (*p_smartcard->txbuffer.buffer & (uint8_t)0xFFU));
        p_smartcard->txbuffer.buffer++;
        p_smartcard->txbuffer.pos++;
    }else{
        /* disable TBE interrupt and enable TC interrupt */
        usart_interrupt_disable(p_smartcard->periph, USART_INT_TBE);
        usart_interrupt_enable(p_smartcard->periph, USART_INT_TC);

        p_smartcard->smartcard_irq.transmit_ready_handle = NULL;
    }
}

/*!
    \brief      smartcard receive interrupt handler
    \param[in]  smartcard: smartcard device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smartcard_receive_interrupt(void *smartcard)
{
    hal_smartcard_dev_struct *p_smartcard = smartcard;
    uint32_t length = p_smartcard->rxbuffer.length;

    /* read data from data register */
    *p_smartcard->rxbuffer.buffer = (uint8_t)(usart_data_receive(p_smartcard->periph) & 0xFFU);
    p_smartcard->rxbuffer.buffer++;
    p_smartcard->rxbuffer.pos++;

    if(p_smartcard->rxbuffer.pos == length){
        hal_smartcard_user_cb p_func = (hal_smartcard_user_cb)p_smartcard->rx_callback;

        /* disable PERR and RBNE interrupt */
        usart_interrupt_disable(p_smartcard->periph, USART_INT_PERR);
        usart_interrupt_disable(p_smartcard->periph, USART_INT_RBNE);

        /* check transmit state, if free then disable ERR interrupt */
        if(SMARTCARD_STATE_FREE == p_smartcard->tx_state){
            usart_interrupt_disable(p_smartcard->periph, USART_INT_ERR);
        }

        /* reset receive_complete_handle */
        p_smartcard->smartcard_irq.receive_complete_handle = NULL;

        /* change the Rx state to free */
        p_smartcard->rx_state = SMARTCARD_STATE_FREE;

        if(NULL != p_func){
            p_func(p_smartcard);
        }
    }
}

/*!
    \brief      smartcard DMA transmit handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _smartcard_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_smartcard_dev_struct *p_smartcard;

    p_dma = (hal_dma_dev_struct*)dma;
    p_smartcard = (hal_smartcard_dev_struct*)p_dma->p_periph;

    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_smartcard->txbuffer.pos = p_smartcard->txbuffer.length;

        /* disable the dma transfer for transmit request */
        usart_dma_transmit_config(p_smartcard->periph, USART_DENT_DISABLE);

        /* enable TC interrupt */
        usart_interrupt_enable(p_smartcard->periph, USART_INT_TC);
    }
}

/*!
    \brief      smartcard DMA receive handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _smartcard_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_smartcard_dev_struct *p_smartcard;
    hal_smartcard_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_smartcard = (hal_smartcard_dev_struct*)p_dma->p_periph;
    p_func = (hal_smartcard_user_cb)p_smartcard->rx_callback;

    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_smartcard->rxbuffer.pos = p_smartcard->rxbuffer.length;

        /* disable the dma transfer for the receiver request */
        usart_dma_receive_config(p_smartcard->periph, USART_DENR_DISABLE);

        /* disable PERR and ERR(frame error, noise error, overrun error) interrupts */
        usart_interrupt_disable(p_smartcard->periph, USART_INT_PERR);
        usart_interrupt_disable(p_smartcard->periph, USART_INT_ERR);

        /* change the Rx state to free */
        p_smartcard->rx_state = SMARTCARD_STATE_FREE;
    }

    if(NULL != p_func){
        p_func(p_smartcard);
    }
}

/*!
    \brief      smartcard dma communication error
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _smartcard_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_smartcard_dev_struct *p_smartcard;

    p_dma = (hal_dma_dev_struct*)dma;
    p_smartcard = (hal_smartcard_dev_struct*)p_dma->p_periph;

    if(SMARTCARD_STATE_BUSY == p_smartcard->tx_state){
        p_smartcard->error_state |= HAL_SMARTCARD_ERROR_DMATX;
        p_smartcard->last_error = HAL_SMARTCARD_ERROR_DMATX;
        p_smartcard->txbuffer.pos = p_smartcard->txbuffer.length;

        /* disable the dma transmit */
        usart_dma_transmit_config(p_smartcard->periph, USART_DENT_DISABLE);

        /* disable and ERR(frame error) interrupt */
        usart_interrupt_disable(p_smartcard->periph, USART_INT_ERR);

        /* change the Tx state to free */
        p_smartcard->tx_state = SMARTCARD_STATE_FREE;
    }else if(SMARTCARD_STATE_BUSY == p_smartcard->rx_state){
        p_smartcard->error_state |= HAL_SMARTCARD_ERROR_DMARX;
        p_smartcard->last_error = HAL_SMARTCARD_ERROR_DMARX;
        p_smartcard->rxbuffer.pos = p_smartcard->rxbuffer.length;

        /* disable the dma receive */
        usart_dma_receive_config(p_smartcard->periph, USART_DENR_DISABLE);

        /* disable PERR and ERR(frame error, noise error, overrun error) interrupts */
        usart_interrupt_disable(p_smartcard->periph, USART_INT_PERR);
        usart_interrupt_disable(p_smartcard->periph, USART_INT_ERR);

        /* change the Rx state to free */
        p_smartcard->rx_state = SMARTCARD_STATE_FREE;
    }else{
        HAL_DEBUGE("smartcard processor fatal error: dma error exception due to run state");
    }

    if(p_smartcard->smartcard_irq.error_handle != NULL){
        p_smartcard->smartcard_irq.error_handle(p_smartcard);

        /* change the error state to none */
        p_smartcard->error_state = HAL_SMARTCARD_ERROR_NONE;
    }
}
