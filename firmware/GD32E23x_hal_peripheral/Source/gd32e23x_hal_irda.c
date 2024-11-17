/*!
    \file    gd32e23x_hal_irda.c
    \brief   IRDA driver
    
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

static uint16_t _irda_data_bit_mask_get(hal_irda_dev_struct *irda);
static FlagStatus _irda_error_flag_get(hal_irda_dev_struct *irda);
static void _irda_transmit_complete_interrupt(void *irda);
static void _irda_transmit_interrupt(void *irda);
static void _irda_receive_interrupt(void *irda);
static void _irda_transmit_dma(void *dma);
static void _irda_receive_dma(void *dma);
static void _irda_dma_error(void *dma);

/*!
    \brief      initialize the IrDA struct with the default values, note that this function must be
                called after the structure is created
    \param[in]  hal_struct_type: type of IrDA struct for initialization
      \arg        HAL_IRDA_INIT_STRUCT: initialization structure
      \arg        HAL_IRDA_DEV_STRUCT: device information structure
      \arg        HAL_IRDA_USER_CALLBCAK_STRUCT: user callback struct
    \param[in]  p_struct: structure pointer
    \param[out] none
    \retval     none
*/
void hal_irda_struct_init(hal_irda_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(hal_struct_type){
    case HAL_IRDA_INIT_STRUCT:
        /* initialize IrDA initialization structure with the default values */
        ((hal_irda_init_struct*)p_struct)->baudrate = 115200U;
        ((hal_irda_init_struct*)p_struct)->direction = IRDA_DIRECTION_RX_TX;
        ((hal_irda_init_struct*)p_struct)->parity = IRDA_PARITY_NONE;
        ((hal_irda_init_struct*)p_struct)->word_length = IRDA_WORD_LENGTH_8BIT;
        ((hal_irda_init_struct*)p_struct)->mode = IRDA_NORMAL_MODE;
        ((hal_irda_init_struct*)p_struct)->prescaler = 1U;
        break;
        
    case HAL_IRDA_DEV_STRUCT:
        /* initialize IrDA device information structure with the default values */
        ((hal_irda_dev_struct*)p_struct)->periph = 0U;
        ((hal_irda_dev_struct*)p_struct)->irda_irq.error_handle = NULL;
        ((hal_irda_dev_struct*)p_struct)->irda_irq.receive_complete_handle = NULL;
        ((hal_irda_dev_struct*)p_struct)->irda_irq.transmit_complete_handle = NULL;
        ((hal_irda_dev_struct*)p_struct)->irda_irq.transmit_ready_handle = NULL;
        ((hal_irda_dev_struct*)p_struct)->p_dma_rx = NULL;
        ((hal_irda_dev_struct*)p_struct)->p_dma_tx = NULL;
        ((hal_irda_dev_struct*)p_struct)->txbuffer.buffer = NULL;
        ((hal_irda_dev_struct*)p_struct)->txbuffer.length = 0U;
        ((hal_irda_dev_struct*)p_struct)->txbuffer.pos = 0U;
        ((hal_irda_dev_struct*)p_struct)->rxbuffer.buffer = NULL;
        ((hal_irda_dev_struct*)p_struct)->rxbuffer.length = 0U;
        ((hal_irda_dev_struct*)p_struct)->rxbuffer.pos = 0U;
        ((hal_irda_dev_struct*)p_struct)->error_state = HAL_USART_ERROR_NONE;
        ((hal_irda_dev_struct*)p_struct)->tx_state = IRDA_STATE_FREE;
        ((hal_irda_dev_struct*)p_struct)->rx_state = IRDA_STATE_FREE;
        ((hal_irda_dev_struct*)p_struct)->last_error = HAL_USART_ERROR_NONE;
        ((hal_irda_dev_struct*)p_struct)->priv = NULL;      /* priv data */

    case HAL_IRDA_USER_CALLBCAK_STRUCT:
        /* initialize user callback structure with the default values */
        ((hal_irda_user_callback_struct*)p_struct)->complete_func = NULL;
        ((hal_irda_user_callback_struct*)p_struct)->error_func = NULL;
        break;
    
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize the IrDA
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_irda_deinit(hal_irda_dev_struct *irda)
{
    uint32_t periph;
    
    periph = irda->periph;
    if(USART0 == periph){
        /* deinitialize the periph and the device information sturcture */
        usart_deinit(periph);
        hal_irda_struct_init(HAL_IRDA_DEV_STRUCT, irda);
        irda->periph = periph;
    }else{
        HAL_DEBUGE("parameter [irda->periph] value is invalid");
    }
}

/*!
    \brief      initialize the IrDA with specified values
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is initialized, USART0 only
    \param[in]  p_init: the initialization data needed to initialize IrDA
                  baudrate: communication baudrate
                  parity: IRDA_PARITY_NONE, IRDA_PARITY_EVEN, IRDA_PARITY_ODD
                  word_length: IRDA_WORD_LENGTH_8BIT, IRDA_WORD_LENGTH_9BIT
                  direction: IRDA_DIRECTION_RX_TX, IRDA_DIRECTION_RX_ONLY, IRDA_DIRECTION_TX_ONLY
                  mode: IRDA_NORMAL_MODE, IRDA_LOW_POWER_MODE
                  prescaler: 1 - 255
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_irda_init(hal_irda_dev_struct *irda, uint32_t periph, \
                      hal_irda_init_struct *p_init)
{
    uint32_t reg_temp;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check IrDA pointer and p_init address */
    if((NULL == irda) || (NULL == p_init)){
        HAL_DEBUGE("pointer [irda] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check periph parameter */
    if(USART0 != periph){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }
    
    /* check periph value from IrDA device struct */
    if(0U != irda->periph){
        HAL_DEBUGI("periph value from irda device struct has been rewrite");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    irda->periph = periph;
    usart_disable(periph);
    
    /* CTL0 register configure */
    reg_temp = USART_CTL0(periph);
    reg_temp &= ~(USART_CTL0_PM | USART_CTL0_PCEN | USART_CTL0_WL | \
                  USART_CTL0_REN | USART_CTL0_TEN);
    reg_temp |= (p_init->direction | p_init->parity | p_init->word_length);
    USART_CTL0(periph) = reg_temp;
    
    /* boudrate configure */
    usart_baudrate_set(periph, p_init->baudrate);
    irda->data_bit_mask = _irda_data_bit_mask_get(irda);
    
    /* reset the Rx and Tx state */
    irda->tx_state = IRDA_STATE_FREE;
    irda->rx_state = IRDA_STATE_FREE;
    
    /* configure the IrDA low power mode */
    usart_irda_lowpower_config(periph, p_init->mode);
    usart_irda_mode_enable(periph);
    
    return HAL_ERR_NONE;
}
 
/*!
    \brief      start IrDA module, note that this function must be called to start USART after initialization
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_irda_start(hal_irda_dev_struct *irda)
{
    usart_enable(irda->periph);
}

/*!
    \brief      stop IrDA module
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_irda_stop(hal_irda_dev_struct *irda)
{
    usart_disable(irda->periph);
}

/*!
    \brief      handle the IrDA interrupts
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_irda_irq(hal_irda_dev_struct *irda)
{
    if(RESET == _irda_error_flag_get(irda)){
        /* check whether USART is in receiver mode or not */
        if(RESET != usart_interrupt_flag_get(irda->periph, USART_INT_FLAG_RBNE)){
            if(NULL != irda->irda_irq.receive_complete_handle){
                irda->irda_irq.receive_complete_handle(irda);
            }
            return;
        }
    }else{
        /* check whether the PERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(irda->periph, USART_INT_FLAG_PERR)){
            usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_PERR);
            irda->error_state |= HAL_USART_ERROR_PERR;
            irda->last_error = HAL_USART_ERROR_PERR;
        }
        
        /* check whether the NERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(irda->periph, USART_INT_FLAG_ERR_NERR)){
            usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_NERR);
            irda->error_state |= HAL_USART_ERROR_NERR;
            irda->last_error = HAL_USART_ERROR_NERR;
        }
        
        /* check whether the FERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(irda->periph, USART_INT_FLAG_ERR_FERR)){
            usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_FERR);
            irda->error_state |= HAL_USART_ERROR_FERR;
            irda->last_error = HAL_USART_ERROR_FERR;
        }
        
        /* check whether the ERR ORERR is set or not */
        if(RESET != usart_interrupt_flag_get(irda->periph, USART_INT_FLAG_ERR_ORERR)){ 
            usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_ORERR);
            irda->error_state |= HAL_USART_ERROR_ORERR;
            irda->last_error = HAL_USART_ERROR_ORERR;
        }
        
        /* check whether RBNE ORERR is set or not */
        if(RESET != usart_interrupt_flag_get(irda->periph, USART_INT_FLAG_RBNE_ORERR)){ 
            usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_RBNE_ORERR);
            irda->error_state |= HAL_USART_ERROR_ORERR;
            irda->last_error = HAL_USART_ERROR_ORERR;
        }
       
        /* check whether error state is none or not */
        if(HAL_USART_ERROR_NONE != irda->error_state){
            if(NULL != irda->irda_irq.error_handle){
                irda->irda_irq.error_handle(irda);
                irda->error_state = HAL_USART_ERROR_NONE;
            }
            return;
        }
    }
    
    /* transmitter buffer empty interrupt handle */
    if(RESET != usart_interrupt_flag_get(irda->periph, USART_INT_FLAG_TBE)){
        if(NULL != irda->irda_irq.transmit_ready_handle){
            irda->irda_irq.transmit_ready_handle(irda);
        }
        return;
    }
    
    /* transmission complete interrupt handle */
    if(RESET != usart_interrupt_flag_get(irda->periph, USART_INT_FLAG_TC)){
        usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_TC);
        
        if(NULL != irda->irda_irq.transmit_complete_handle){
            irda->irda_irq.transmit_complete_handle(irda);
        }
        return;
    }
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  irda: IrDA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to IrDA interrupt callback functions structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_irda_irq_handle_set(hal_irda_dev_struct *irda, hal_irda_irq_struct *p_irq)
{
    /* set user-defined error interrupt callback */
    if(NULL != p_irq->error_handle){
        irda->irda_irq.error_handle = p_irq->error_handle;
        usart_interrupt_enable(irda->periph, USART_INT_ERR);
        usart_interrupt_enable(irda->periph, USART_INT_PERR);
    }else{
        irda->irda_irq.error_handle = NULL;
        usart_interrupt_disable(irda->periph, USART_INT_ERR);
        usart_interrupt_disable(irda->periph, USART_INT_PERR);
    }
    
    /* set user-defined receive complete interrupt callback */
    if(NULL != p_irq->receive_complete_handle){
        irda->irda_irq.receive_complete_handle = p_irq->receive_complete_handle;
        usart_interrupt_enable(irda->periph, USART_INT_RBNE);
    }else{
        irda->irda_irq.receive_complete_handle = NULL;
        usart_interrupt_disable(irda->periph, USART_INT_RBNE);
    }
    
    /* set user-defined transmit complete interrupt callback */
    if(NULL != p_irq->transmit_complete_handle){
        irda->irda_irq.transmit_complete_handle = p_irq->transmit_complete_handle;
        usart_interrupt_enable(irda->periph, USART_INT_TC);
    }else{
        irda->irda_irq.transmit_complete_handle = NULL;
        usart_interrupt_disable(irda->periph, USART_INT_TC);
    }
    
    /* set user-defined transmit ready interrupt callback */
    if(NULL != p_irq->transmit_ready_handle){
        irda->irda_irq.transmit_ready_handle = p_irq->transmit_ready_handle;
        usart_interrupt_enable(irda->periph, USART_INT_TBE);
    }else{
        irda->irda_irq.transmit_ready_handle = NULL;
        usart_interrupt_disable(irda->periph, USART_INT_TBE);
    }    
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_irda_irq_handle_all_reset(hal_irda_dev_struct *irda)
{
    /* configure interrupt callback function to NULL */
    irda->irda_irq.error_handle = NULL;
    irda->irda_irq.receive_complete_handle = NULL;
    irda->irda_irq.transmit_complete_handle = NULL;
    irda->irda_irq.transmit_ready_handle = NULL;
}

/*!
    \brief      transmit amounts of data, poll transmit process and completed status
                the function is blocking
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_irda_transmit_poll(hal_irda_dev_struct *irda, void *p_buffer, \
                                uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    uint32_t temp_val;
    uint32_t tick_start;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state){
        HAL_DEBUGE("irda tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_WL)){
        if(RESET == (USART_CTL0(irda->periph) & USART_CTL0_PCEN)){
            data_length = 2U;
        }
    }
    
    /* initialize transmit parameters */
    irda->error_state = HAL_USART_ERROR_NONE;
    irda->txbuffer.buffer = (uint8_t *)p_buffer; 
    irda->txbuffer.length = length;
    irda->txbuffer.pos = 0U;
    irda->tx_state = IRDA_STATE_BUSY;
    /* configure timeout */
    tick_start = hal_basetick_count_get();
    
    temp_val = irda->txbuffer.pos;
    while(temp_val < irda->txbuffer.length){
        /* wait for transmit buffer empty */
        while(RESET == usart_flag_get(irda->periph, USART_FLAG_TBE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("irda transmit timeout");
                    /* reset the state */
                    irda->rx_state = IRDA_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        
        /* send the data to be transmitted */
        if(1U == data_length){
            usart_data_transmit(irda->periph, (*irda->txbuffer.buffer & (uint8_t)0xFFU));
            irda->txbuffer.buffer++;
        }else{
            usart_data_transmit(irda->periph, (*(uint16_t*)irda->txbuffer.buffer & (uint16_t)0x1FFU));
            irda->txbuffer.buffer += data_length;
        }
        
        /* change the transmit pointer */
        irda->txbuffer.pos++;
        temp_val = irda->txbuffer.pos;
    }
    
    /* wait for transmit complete */
    while(RESET == usart_flag_get(irda->periph, USART_FLAG_TC)){
        if(HAL_TIMEOUT_FOREVER != timeout_ms){
            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                HAL_DEBUGW("irda transmit timeout");
                return HAL_ERR_TIMEOUT;
            }
        }
    }
    
    /* change the Tx state to free */
    irda->tx_state = IRDA_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data, poll receive process and completed status
                the function is blocking
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_irda_receive_poll(hal_irda_dev_struct *irda, void *p_buffer, \
                               uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    uint32_t temp_val;
    uint32_t tick_start;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state){
        HAL_DEBUGE("irda rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_WL)){
        if(RESET == (USART_CTL0(irda->periph) & USART_CTL0_PCEN)){
            data_length = 2U;
        }
    }
    
    /* initialize receive parameters */
    irda->error_state = HAL_USART_ERROR_NONE;
    irda->rxbuffer.buffer = (uint8_t *)p_buffer; 
    irda->rxbuffer.length = length;
    irda->rxbuffer.pos = 0U;
    irda->data_bit_mask = _irda_data_bit_mask_get(irda);
    irda->rx_state = IRDA_STATE_BUSY;
    /* configure timeout */
    tick_start = hal_basetick_count_get();
    
    temp_val = irda->rxbuffer.pos;
    while(temp_val < irda->rxbuffer.length){
        /* wait for read data buffer not empty */
        while(RESET == usart_flag_get(irda->periph, USART_FLAG_RBNE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("irda receive timeout");
                    /* reset the state */
                    irda->rx_state = IRDA_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        
        /* read data from data register */
        if(1U == data_length){
            *irda->rxbuffer.buffer = (uint8_t)(usart_data_receive(irda->periph) & irda->data_bit_mask);
            irda->rxbuffer.buffer++;
        }else{
            *(uint16_t*)irda->rxbuffer.buffer = (usart_data_receive(irda->periph) & irda->data_bit_mask);
             irda->rxbuffer.buffer += data_length;
        }
        
        /* change the receive pointer */
        irda->rxbuffer.pos++;
        temp_val = irda->rxbuffer.pos;
    }
    
    /* change the Rx state to free */
    irda->rx_state = IRDA_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method, after the transfer is completed, 
                the user function is called
                the function is non-blocking
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_irda_transmit_interrupt(hal_irda_dev_struct *irda, void *p_buffer, \
                                     uint32_t length, hal_irda_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state){
        HAL_DEBUGE("irda tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize transmit parameters */
    irda->tx_state = IRDA_STATE_BUSY;
    irda->error_state = HAL_USART_ERROR_NONE;
    irda->txbuffer.buffer = (uint8_t *)p_buffer; 
    irda->txbuffer.length = length;
    irda->txbuffer.pos = 0U;
    irda->tx_callback = (void *)p_user_func;
    /* configure the transmit ready and complete callback as the function implemented */
    irda->irda_irq.transmit_ready_handle = _irda_transmit_interrupt;
    irda->irda_irq.transmit_complete_handle = _irda_transmit_complete_interrupt;
    
    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_TC);

    /* enable the TBE interrupt */
    usart_interrupt_enable(irda->periph, USART_INT_TBE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method, after the transfer is completed, 
                the user function is called
                the function is non-blocking
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_irda_receive_interrupt(hal_irda_dev_struct *irda, void *p_buffer, \
                                    uint32_t length, hal_irda_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state){
        HAL_DEBUGE("irda rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize receive parameters */
    irda->rx_state = IRDA_STATE_BUSY;
    irda->error_state = HAL_USART_ERROR_NONE;
    irda->rxbuffer.buffer = (uint8_t *)p_buffer; 
    irda->rxbuffer.length = length;
    irda->rxbuffer.pos = 0U;
    irda->rx_callback = (void *)p_user_func;
    irda->irda_irq.receive_complete_handle = _irda_receive_interrupt;
    irda->data_bit_mask = _irda_data_bit_mask_get(irda);
    
    /* enable PERR, ERR, RBNE interrupt */
    usart_interrupt_enable(irda->periph, USART_INT_PERR);
    usart_interrupt_enable(irda->periph, USART_INT_ERR);
    usart_interrupt_enable(irda->periph, USART_INT_RBNE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by DMA method, after the transfer is completed or error occurs, 
                the user function is called
                the function is non-blocking
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_irda_transmit_dma(hal_irda_dev_struct *irda, void *p_buffer, \
                              uint32_t length, hal_irda_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameter */
    if(NULL == irda->p_dma_tx){
        HAL_DEBUGE("parameter [irda->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state){
        HAL_DEBUGE("irda tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    
    /* initialize transmit parameters */
    irda->tx_state = IRDA_STATE_BUSY;
    irda->error_state = HAL_USART_ERROR_NONE;
    irda->txbuffer.buffer = (uint8_t *)p_buffer; 
    irda->txbuffer.length = length;
    irda->txbuffer.pos = 0U;
    if(NULL != p_func){
        irda->tx_callback = (void *)p_func->complete_func;
        irda->irda_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    }else{
        irda->tx_callback = NULL;
        irda->irda_irq.error_handle = NULL;
    }
    irda->irda_irq.transmit_complete_handle = _irda_transmit_complete_interrupt;
    
    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _irda_transmit_dma;
    dma_irq.error_handle = _irda_dma_error;
    if(NULL != irda->p_dma_tx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = irda->p_dma_tx->dma_irq.half_finish_handle;
    }
    
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(irda->p_dma_tx, (uint32_t)irda->txbuffer.buffer, \
                           irda->periph + 0x28U, length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(irda->p_dma_tx->channel, DMA_FLAG_G);
    
    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_TC);

    /* DMA enable for transmission */
    usart_dma_transmit_config(irda->periph, USART_DENT_ENABLE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by DMA method, after the transfer is completed or error occurs, 
                the user function is called
                the function is non-blocking
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_irda_receive_dma(hal_irda_dev_struct *irda, void *p_buffer, \
                              uint32_t length, hal_irda_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == irda) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [irda] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameter */
    if(NULL == irda->p_dma_rx){
        HAL_DEBUGE("parameter [irda->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state){
        HAL_DEBUGE("irda rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    
    /* initialize receive parameters */
    irda->rx_state = IRDA_STATE_BUSY;
    irda->error_state = HAL_USART_ERROR_NONE;
    irda->rxbuffer.buffer = (uint8_t *)p_buffer; 
    irda->rxbuffer.length = length;
    irda->rxbuffer.pos = 0U;
    if(NULL != p_func){
        irda->rx_callback = (void *)p_func->complete_func;
        irda->irda_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    }else{
        irda->rx_callback = NULL;
        irda->irda_irq.error_handle = NULL;
    }
    irda->data_bit_mask = _irda_data_bit_mask_get(irda);
    
    /* enable the usart parity error and error interrupt: (frame error, noise error, overrun error)  */
    usart_interrupt_enable(irda->periph, USART_INT_PERR);
    usart_interrupt_enable(irda->periph, USART_INT_ERR);
    
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _irda_receive_dma;
    dma_irq.error_handle = _irda_dma_error;
    if(NULL != irda->p_dma_rx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = irda->p_dma_rx->dma_irq.half_finish_handle;
    }
    
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(irda->p_dma_rx, irda->periph + 0x24U, \
                           (uint32_t)irda->rxbuffer.buffer, length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(irda->p_dma_tx->channel, DMA_FLAG_G);
    /* DMA enable for reception */
    usart_dma_receive_config(irda->periph, USART_DENR_ENABLE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      pause IrDA DMA transfer during transmission process
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_irda_dma_pause(hal_irda_dev_struct *irda)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda){
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state){
        /* disable DMA transimt */
        usart_dma_transmit_config(irda->periph, USART_DENT_DISABLE);
    }
    
    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state){
        /* disable the PERR and ERR interrupt */
        usart_interrupt_disable(irda->periph, USART_INT_PERR);
        usart_interrupt_disable(irda->periph, USART_INT_ERR);
        
        /* disable DMA receive */
        usart_dma_receive_config(irda->periph, USART_DENR_DISABLE);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      resume IrDA DMA transfer during transmission process
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_irda_dma_resume(hal_irda_dev_struct *irda)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda){
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->tx_state){
        /* enable DMA transimt */
        usart_dma_transmit_config(irda->periph, USART_DENT_ENABLE);
    }
    
    /* check the rx_state wheher is busy or not */
    if(IRDA_STATE_BUSY == irda->rx_state){
        /* enable the PERR and ERR interrupt */
        usart_interrupt_enable(irda->periph, USART_INT_PERR);
        usart_interrupt_enable(irda->periph, USART_INT_ERR);
        
        /* enable DMA receive */
        usart_dma_receive_config(irda->periph, USART_DENR_ENABLE);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      stop IrDA transmit transfer
                the function is blocking
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_irda_transmit_stop(hal_irda_dev_struct *irda)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda){
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable the TBE and TC interrupt */
    usart_interrupt_disable(irda->periph, USART_INT_TBE);
    usart_interrupt_disable(irda->periph, USART_INT_TC);
    
    /* disable DMA transimt and stop DMA */
    usart_dma_transmit_config(irda->periph, USART_DENT_DISABLE);
    hal_dma_stop(irda->p_dma_tx);
    
    /* reset the position and state */
    irda->txbuffer.pos = 0;
    irda->tx_state = IRDA_STATE_FREE;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      stop IrDA receive transfer
                the function is blocking
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_irda_receive_stop(hal_irda_dev_struct *irda)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda){
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable the RBNE, PERR and ERR interrupt */
    usart_interrupt_disable(irda->periph, USART_INT_RBNE);
    usart_interrupt_disable(irda->periph, USART_INT_PERR);
    usart_interrupt_disable(irda->periph, USART_INT_ERR);
    
    /* disable DMA receive and stop DMA */
    usart_dma_receive_config(irda->periph, USART_DENR_DISABLE);
    hal_dma_stop(irda->p_dma_rx);
    
    /* reset the position and state */
    irda->rxbuffer.pos = 0;
    irda->rx_state = IRDA_STATE_FREE;
    
    /* clear interrupt error flags */
    usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_PERR);
    usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_FERR);
    usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_NERR);
    usart_interrupt_flag_clear(irda->periph, USART_INT_FLAG_ERR_ORERR);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      configure the IrDA baudrate and prescaler, the other parameters are configured as default values
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is configured, USART0 only
    \param[in]  baud: baudrate
    \param[in]  psc: prescaler, can be set to 1 only in IrDA normal mode
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_irda_simple_config(hal_irda_dev_struct *irda, uint32_t periph, uint32_t baud, uint8_t psc)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == irda){
        HAL_DEBUGE("parameter [irda] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameters */
    if((USART0 != periph) || (0U == baud)){
        HAL_DEBUGE("parameter [periph] or [baud] value is invalid"); 
        return HAL_ERR_VAL;
    }
    
    /* check the parameters */
    if(0U == psc){
        HAL_DEBUGE("parameter [psc] value is invalid"); 
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state or rx_state wheher is busy or not */
    if((IRDA_STATE_BUSY == irda->tx_state) || (IRDA_STATE_BUSY == irda->rx_state)){
        HAL_DEBUGE("IrDA is in tx or rx state, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    irda->periph = periph;
    
    /* deinitilize USART */
    usart_deinit(periph);
    /* USART configure */
    usart_baudrate_set(periph, baud);
    usart_parity_config(periph, USART_PM_NONE);
    usart_word_length_set(periph, USART_WL_8BIT);
    usart_stop_bit_set(periph, USART_STB_1BIT);
    usart_receive_config(periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(periph, USART_TRANSMIT_ENABLE);
    /* IrDA configure */
    usart_irda_lowpower_config(periph, USART_IRLP_NORMAL);
    usart_prescaler_config(periph, psc);
    usart_irda_mode_enable(periph);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      get the mask of date bit
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the mask of date bit(0x7F, 0xFF, 0x1FF)
*/
static uint16_t _irda_data_bit_mask_get(hal_irda_dev_struct *irda)
{
    uint16_t reval;
    
    if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_WL)){
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_PCEN)){
            reval = 0xFFU;
        }else{
            reval = 0x1FFU;
        }
    }else{
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(irda->periph) & USART_CTL0_PCEN)){
            reval = 0x7FU;
        }else{
            reval = 0xFFU;
        }
    }
    
    return reval;
}

/*!
    \brief      get IrDA error flag
    \param[in]  irda: IrDA device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     SET or RESET
*/
static FlagStatus _irda_error_flag_get(hal_irda_dev_struct *irda)
{
    if(0U == (USART_STAT(irda->periph) & (uint32_t)(USART_STAT_PERR | USART_STAT_FERR | \
                                                    USART_STAT_ORERR | USART_STAT_NERR))){
        return RESET;
    }else{
        return SET;
    }
}

/*!
    \brief      handle the transmit complete interrupt
    \param[in]  irda: pointer to a IrDA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_transmit_complete_interrupt(void *irda)
{
    hal_irda_dev_struct *p_irda = irda;
    hal_irda_user_cb p_func = (hal_irda_user_cb)p_irda->tx_callback;
    
    /* disable the transmit complete interrupt */
    usart_interrupt_disable(p_irda->periph, USART_INT_TC);
    /* reset transmit_complete_handle and tx_state */
    p_irda->irda_irq.transmit_complete_handle = NULL;
    p_irda->tx_state = IRDA_STATE_FREE;
    
    if(NULL != p_func){
        /* if there is a user transmit complete callback */
        p_func(p_irda);
    }
}

/*!
    \brief      handle the transmit interrupt
    \param[in]  irda: pointer to a IrDA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_transmit_interrupt(void *irda)
{
    uint32_t temp_val;
    hal_irda_dev_struct *p_irda = irda;
    
    temp_val = p_irda->txbuffer.pos;
    if(temp_val < p_irda->txbuffer.length){
        if((RESET != (USART_CTL0(p_irda->periph) & USART_CTL0_WL)) && \
           (RESET == (USART_CTL0(p_irda->periph) & USART_CTL0_PCEN))){
            /* 9-bit data, none parity */
            usart_data_transmit(p_irda->periph, (*(uint16_t*)p_irda->txbuffer.buffer & (uint16_t)0x1FFU));
            p_irda->txbuffer.buffer += 2U;
        }else{
            /* 9-bit data, with parity or 8-bit data */
            usart_data_transmit(p_irda->periph, (*p_irda->txbuffer.buffer & (uint8_t)0xFFU));
            p_irda->txbuffer.buffer++;
        }
        p_irda->txbuffer.pos++;
    }else{
        /* disable the TBE interrupt, enable the TC interrupt and reset the transmit_ready_handle */
        usart_interrupt_disable(p_irda->periph, USART_INT_TBE);
        usart_interrupt_enable(p_irda->periph, USART_INT_TC);
        p_irda->irda_irq.transmit_ready_handle = NULL;
    }
}

/*!
    \brief      handle the receive interrupt
    \param[in]  irda: pointer to a IrDA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_receive_interrupt(void *irda)
{
    uint16_t recv_data;
    hal_irda_dev_struct *p_irda = irda;
    uint32_t temp_val;
    
    recv_data = (usart_data_receive(p_irda->periph) & p_irda->data_bit_mask);
    if(0x1FFU == p_irda->data_bit_mask){
        /* store the received data */
        *(uint16_t *)p_irda->rxbuffer.buffer = recv_data;
        p_irda->rxbuffer.buffer += 2U;
    }else{
        /* store the received data */
        *p_irda->rxbuffer.buffer = (uint8_t)recv_data;
        p_irda->rxbuffer.buffer++;
    }
    p_irda->rxbuffer.pos++;
    
    temp_val = p_irda->rxbuffer.pos;
    if(temp_val == p_irda->rxbuffer.length){
        hal_irda_user_cb p_func = (hal_irda_user_cb)p_irda->rx_callback;
        /* disable PERR, ERR, RBNE interrupt */
        usart_interrupt_disable(p_irda->periph, USART_INT_PERR);
        usart_interrupt_disable(p_irda->periph, USART_INT_ERR);
        usart_interrupt_disable(p_irda->periph, USART_INT_RBNE);
        /* reset receive_complete_handle and rx_state */
        p_irda->irda_irq.receive_complete_handle = NULL;
        p_irda->rx_state = IRDA_STATE_FREE;
        
        if(NULL != p_func){
            /* if there is a user receive complete callback */
            p_func(p_irda);
        }
    }
}

/*!
    \brief      handle the IrDA DMA transmit process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_irda_dev_struct *p_irda;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_irda = (hal_irda_dev_struct*)p_dma->p_periph;
    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_irda->txbuffer.pos = p_irda->txbuffer.length;
        usart_dma_transmit_config(p_irda->periph, USART_DENT_DISABLE);
        /* enable TC interrupt */
        usart_interrupt_enable(p_irda->periph, USART_INT_TC);
    }
}

/*!
    \brief      handle the IrDA DMA receive process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_irda_dev_struct *p_irda;
    hal_irda_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_irda = (hal_irda_dev_struct*)p_dma->p_periph;
    p_func = (hal_irda_user_cb)p_irda->rx_callback;
    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_irda->rxbuffer.pos = p_irda->rxbuffer.length;
        /* disable DMA receive, PERR and ERR interrupt */
        usart_dma_receive_config(p_irda->periph, USART_DENR_DISABLE);
        usart_interrupt_disable(p_irda->periph, USART_INT_PERR);
        usart_interrupt_disable(p_irda->periph, USART_INT_ERR);
        /* reset rx_state */
        p_irda->rx_state = IRDA_STATE_FREE;
    }
    
    if(NULL != p_func){
        /* if there is a user receive complete callback */
        p_func(p_irda);
    }
}

/*!
    \brief      handle the IrDA DMA error process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _irda_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_irda_dev_struct *p_irda;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_irda = (hal_irda_dev_struct*)p_dma->p_periph;

    if(IRDA_STATE_BUSY == p_irda->tx_state){
        /* transmit state is busy */
        p_irda->error_state |= HAL_USART_ERROR_DMATX;
        p_irda->last_error = HAL_USART_ERROR_DMATX;
        p_irda->txbuffer.pos = p_irda->txbuffer.length;
        /* disable DMA transmit and reset tx_state */
        usart_dma_transmit_config(p_irda->periph, USART_DENT_DISABLE);
        p_irda->tx_state = IRDA_STATE_FREE;
    }else if(IRDA_STATE_BUSY == p_irda->rx_state){
        /* receive state is busy */
        p_irda->error_state |= HAL_USART_ERROR_DMARX;
        p_irda->last_error = HAL_USART_ERROR_DMARX;
        p_irda->rxbuffer.pos = p_irda->rxbuffer.length;
        /* disable DMA receive, PERR, ERR interrupt */
        usart_dma_receive_config(p_irda->periph, USART_DENR_DISABLE);
        usart_interrupt_disable(p_irda->periph, USART_INT_PERR);
        usart_interrupt_disable(p_irda->periph, USART_INT_ERR);
        /* reset rx_state */
        p_irda->rx_state = IRDA_STATE_FREE;
    }else{
        HAL_DEBUGE("irda processor fatal error: dma error exception due to run state");
    }

    if(p_irda->irda_irq.error_handle != NULL){
        /* if there is a user error callback */
        p_irda->irda_irq.error_handle(p_irda);
        p_irda->error_state = HAL_USART_ERROR_NONE;
    }
}
