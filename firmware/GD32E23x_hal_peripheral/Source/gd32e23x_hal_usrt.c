/*!
    \file    gd32e23x_hal_usrt.c
    \brief   USRT driver
    
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

#define USRT_DUMMY_DATA            ((uint16_t) 0xFFFFU) 

static uint16_t _usrt_data_bit_mask_get(hal_usrt_dev_struct *usrt);
static FlagStatus _usrt_error_flag_get(hal_usrt_dev_struct *usrt);
static void _usrt_transmit_complete_interrupt(void *usrt);
static void _usrt_transmit_interrupt(void *usrt);
static void _usrt_receive_interrupt(void *usrt);
static void _usrt_tx_rx_interrupt(void *usrt);
static void _usrt_transmit_dma(void *dma);
static void _usrt_receive_dma(void *dma);
static void _usrt_dma_error(void *dma);

/*!
    \brief      initialize the USRT struct with the default values, note that this function must be
                called after the structure is created
    \param[in]  hal_struct_type: type of USRT struct for initialization
      \arg        HAL_USRT_INIT_STRUCT: initialization structure
      \arg        HAL_USRT_DEV_STRUCT: device information structure
      \arg        HAL_USRT_USER_CALLBCAK_STRUCT: user callback struct
    \param[in]  p_struct: structure pointer
    \param[out] none
    \retval     none
*/
void hal_usrt_struct_init(hal_usrt_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(hal_struct_type){
    case HAL_USRT_INIT_STRUCT:
        /* initialize usrt initialization structure with the default values */
        ((hal_usrt_init_struct*)p_struct)->baudrate = 115200U;
        ((hal_usrt_init_struct*)p_struct)->direction = USRT_DIRECTION_RX_TX;
        ((hal_usrt_init_struct*)p_struct)->parity = USRT_PARITY_NONE;
        ((hal_usrt_init_struct*)p_struct)->stop_bit = USRT_STOP_BIT_1;
        ((hal_usrt_init_struct*)p_struct)->word_length = USRT_WORD_LENGTH_8BIT;
        ((hal_usrt_init_struct*)p_struct)->clock_polarity = USRT_CLOCK_POLARITY_LOW;
        ((hal_usrt_init_struct*)p_struct)->clock_phase = USRT_CLOCK_PHASE_1CK;
        ((hal_usrt_init_struct*)p_struct)->clock_length_lastbit = USRT_LAST_BIT_NOT_OUTPUT;
        break;
        
    case HAL_USRT_DEV_STRUCT:
        /* initialize usrt device information structure with the default values */
        ((hal_usrt_dev_struct*)p_struct)->periph = 0U;
        ((hal_usrt_dev_struct*)p_struct)->usrt_irq.error_handle = NULL;
        ((hal_usrt_dev_struct*)p_struct)->usrt_irq.receive_complete_handle = NULL;
        ((hal_usrt_dev_struct*)p_struct)->usrt_irq.transmit_complete_handle = NULL;
        ((hal_usrt_dev_struct*)p_struct)->usrt_irq.transmit_ready_handle = NULL;
        ((hal_usrt_dev_struct*)p_struct)->p_dma_rx = NULL;
        ((hal_usrt_dev_struct*)p_struct)->p_dma_tx = NULL;
        ((hal_usrt_dev_struct*)p_struct)->txbuffer.buffer = NULL;
        ((hal_usrt_dev_struct*)p_struct)->txbuffer.length = 0U;
        ((hal_usrt_dev_struct*)p_struct)->txbuffer.pos = 0U;
        ((hal_usrt_dev_struct*)p_struct)->rxbuffer.buffer = NULL;
        ((hal_usrt_dev_struct*)p_struct)->rxbuffer.length = 0U;
        ((hal_usrt_dev_struct*)p_struct)->rxbuffer.pos = 0U;
        ((hal_usrt_dev_struct*)p_struct)->error_state = HAL_USART_ERROR_NONE;
        ((hal_usrt_dev_struct*)p_struct)->tx_state = USRT_STATE_FREE;
        ((hal_usrt_dev_struct*)p_struct)->rx_state = USRT_STATE_FREE;
        ((hal_usrt_dev_struct*)p_struct)->last_error = HAL_USART_ERROR_NONE;
        ((hal_usrt_dev_struct*)p_struct)->priv = NULL;      
        break;
    
    case HAL_USRT_USER_CALLBCAK_STRUCT:
        /* initialize user callback structure with the default values */
        ((hal_usrt_user_callback_struct*)p_struct)->complete_func = NULL;
        ((hal_usrt_user_callback_struct*)p_struct)->error_func = NULL;
        break;
    
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize the USRT
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_usrt_deinit(hal_usrt_dev_struct *usrt)
{
    uint32_t periph;
    
    periph = usrt->periph;
    if((USART0 == periph) || (USART1 == periph)){
        /* deinitialize the periph and the device information sturcture */
        usart_deinit(periph);
        hal_usrt_struct_init(HAL_USRT_DEV_STRUCT, usrt);
        usrt->periph = periph;
    }else{
        HAL_DEBUGE("parameter [usrt->periph] value is invalid");
    }
}

/*!
    \brief      initialize the USRT with specified values
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is initialized
    \param[in]  p_init: the initialization data needed to initialize USRT
                  baudrate: communication baudrate
                  parity: USRT_PARITY_NONE, USRT_PARITY_EVEN, USRT_PARITY_ODD
                  word_length: USRT_WORD_LENGTH_8BIT, USRT_WORD_LENGTH_9BIT
                  stop_bit: USRT_STOP_BIT_1, USRT_STOP_BIT_2, USRT_STOP_BIT_1_5
                  direction: USRT_DIRECTION_RX_TX, USRT_DIRECTION_RX_ONLY, USRT_DIRECTION_TX_ONLY
                  clock_polarity: USRT_CLOCK_POLARITY_LOW, USRT_CLOCK_POLARITY_HIGH
                  clock_phase: USRT_CLOCK_PHASE_1CK, USRT_CLOCK_PHASE_2CK
                  clock_length_lastbit: USRT_LAST_BIT_NOT_OUTPUT, USRT_LAST_BIT_OUTPUT
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_init(hal_usrt_dev_struct *usrt, uint32_t periph, \
                      hal_usrt_init_struct *p_init)
{
    uint32_t reg_temp;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check USRT pointer and p_init address */
    if((NULL == usrt) || (NULL == p_init)){
        HAL_DEBUGE("pointer [usrt] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check periph parameter */
    if((USART0 != periph) && (USART1 != periph)){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }
    
    /* check periph value from USRT device struct */
    if(0U != usrt->periph){
        HAL_DEBUGI("periph value from usrt device struct has been rewrite");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    usrt->periph = periph;
    
    usart_disable(periph);
    
    /* CTL0 register configure */
    reg_temp = USART_CTL0(periph);
    reg_temp &= ~(USART_CTL0_PM | USART_CTL0_PCEN | USART_CTL0_WL | \
                  USART_CTL0_REN | USART_CTL0_TEN);
    reg_temp |= (p_init->direction | p_init->parity | p_init->word_length);
    USART_CTL0(periph) = reg_temp;
    
    /* CTL1 register configure */
    reg_temp = USART_CTL1(periph);
    reg_temp &= ~(USART_CTL1_STB | USART_CTL1_CLEN | USART_CTL1_CPH | USART_CTL1_CPL);
    reg_temp |= (p_init->stop_bit | p_init->clock_length_lastbit | p_init->clock_phase | \
                p_init->clock_polarity | USART_CTL1_CKEN);
    USART_CTL1(periph) = reg_temp;
    
    /* boudrate configure */
    usart_baudrate_set(periph, p_init->baudrate);
    usrt->data_bit_mask = _usrt_data_bit_mask_get(usrt);
    
    /* reset the Rx and Tx state */
    usrt->tx_state = USRT_STATE_FREE;
    usrt->rx_state = USRT_STATE_FREE;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start USRT module
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_usrt_start(hal_usrt_dev_struct *usrt)
{
    usart_enable(usrt->periph);
}

/*!
    \brief      stop USRT module
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_usrt_stop(hal_usrt_dev_struct *usrt)
{
    usart_disable(usrt->periph);
}

/*!
    \brief      handle the USRT interrupts
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_usrt_irq(hal_usrt_dev_struct *usrt)
{
    if(RESET == _usrt_error_flag_get(usrt)){
        /* check whether USART is in receiver mode or not */
        if(RESET != usart_interrupt_flag_get(usrt->periph, USART_INT_FLAG_RBNE)){
            if(NULL != usrt->usrt_irq.receive_complete_handle){
                usrt->usrt_irq.receive_complete_handle(usrt);
            }
            return;
        }
    }else{
        /* check whether the PERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(usrt->periph, USART_INT_FLAG_PERR)){
            usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_PERR);
            usrt->error_state |= HAL_USART_ERROR_PERR;
            usrt->last_error = HAL_USART_ERROR_PERR;
        }
        
        /* check whether the NERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(usrt->periph, USART_INT_FLAG_ERR_NERR)){
            usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_NERR);
            usrt->error_state |= HAL_USART_ERROR_NERR;
            usrt->last_error = HAL_USART_ERROR_NERR;
        }
        
        /* check whether the FERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(usrt->periph, USART_INT_FLAG_ERR_FERR)){
            usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_FERR);
            usrt->error_state |= HAL_USART_ERROR_FERR;
            usrt->last_error = HAL_USART_ERROR_FERR;
        }
        
        /* check whether the ERR ORERR is set or not */
        if(RESET != usart_interrupt_flag_get(usrt->periph, USART_INT_FLAG_ERR_ORERR)){ 
            usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_ORERR);
            usrt->error_state |= HAL_USART_ERROR_ORERR;
            usrt->last_error = HAL_USART_ERROR_ORERR;
        }
        
        /* check whether RBNE ORERR is set or not */
        if(RESET != usart_interrupt_flag_get(usrt->periph, USART_INT_FLAG_RBNE_ORERR)){ 
            usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_RBNE_ORERR);
            usrt->error_state |= HAL_USART_ERROR_ORERR;
            usrt->last_error = HAL_USART_ERROR_ORERR;
        }
        
        /* check whether error state is none or not */
        if(HAL_USART_ERROR_NONE != usrt->error_state){
            if(NULL != usrt->usrt_irq.error_handle){
                usrt->usrt_irq.error_handle(usrt);
                usrt->error_state = HAL_USART_ERROR_NONE;
            }
            return;
        }
    }
    
    /* transmitter buffer empty interrupt handle */
    if(RESET != usart_interrupt_flag_get(usrt->periph, USART_INT_FLAG_TBE)){
        if(NULL != usrt->usrt_irq.transmit_ready_handle){
            usrt->usrt_irq.transmit_ready_handle(usrt);
        }
        
        return;
    }
    /* transmission complete interrupt handle */
    if(RESET != usart_interrupt_flag_get(usrt->periph, USART_INT_FLAG_TC)){
        usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_TC);
        
        if(NULL != usrt->usrt_irq.transmit_complete_handle){
            usrt->usrt_irq.transmit_complete_handle(usrt);
        }
        
        return;
    }
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  usrt: USRT device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to USRT interrupt callback functions structure
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
void hal_usrt_irq_handle_set(hal_usrt_dev_struct *usrt, hal_usrt_irq_struct *p_irq)
{
    /* set user-defined error interrupt callback */
    if(NULL != p_irq->error_handle){
        usrt->usrt_irq.error_handle = p_irq->error_handle;
        usart_interrupt_enable(usrt->periph, USART_INT_ERR);
        usart_interrupt_enable(usrt->periph, USART_INT_PERR);
    }else{
        usrt->usrt_irq.error_handle = NULL;
        usart_interrupt_disable(usrt->periph, USART_INT_ERR);
        usart_interrupt_disable(usrt->periph, USART_INT_PERR);
    }
    
    /* set user-defined receive complete interrupt callback */
    if(NULL != p_irq->receive_complete_handle){
        usrt->usrt_irq.receive_complete_handle = p_irq->receive_complete_handle;
        usart_interrupt_enable(usrt->periph, USART_INT_RBNE);
    }else{
        usrt->usrt_irq.receive_complete_handle = NULL;
        usart_interrupt_disable(usrt->periph, USART_INT_RBNE);
    }
    
    /* set user-defined transmit complete interrupt callback */
    if(NULL != p_irq->transmit_complete_handle){
        usrt->usrt_irq.transmit_complete_handle = p_irq->transmit_complete_handle;
        usart_interrupt_enable(usrt->periph, USART_INT_TC);
    }else{
        usrt->usrt_irq.transmit_complete_handle = NULL;
        usart_interrupt_disable(usrt->periph, USART_INT_TC);
    }
    
    /* set user-defined transmit ready interrupt callback */
    if(NULL != p_irq->transmit_ready_handle){
        usrt->usrt_irq.transmit_ready_handle = p_irq->transmit_ready_handle;
        usart_interrupt_enable(usrt->periph, USART_INT_TBE);
    }else{
        usrt->usrt_irq.transmit_ready_handle = NULL;
        usart_interrupt_disable(usrt->periph, USART_INT_TBE);
    }    
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_usrt_irq_handle_all_reset(hal_usrt_dev_struct *usrt)
{
    usrt->usrt_irq.error_handle = NULL;
    usrt->usrt_irq.receive_complete_handle = NULL;
    usrt->usrt_irq.transmit_complete_handle = NULL;
    usrt->usrt_irq.transmit_ready_handle = NULL;
}

/*!
    \brief      transmit amounts of data, poll transmit process and completed status
                the function is blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_transmit_poll(hal_usrt_dev_struct *usrt, void *p_buffer, \
                                uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    uint32_t temp_val;
    uint32_t tick_start;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state){
        HAL_DEBUGE("usrt tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)){
        if(RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)){
            data_length = 2U;
        }
    }
    
    /* initialize transmit parameters */
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->txbuffer.buffer = (uint8_t *)p_buffer; 
    usrt->txbuffer.length = length;
    usrt->txbuffer.pos = 0U;
    usrt->tx_state = USRT_STATE_BUSY;
    /* configure timeout */
    tick_start = hal_basetick_count_get();
    
    temp_val = usrt->txbuffer.pos;
    while(temp_val < usrt->txbuffer.length){
        /* wait for transmit buffer empty */
        while(RESET == usart_flag_get(usrt->periph, USART_FLAG_TBE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("usrt transmit timeout");
                    /* reset the state */
                    usrt->rx_state = USRT_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        
        /* send the data to be transmitted */
        if(1U == data_length){
            usart_data_transmit(usrt->periph, (*usrt->txbuffer.buffer & (uint8_t)0xFFU));
            usrt->txbuffer.buffer++;
        }else{
            usart_data_transmit(usrt->periph, (*(uint16_t*)usrt->txbuffer.buffer & (uint16_t)0x1FFU));
            usrt->txbuffer.buffer += data_length;
        }
        
        /* change the transmit pointer */
        usrt->txbuffer.pos++;
        temp_val = usrt->txbuffer.pos;
    }
    
    /* wait for transmit complete */
    while(RESET == usart_flag_get(usrt->periph, USART_FLAG_TC)){
        if(HAL_TIMEOUT_FOREVER != timeout_ms){
            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                HAL_DEBUGW("usrt transmit timeout");
                return HAL_ERR_TIMEOUT;
            }
        }
    }
    
    /* change the Tx state to free */
    usrt->tx_state = USRT_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data, poll receive process and completed status
                the function is blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_receive_poll(hal_usrt_dev_struct *usrt, void *p_buffer, \
                               uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    uint32_t temp_val;
    uint32_t tick_start;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state){
        HAL_DEBUGE("usrt rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* calculate the data length */
    data_length = 1;
    if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)){
        if(RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)){
            data_length = 2U;
        }
    }
    
    /* initialize receive parameters */
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->rxbuffer.buffer = (uint8_t *)p_buffer; 
    usrt->rxbuffer.length = length;
    usrt->rxbuffer.pos = 0U;
    usrt->data_bit_mask = _usrt_data_bit_mask_get(usrt);
    usrt->rx_state = USRT_STATE_BUSY;
    /* configure timeout */
    tick_start = hal_basetick_count_get();
    
    temp_val = usrt->rxbuffer.pos;
    while(temp_val < usrt->rxbuffer.length){
       /* send dummy byte to generate clock for the slave to send data */
        while(RESET == usart_flag_get(usrt->periph, USART_FLAG_TC)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("usrt transmit timeout");
                    /* reset the state */
                    usrt->rx_state = USRT_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
                usart_data_transmit(usrt->periph, USRT_DUMMY_DATA);
            }
        }
        
        /* reconfigure the timeout */
        tick_start = hal_basetick_count_get();
        /* wait for read data buffer not empty */
        while(RESET == usart_flag_get(usrt->periph, USART_FLAG_RBNE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("usrt receive timeout");
                    /* reset the state */
                    usrt->rx_state = USRT_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }            
            }
        }
        
        /* read data from data register */
        if(1U == data_length){
            *usrt->rxbuffer.buffer = (uint8_t)(usart_data_receive(usrt->periph) & usrt->data_bit_mask);
            usrt->rxbuffer.buffer++;
        }else{
            *(uint16_t*)usrt->rxbuffer.buffer = (usart_data_receive(usrt->periph) & usrt->data_bit_mask);
             usrt->rxbuffer.buffer += data_length;
        }
        
        /* change the receive pointer */
        usrt->rxbuffer.pos++;
        temp_val = usrt->rxbuffer.pos;
    }
    
    /* change the Rx state to free */
    usrt->rx_state = USRT_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit & receive amounts of data, poll transfer process and completed status
                the function is blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_tx_buffer: pointer to Tx data buffer
    \param[in]  p_rx_buffer: pointer to Rx data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_transmit_receive_poll(hal_usrt_dev_struct *usrt, void *p_tx_buffer, void *p_rx_buffer, \
                                uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    uint32_t temp_val;
    uint32_t tick_start;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_tx_buffer) || (NULL == p_rx_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_tx_buffer] or [p_rx_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* 9-bit transfer with no parity, if tx or rx buffer address is not aligned on uint16_t, return error */
    if((RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)) && \
        (RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN))){
        if(RESET != (((uint32_t)p_tx_buffer || (uint32_t)p_rx_buffer) & 1U)){
            return HAL_ERR_ADDRESS;
        }
    }
    
    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state){
        HAL_DEBUGE("usrt rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* calculate the data length */
    data_length = 1;
    if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)){
        if(RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)){
            data_length = 2U;
        }
    }
    
    /* initialize receive parameters */
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->rxbuffer.buffer = (uint8_t *)p_rx_buffer;
    usrt->rxbuffer.length = length;
    usrt->rxbuffer.pos = 0U;
    usrt->data_bit_mask = _usrt_data_bit_mask_get(usrt);
    usrt->rx_state = USRT_STATE_BUSY;
    
    /* initialize transmit parameters */
    usrt->txbuffer.buffer = (uint8_t *)p_tx_buffer;
    usrt->txbuffer.length = length;
    usrt->tx_state = USRT_STATE_BUSY;
    
    /* configure timeout */
    tick_start = hal_basetick_count_get();
    
    temp_val = usrt->rxbuffer.pos;
    while(temp_val < usrt->rxbuffer.length){
        /* wait the TBE flag is set */
        while(RESET == usart_flag_get(usrt->periph, USART_FLAG_TBE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("usrt TBE flag set timeout");
                    /* reset the state */
                    usrt->tx_state = USRT_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        
        /* send the data to be transmitted */
        if(1U == data_length){
            usart_data_transmit(usrt->periph, (*usrt->txbuffer.buffer & (uint8_t)usrt->data_bit_mask));
            usrt->txbuffer.buffer++;
        }else{
            usart_data_transmit(usrt->periph, (*(uint16_t*)usrt->txbuffer.buffer & usrt->data_bit_mask));
            usrt->txbuffer.buffer += data_length;
        }
        
        /* wait for transmit complete */
        while(RESET == usart_flag_get(usrt->periph, USART_FLAG_TC)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("usrt transmit timeout");
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        
        /* wait the RBNE flag is set */
        while(RESET == usart_flag_get(usrt->periph, USART_FLAG_RBNE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("usrt RBNE flag set timeout");
                    /* reset the state */
                    usrt->rx_state = USRT_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        
        /* read data from data register */
        if(1U == data_length){
            *usrt->rxbuffer.buffer = (uint8_t)(usart_data_receive(usrt->periph) & usrt->data_bit_mask);
            usrt->rxbuffer.buffer++;
        }else{
            *(uint16_t*)usrt->rxbuffer.buffer = (usart_data_receive(usrt->periph) & usrt->data_bit_mask);
             usrt->rxbuffer.buffer += data_length;
        }
        
        /* change the receive pointer */
        usrt->rxbuffer.pos++;
        temp_val = usrt->rxbuffer.pos;
    }
    
    /* change the Tx and Rx state to free */
    usrt->tx_state = USRT_STATE_FREE;
    usrt->rx_state = USRT_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method, after the transfer is completed, 
                the user function is called
                the function is non-blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h 
*/
int32_t hal_usrt_transmit_interrupt(hal_usrt_dev_struct *usrt, void *p_buffer, \
                                     uint32_t length, hal_usrt_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state){
        HAL_DEBUGE("usrt tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize transmit parameters */
    usrt->tx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->txbuffer.buffer = (uint8_t *)p_buffer; 
    usrt->txbuffer.length = length;
    usrt->txbuffer.pos = 0U;
    usrt->tx_callback = (void *)p_user_func;
    /* configure the transmit ready and complete callback as the function implemented */
    usrt->usrt_irq.transmit_ready_handle = _usrt_transmit_interrupt;
    usrt->usrt_irq.transmit_complete_handle = _usrt_transmit_complete_interrupt;
    
    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_TC);

    /* enable the TBE interrupt */
    usart_interrupt_enable(usrt->periph, USART_INT_TBE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method, after the transfer is completed, 
                the user function is called 
                the function is non-blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_receive_interrupt(hal_usrt_dev_struct *usrt, void *p_buffer, \
                                    uint32_t length, hal_usrt_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state){
        HAL_DEBUGE("usrt rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize receive parameters */
    usrt->rx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->rxbuffer.buffer = (uint8_t *)p_buffer; 
    usrt->rxbuffer.length = length;
    usrt->rxbuffer.pos = 0U;
    usrt->rx_callback = (void *)p_user_func;
    usrt->usrt_irq.receive_complete_handle = _usrt_receive_interrupt;
    usrt->data_bit_mask = _usrt_data_bit_mask_get(usrt);
    
    /* enable PERR, ERR, RBNE interrupt */
    usart_interrupt_enable(usrt->periph, USART_INT_PERR);
    usart_interrupt_enable(usrt->periph, USART_INT_ERR);
    usart_interrupt_enable(usrt->periph, USART_INT_RBNE);
    
    /* send dummy byte to generate clock for the slave to send data */
    usart_data_transmit(usrt->periph, USRT_DUMMY_DATA & _usrt_data_bit_mask_get(usrt));
    
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit & receive amounts of data by interrupt method, after the transfer is completed, 
                the user function is called 
                the function is non-blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_tx_buffer: pointer to Tx data buffer
    \param[in]  p_rx_buffer: pointer to Rx data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_transmit_receive_interrupt(hal_usrt_dev_struct *usrt, void *p_tx_buffer, void *p_rx_buffer, \
                                    uint32_t length, hal_usrt_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_tx_buffer) || (NULL == p_rx_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_tx_buffer] or [p_rx_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* 9-bit transfer with no parity, if tx or rx buffer address is not aligned on uint16_t, return error */
    if((RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)) && \
        (RESET == (USART_CTL0(usrt->periph) & USART_CTL0_PCEN))){
        if(RESET != (((uint32_t)p_tx_buffer || (uint32_t)p_rx_buffer) & 1U)){
            return HAL_ERR_ADDRESS;
        }
    }
    
    /* check the rx_state wheher is in busy Tx Rx or not */
    if(USRT_STATE_BUSY_TX_RX == usrt->rx_state){
        HAL_DEBUGE("usrt tx or rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize transmit parameters */
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->tx_state = USRT_STATE_BUSY_TX_RX;
    usrt->txbuffer.buffer = (uint8_t *)p_tx_buffer; 
    usrt->txbuffer.length = length;
    usrt->txbuffer.pos = 0U;
    usrt->usrt_irq.transmit_ready_handle = _usrt_tx_rx_interrupt;
    usrt->usrt_irq.transmit_complete_handle = _usrt_transmit_complete_interrupt;
    
    /* initialize receive parameters */
    usrt->rx_state = USRT_STATE_BUSY_TX_RX;
    usrt->rxbuffer.buffer = (uint8_t *)p_rx_buffer; 
    usrt->rxbuffer.length = length;
    usrt->rxbuffer.pos = 0U;
    usrt->rx_callback = (void *)p_user_func;
    usrt->usrt_irq.receive_complete_handle = _usrt_tx_rx_interrupt;
    usrt->data_bit_mask = _usrt_data_bit_mask_get(usrt);
    
    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_TC);

    /* enable PERR, ERR, RBNE, TBE interrupt */
    usart_interrupt_enable(usrt->periph, USART_INT_PERR);
    usart_interrupt_enable(usrt->periph, USART_INT_ERR);
    usart_interrupt_enable(usrt->periph, USART_INT_RBNE);
    usart_interrupt_enable(usrt->periph, USART_INT_TBE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by DMA method, after the transfer is completed or error occurs, 
                the user function is called 
                the function is non-blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_transmit_dma(hal_usrt_dev_struct *usrt, void *p_buffer, \
                              uint32_t length, hal_usrt_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameter */
    if(NULL == usrt->p_dma_tx){
        HAL_DEBUGE("parameter [usrt->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state){
        HAL_DEBUGE("usrt tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    
    /* initialize transmit parameters */
    usrt->tx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->txbuffer.buffer = (uint8_t *)p_buffer; 
    usrt->txbuffer.length = length;
    usrt->txbuffer.pos = 0U;
    if(NULL != p_func){
        usrt->tx_callback = (void *)p_func->complete_func;
        usrt->usrt_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    }else{
        usrt->tx_callback = NULL;
        usrt->usrt_irq.error_handle = NULL;
    }
    usrt->usrt_irq.transmit_complete_handle = _usrt_transmit_complete_interrupt;
    
    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _usrt_transmit_dma;
    dma_irq.error_handle = _usrt_dma_error;
    if(NULL != usrt->p_dma_tx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = usrt->p_dma_tx->dma_irq.half_finish_handle;
    }
    
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(usrt->p_dma_tx, (uint32_t)usrt->txbuffer.buffer, \
                           usrt->periph + 0x28U, length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(usrt->p_dma_tx->channel, DMA_FLAG_G);
    
    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_TC);

    /* DMA enable for transmission */
    usart_dma_transmit_config(usrt->periph, USART_DENT_ENABLE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by DMA method, after the transfer is completed or error occurs, 
                the user function is called     
                the function is non-blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_receive_dma(hal_usrt_dev_struct *usrt, void *p_buffer, \
                              uint32_t length, hal_usrt_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameter */
    if(NULL == usrt->p_dma_rx){
        HAL_DEBUGE("parameter [usrt->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state){
        HAL_DEBUGE("usrt rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    
    /* initialize receive parameters */
    usrt->rx_state = USRT_STATE_BUSY;
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->rxbuffer.buffer = (uint8_t *)p_buffer; 
    usrt->rxbuffer.length = length;
    usrt->rxbuffer.pos = 0U;
    if(NULL != p_func){
        usrt->rx_callback = (void *)p_func->complete_func;
        usrt->usrt_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    }else{
        usrt->rx_callback = NULL;
        usrt->usrt_irq.error_handle = NULL;
    }
    usrt->data_bit_mask = _usrt_data_bit_mask_get(usrt);
    
    /* enable the usart parity error and error interrupt: (frame error, noise error, overrun error) */
    usart_interrupt_enable(usrt->periph, USART_INT_PERR);
    usart_interrupt_enable(usrt->periph, USART_INT_ERR);
    
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _usrt_receive_dma;
    dma_irq.error_handle = _usrt_dma_error;
    if(NULL != usrt->p_dma_rx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = usrt->p_dma_rx->dma_irq.half_finish_handle;
    }
    
    hal_dma_start_interrupt(usrt->p_dma_rx, usrt->periph + 0x24U, \
                           (uint32_t)usrt->rxbuffer.buffer, length, &dma_irq);
    
    /* USRT transmit data by DMA to generate clock */
    dma_irq.full_finish_handle = NULL;
    dma_irq.half_finish_handle = NULL;
    dma_irq.error_handle = NULL;
    
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(usrt->p_dma_tx, (uint32_t)usrt->rxbuffer.buffer, \
                           usrt->periph + 0x28U, length, &dma_irq);
    
    /* clear DMA global interrupt flag */
    dma_flag_clear(usrt->p_dma_tx->channel, DMA_FLAG_G);
    /* DMA enable for reception and transmission */
    usart_dma_receive_config(usrt->periph, USART_DENR_ENABLE);
    usart_dma_transmit_config(usrt->periph, USART_DENT_ENABLE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit & receive amounts of data by DMA method, after the transfer is completed or error occurs, 
                the user function is called 
                the function is non-blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_tx_buffer: pointer to Tx data buffer
    \param[in]  p_rx_buffer: pointer to Rx data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_transmit_receive_dma(hal_usrt_dev_struct *usrt, void *p_tx_buffer, void *p_rx_buffer, \
                                    uint32_t length, hal_usrt_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == usrt) || (NULL == p_tx_buffer) || (NULL == p_rx_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [usrt] or [p_tx_buffer] or [p_rx_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameter */
    if(NULL == usrt->p_dma_tx){
        HAL_DEBUGE("parameter [usrt->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy Tx Rx or not */
    if(USRT_STATE_BUSY_TX_RX == usrt->tx_state){
        HAL_DEBUGE("usrt tx or rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    
    /* initialize receive parameters */
    usrt->rx_state = USRT_STATE_BUSY_TX_RX;
    usrt->error_state = HAL_USART_ERROR_NONE;
    usrt->rxbuffer.buffer = (uint8_t *)p_rx_buffer; 
    usrt->rxbuffer.length = length;
    usrt->rxbuffer.pos = 0U;
    if(NULL != p_func){
        usrt->rx_callback = (void *)p_func->complete_func;
        usrt->usrt_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    }else{
        usrt->rx_callback = NULL;
        usrt->usrt_irq.error_handle = NULL;
    }
    usrt->data_bit_mask = _usrt_data_bit_mask_get(usrt);
    
    /* enable the usart parity error and error interrupt: (frame error, noise error, overrun error) */
    usart_interrupt_enable(usrt->periph, USART_INT_PERR);
    usart_interrupt_enable(usrt->periph, USART_INT_ERR);
    
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _usrt_receive_dma;
    dma_irq.error_handle = _usrt_dma_error;
    if(NULL != usrt->p_dma_rx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = usrt->p_dma_rx->dma_irq.half_finish_handle;
    }
    
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(usrt->p_dma_rx, usrt->periph + 0x24U, \
                           (uint32_t)usrt->rxbuffer.buffer, length, &dma_irq);
    
    /* initialize transmit parameters */
    usrt->tx_state = USRT_STATE_BUSY_TX_RX;
    usrt->txbuffer.buffer = (uint8_t *)p_tx_buffer; 
    usrt->txbuffer.length = length;
    usrt->txbuffer.pos = 0U;
    if(NULL != p_func){
        usrt->tx_callback = (void *)p_func->complete_func;
    }else{
        usrt->tx_callback = NULL;
    }
    usrt->usrt_irq.transmit_complete_handle = _usrt_transmit_complete_interrupt;
    
    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _usrt_transmit_dma;
    dma_irq.error_handle = _usrt_dma_error;
    if(NULL != usrt->p_dma_tx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = usrt->p_dma_tx->dma_irq.half_finish_handle;
    }
    
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(usrt->p_dma_tx, (uint32_t)usrt->txbuffer.buffer, \
                           usrt->periph + 0x28U, length, &dma_irq);
    /* clear DMA global interrupt flag and USART TC flag */
    dma_flag_clear(usrt->p_dma_tx->channel, DMA_FLAG_G);
    usart_flag_clear(usrt->periph, USART_FLAG_TC);
    /* DMA enable for reception and transmission */
    usart_dma_receive_config(usrt->periph, USART_DENR_ENABLE);
    usart_dma_transmit_config(usrt->periph, USART_DENT_ENABLE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      pause USRT DMA transfer during transmission process
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_dma_pause(hal_usrt_dev_struct *usrt)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == usrt){
        HAL_DEBUGE("parameter [usrt] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state){
        /* disable DMA transimt */
        usart_dma_transmit_config(usrt->periph, USART_DENT_DISABLE);
    }
    
    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state){
        /* disable the PERR and ERR interrupt */
        usart_interrupt_disable(usrt->periph, USART_INT_PERR);
        usart_interrupt_disable(usrt->periph, USART_INT_ERR);
        
        /* disable DMA receive */
        usart_dma_receive_config(usrt->periph, USART_DENR_DISABLE);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      resume USRT DMA transfer during transmission process
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_dma_resume(hal_usrt_dev_struct *usrt)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == usrt){
        HAL_DEBUGE("parameter [usrt] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->tx_state){
        /* enable DMA transimt */
        usart_dma_transmit_config(usrt->periph, USART_DENT_ENABLE);
    }
    
    /* check the rx_state wheher is busy or not */
    if(USRT_STATE_BUSY == usrt->rx_state){
        /* enable the PERR and ERR interrupt */
        usart_interrupt_enable(usrt->periph, USART_INT_PERR);
        usart_interrupt_enable(usrt->periph, USART_INT_ERR);
        
        /* enable DMA receive */
        usart_dma_receive_config(usrt->periph, USART_DENR_ENABLE);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      stop USRT transmit and receive transfer 
                the function is blocking
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_transfer_stop(hal_usrt_dev_struct *usrt)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == usrt){
        HAL_DEBUGE("parameter [usrt] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable the TBE and TC interrupt */
    usart_interrupt_disable(usrt->periph, USART_INT_TBE);
    usart_interrupt_disable(usrt->periph, USART_INT_TC);
    /* disable the RBNE, PERR and ERR interrupt */
    usart_interrupt_disable(usrt->periph, USART_INT_RBNE);
    usart_interrupt_disable(usrt->periph, USART_INT_PERR);
    usart_interrupt_disable(usrt->periph, USART_INT_ERR);
    
    /* disable DMA transimt and stop DMA */
    usart_dma_transmit_config(usrt->periph, USART_DENT_DISABLE);
    hal_dma_stop(usrt->p_dma_tx);
    /* disable DMA receive and stop DMA */
    usart_dma_receive_config(usrt->periph, USART_DENR_DISABLE);
    hal_dma_stop(usrt->p_dma_rx);
    
    /* reset the position and state */
    usrt->txbuffer.pos = 0;
    usrt->tx_state = USRT_STATE_FREE;
    usrt->rxbuffer.pos = 0;
    usrt->rx_state = USRT_STATE_FREE;
    
    /* clear interrupt error flags */
    usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_PERR);
    usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_FERR);
    usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_NERR);
    usart_interrupt_flag_clear(usrt->periph, USART_INT_FLAG_ERR_ORERR);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      configure the USRT baudrate and stop bits, the other parameters are configured as default values
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is configured
    \param[in]  baud: baudrate
    \param[in]  stopbits: stop bits
      \arg        USRT_STOP_BIT_1: 1 bit stop bit 
      \arg        USRT_STOP_BIT_2: 2 bits stop bit 
      \arg        USRT_STOP_BIT_1_5: 1.5 bits stop bit 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_simple_config(hal_usrt_dev_struct *usrt, uint32_t periph, uint32_t baud, uint32_t stopbits)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == usrt){
        HAL_DEBUGE("parameter [usrt] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameters */
    if(((USART0 != periph) && (USART1 != periph)) || (0U == baud)){
        HAL_DEBUGE("parameter [periph] or [baud] value is invalid"); 
        return HAL_ERR_VAL;
    }
    
    /* check the parameters */
    if((USART_STB_1BIT != stopbits) && (USART_STB_1_5BIT != stopbits) && (USART_STB_2BIT != stopbits)){
        HAL_DEBUGE("parameter [stopbits] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the transfer state */
    if((USRT_STATE_BUSY == usrt->tx_state) || (USRT_STATE_BUSY == usrt->rx_state)){
        HAL_DEBUGE("usrt is in tx or rx state, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    usrt->periph = periph;
    /* deinitilize USART */
    usart_deinit(periph);
    /* configure the UART baudrate and stop bits */
    usart_baudrate_set(periph, baud);
    usart_stop_bit_set(periph, stopbits);
    /* the other parameters are configured as default values */
    usart_parity_config(periph, USART_PM_NONE);
    usart_word_length_set(periph, USART_WL_8BIT);
    usart_receive_config(periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(periph, USART_TRANSMIT_ENABLE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      configure the USRT clock polarity, phase and last bit
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is configured
    \param[in]  clock_polarity_phase: clock polarity and phase
      \arg        USRT_POLARITY_LOW_PHASE_1CK: polarity low and phase on 1st clock
      \arg        USRT_POLARITY_LOW_PHASE_2CK: polarity low and phase on 2nd clock 
      \arg        USRT_POLARITY_HIGH_PHASE_1CK: polarity high and phase on 1st clock 
      \arg        USRT_POLARITY_HIGH_PHASE_2CK: polarity high and phase on 2nd clock
    \param[in]  clock_lastbit: clock length lastbit output
      \arg        USRT_LAST_BIT_NOT_OUTPUT: the clock pulse of the last data bit (MSB) is not output to the CK pin
      \arg        USRT_LAST_BIT_OUTPUT: the clock pulse of the last data bit (MSB) is output to the CK pin
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_usrt_simple_clock_config(hal_usrt_dev_struct *usrt, uint32_t periph, \
                              uint32_t clock_polarity_phase, uint32_t clock_lastbit)
{
    uint8_t uen_flag = 0U;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == usrt){
        HAL_DEBUGE("parameter [usrt] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameters */
    if((USART0 != periph) && (USART1 != periph)){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }
    
    /* check the parameter */
    if((USRT_POLARITY_LOW_PHASE_1CK != clock_polarity_phase) && (USRT_POLARITY_LOW_PHASE_2CK != \
            clock_polarity_phase) && (USRT_POLARITY_HIGH_PHASE_1CK != clock_polarity_phase) && \
            (USRT_POLARITY_HIGH_PHASE_2CK != clock_polarity_phase)){
        HAL_DEBUGE("parameter [clock_polarity_phase] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check the parameter */
    if((USRT_LAST_BIT_NOT_OUTPUT != clock_lastbit) && (USRT_LAST_BIT_OUTPUT != clock_lastbit)){
        HAL_DEBUGE("parameter [clock_lastbit] value is invalid"); 
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx and tx state whether is in busy state */
    if((USRT_STATE_BUSY == usrt->tx_state) || (USRT_STATE_BUSY == usrt->rx_state)){
        HAL_DEBUGE("usrt is in tx or rx state, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    usrt->periph = periph;
    /* check whether the UEN is set or not */
    if(RESET != (USART_CTL0(periph) & USART_CTL0_UEN)){
        uen_flag = 1U;
        usart_disable(periph);
    }
    
    /* reset USART_CTL1 CLEN,CPH,CPL bits */
    USART_CTL1(periph) &= ~(USART_CTL1_CLEN | USART_CTL1_CPH | USART_CTL1_CPL);
    USART_CTL1(periph) |= (clock_polarity_phase | clock_lastbit);
    /* restore the state of UEN bit */
    if(0U != uen_flag){
        usart_enable(periph);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      get the mask of date bit
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the mask of date bit(0x7F, 0xFF, 0x1FF)
*/
static uint16_t _usrt_data_bit_mask_get(hal_usrt_dev_struct *usrt)
{
    uint16_t reval;
    
    if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_WL)){
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)){
            reval = 0xFFU;
        }else{
            reval = 0x1FFU;
        }
    }else{
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(usrt->periph) & USART_CTL0_PCEN)){
            reval = 0x7FU;
        }else{
            reval = 0xFFU;
        }
    }
    
    return reval;
}

/*!
    \brief      get USRT error flag
    \param[in]  usrt: USRT device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     SET or RESET
*/
static FlagStatus _usrt_error_flag_get(hal_usrt_dev_struct *usrt)
{
    if(0U == (USART_STAT(usrt->periph) & (uint32_t)(USART_STAT_PERR | USART_STAT_FERR | \
                                                    USART_STAT_ORERR | USART_STAT_NERR))){
        return RESET;
    }else{
        return SET;
    }
}

/*!
    \brief      handle the transmit complete interrupt
    \param[in]  usrt: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_transmit_complete_interrupt(void *usrt)
{
    hal_usrt_dev_struct *p_usrt = usrt;
    hal_usrt_user_cb p_func = (hal_usrt_user_cb)p_usrt->tx_callback;
    if(USRT_STATE_BUSY_TX_RX == p_usrt->tx_state){
        /* in Tx Rx state, the callback function is called when receive complete event occurs */
        p_func = NULL;
    }
    
    /* disable the transmit complete interrupt */
    usart_interrupt_disable(p_usrt->periph, USART_INT_TC);
    /* reset transmit_complete_handle and tx_state */
    p_usrt->usrt_irq.transmit_complete_handle = NULL;
    p_usrt->tx_state = USRT_STATE_FREE;
    
    if(NULL != p_func){
        /* if there is a user transmit complete callback */
        p_func(p_usrt);
    }
}

/*!
    \brief      handle the transmit interrupt
    \param[in]  usrt: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_transmit_interrupt(void *usrt)
{
    uint32_t temp_val;
    hal_usrt_dev_struct *p_usrt = usrt;
    
    temp_val = p_usrt->txbuffer.pos;
    if(temp_val < p_usrt->txbuffer.length){
        if((RESET != (USART_CTL0(p_usrt->periph) & USART_CTL0_WL)) && \
           (RESET == (USART_CTL0(p_usrt->periph) & USART_CTL0_PCEN))){
            /* 9-bit data, none parity */
            usart_data_transmit(p_usrt->periph, (*(uint16_t*)p_usrt->txbuffer.buffer & (uint16_t)0x1FFU));
            p_usrt->txbuffer.buffer += 2U;
        }else{
            /* 9-bit data, with parity or 8-bit data */
            usart_data_transmit(p_usrt->periph, (*p_usrt->txbuffer.buffer & (uint8_t)0xFFU));
            p_usrt->txbuffer.buffer++;
        }
        p_usrt->txbuffer.pos++;
    }else{
        /* disable the TBE interrupt, enable the TC interrupt and reset the transmit_ready_handle */
        usart_interrupt_disable(p_usrt->periph, USART_INT_TBE);
        usart_interrupt_enable(p_usrt->periph, USART_INT_TC);
        p_usrt->usrt_irq.transmit_ready_handle = NULL;
    }
}

/*!
    \brief      handle the receive interrupt
    \param[in]  usrt: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_receive_interrupt(void *usrt)
{
    uint16_t recv_data;
    hal_usrt_dev_struct *p_usrt = usrt;
    uint32_t temp_val;
    
    recv_data = (usart_data_receive(p_usrt->periph) & p_usrt->data_bit_mask);
    if(0x1FFU == p_usrt->data_bit_mask){
        /* store the received data */
        *(uint16_t *)p_usrt->rxbuffer.buffer = recv_data;
        p_usrt->rxbuffer.buffer += 2U;
    }else{
        /* store the received data */
        *p_usrt->rxbuffer.buffer = (uint8_t)recv_data;
        p_usrt->rxbuffer.buffer++;
    }
    p_usrt->rxbuffer.pos++;
    
    temp_val = p_usrt->rxbuffer.pos;
    if(temp_val == p_usrt->rxbuffer.length){
        hal_usrt_user_cb p_func = (hal_usrt_user_cb)p_usrt->rx_callback;
        /* disable PERR, ERR, RBNE interrupt */
        usart_interrupt_disable(p_usrt->periph, USART_INT_PERR);
        usart_interrupt_disable(p_usrt->periph, USART_INT_ERR);
        usart_interrupt_disable(p_usrt->periph, USART_INT_RBNE);
        /* reset receive_complete_handle and rx_state */
        p_usrt->usrt_irq.receive_complete_handle = NULL;
        p_usrt->rx_state = USRT_STATE_FREE;
        
        if(NULL != p_func){
            /* if there is a user receive complete callback */
            p_func(p_usrt);
        }
    }
}

/*!
    \brief      handle the transmit and receive interrupt
    \param[in]  usrt: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_tx_rx_interrupt(void *usrt)
{
    uint16_t recv_data;
    hal_usrt_dev_struct *p_usrt = usrt;
    uint32_t temp_val;
    
    if(USRT_STATE_BUSY_TX_RX == p_usrt->tx_state){
        /* check whether the TBE is set or not */
        if(RESET != usart_flag_get(p_usrt->periph, USART_FLAG_TBE)){
            temp_val = p_usrt->txbuffer.pos;
            if(temp_val < p_usrt->txbuffer.length){
                if((RESET != (USART_CTL0(p_usrt->periph) & USART_CTL0_WL)) && \
                   (RESET == (USART_CTL0(p_usrt->periph) & USART_CTL0_PCEN))){
                    /* 9-bit data, none parity */
                    usart_data_transmit(p_usrt->periph, (*(uint16_t*)p_usrt->txbuffer.buffer & (uint16_t)0x1FFU));
                    p_usrt->txbuffer.buffer += 2U;
                }else{
                    /* 9-bit data, with parity or 8-bit data */
                    usart_data_transmit(p_usrt->periph, (*p_usrt->txbuffer.buffer & (uint8_t)0xFFU));
                    p_usrt->txbuffer.buffer++;
                }
                p_usrt->txbuffer.pos++;
            }else{
                /* disable the TBE interrupt */
                usart_interrupt_disable(p_usrt->periph, USART_INT_TBE);
                usart_interrupt_enable(p_usrt->periph, USART_INT_TC);
                p_usrt->usrt_irq.transmit_ready_handle = NULL;
            }
        }
    }
    if(USRT_STATE_BUSY_TX_RX == p_usrt->rx_state){
        /* check whether the RBNE is set or not */
        if(RESET != usart_flag_get(p_usrt->periph, USART_FLAG_RBNE)){
            recv_data = (usart_data_receive(p_usrt->periph) & p_usrt->data_bit_mask);
            if(0x1FFU == p_usrt->data_bit_mask){
                /* 9-bit data, none parity */
                *(uint16_t *)p_usrt->rxbuffer.buffer = recv_data;
                p_usrt->rxbuffer.buffer += 2U;
            }else{
                /* 9-bit data, with parity or 8-bit data */
                *p_usrt->rxbuffer.buffer = (uint8_t)recv_data;
                p_usrt->rxbuffer.buffer++;
            }
            p_usrt->rxbuffer.pos++;
        }
        
        temp_val = p_usrt->rxbuffer.pos;
        if(temp_val == p_usrt->rxbuffer.length){
            hal_usrt_user_cb p_func = (hal_usrt_user_cb)p_usrt->rx_callback;
            /* disable PERR, ERR, RBNE interrupt */
            usart_interrupt_disable(p_usrt->periph, USART_INT_PERR);
            usart_interrupt_disable(p_usrt->periph, USART_INT_ERR);
            usart_interrupt_disable(p_usrt->periph, USART_INT_RBNE);
            /* reset receive_complete_handle and rx_state */
            p_usrt->usrt_irq.receive_complete_handle = NULL;
            p_usrt->rx_state = USRT_STATE_FREE;
            
            if(NULL != p_func){
                /* if there is a user Tx Rx complete callback */
                p_func(p_usrt);
            }
        }
    }
}

/*!
    \brief      handle the USRT DMA transmit process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_usrt_dev_struct *p_usrt;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_usrt = (hal_usrt_dev_struct*)p_dma->p_periph;
    
    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_usrt->txbuffer.pos = p_usrt->txbuffer.length;
        usart_dma_transmit_config(p_usrt->periph, USART_DENT_DISABLE);
        /* enable TC interrupt */
        usart_interrupt_enable(p_usrt->periph, USART_INT_TC);
    }
}

/*!
    \brief      handle the USRT DMA receive process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_usrt_dev_struct *p_usrt;
    hal_usrt_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_usrt = (hal_usrt_dev_struct*)p_dma->p_periph;
    p_func = (hal_usrt_user_cb)p_usrt->rx_callback;
    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_usrt->rxbuffer.pos = p_usrt->rxbuffer.length;
        /* disable DMA receive, PERR and ERR interrupt */
        usart_dma_receive_config(p_usrt->periph, USART_DENR_DISABLE);
        usart_interrupt_disable(p_usrt->periph, USART_INT_PERR);
        usart_interrupt_disable(p_usrt->periph, USART_INT_ERR);
        /* reset rx_state */
        p_usrt->rx_state = USRT_STATE_FREE;
    }
    
    if(NULL != p_func){
        /* if there is a user receive complete callback */
        p_func(p_usrt);
    }
}

/*!
    \brief      handle the UART DMA error process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _usrt_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_usrt_dev_struct *p_usrt;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_usrt = (hal_usrt_dev_struct*)p_dma->p_periph;

    if(USRT_STATE_BUSY == p_usrt->tx_state){
        /* transmit state is busy */
        p_usrt->error_state |= HAL_USART_ERROR_DMATX;
        p_usrt->last_error = HAL_USART_ERROR_DMATX;
        p_usrt->txbuffer.pos = p_usrt->txbuffer.length;
        /* disable DMA transmit and reset tx_state */
        usart_dma_transmit_config(p_usrt->periph, USART_DENT_DISABLE);
        p_usrt->tx_state = USRT_STATE_FREE;
    }else if(USRT_STATE_BUSY == p_usrt->rx_state){
        /* receive state is busy */
        p_usrt->error_state |= HAL_USART_ERROR_DMARX;
        p_usrt->last_error = HAL_USART_ERROR_DMARX;
        p_usrt->rxbuffer.pos = p_usrt->rxbuffer.length;
        /* disable DMA receive, PERR, ERR interrupt */
        usart_dma_receive_config(p_usrt->periph, USART_DENR_DISABLE);
        usart_interrupt_disable(p_usrt->periph, USART_INT_PERR);
        usart_interrupt_disable(p_usrt->periph, USART_INT_ERR);
        /* reset rx_state */
        p_usrt->rx_state = USRT_STATE_FREE;
    }else{
        HAL_DEBUGE("usrt processor fatal error: dma error exception due to run state");
    }

    if(p_usrt->usrt_irq.error_handle != NULL){
        /* if there is a user error callback */
        p_usrt->usrt_irq.error_handle(p_usrt);
        p_usrt->error_state = HAL_USART_ERROR_NONE;
    }
}
