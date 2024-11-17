/*!
    \file    gd32e23x_hal_i2c.c
    \brief   I2C driver
    
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

#define SLAVE10_FIRST_BYTE(address)         ((0xF0) | (uint8_t)((address & 0x0300U)>>7U))
#define SLAVE10_SECOND_BYTE(address)        ((uint8_t)(address & 0x00FFU))
#define I2C_MEMORY_ADDRESS_MSB(address)     ((uint8_t)((address & 0xFF00U) >> 8U))  
#define I2C_MEMORY_ADDRESS_LSB(address)     ((uint8_t)(address & 0x00FFU))

/* wait the flag status until timeout */
static int32_t _i2c_wait_flag_timeout(uint32_t i2c_periph, i2c_flag_enum flag, \
                                      FlagStatus status, uint32_t timeout_ms);

/* master sends device address for write request */
static int32_t _i2c_master_write(hal_i2c_dev_struct *i2c, uint32_t timeout_ms);
/* event handler in I2C master transmit mode */
static void _i2c_master_transmit_interrupt(void *i2c);
/* DMA full finish handler in I2C master transmit mode */
static void _i2c_transmit_dma(void *dma);
/* handle the I2C DMA error process */
static void _i2c_dma_error(void *dma);

/* master sends device address for read request */
static int32_t _i2c_master_read(hal_i2c_dev_struct *i2c, uint32_t timeout_ms);
/* event handler in I2C master receive mode */
static void _i2c_master_receive_interrupt(void *i2c);
/* DMA full finish handler in I2C master receive mode */
static void _i2c_receive_dma(void *dma);

/* event handler in I2C slave transmit mode */
static void _i2c_slave_transmit_interrupt(void *i2c);
/* event handler in I2C slave receive mode */
static void _i2c_slave_receive_interrupt(void *i2c);
/* event handler in I2C slave serial receive mode */
static void _i2c_slave_serial_receive_interrupt(void *i2c);

/* master sends device address for memory write request */
static int32_t _i2c_memory_write(hal_i2c_dev_struct *i2c, uint32_t timeout_ms);
/* master sends device address for memory read request */
static int32_t _i2c_memory_read(hal_i2c_dev_struct *i2c, uint32_t timeout_ms);
/* event handler of memory write in I2C master mode */
static void _i2c_memmory_write_interrupt(void *i2c);
/* event handler of memory read in I2C master mode */
static void _i2c_memory_read_interrupt(void *i2c);

/* event handler for address listen in slave mode */
static void _i2c_address_listen_interrupt(void *i2c);

/*!
    \brief      initialize the I2C structure with the default values
    \param[in]  struct_type: refer to hal_i2c_struct_type_enum
    \param[in]  p_struct: pointer to I2C structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_i2c_struct_init(hal_i2c_struct_type_enum struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(struct_type){
        case HAL_I2C_INIT_STRUCT:
            /* initialize I2C initialization structure with the default values */
            ((hal_i2c_init_struct*)p_struct)->duty_cycle = I2C_DTCY_2;
            ((hal_i2c_init_struct*)p_struct)->clock_speed = 100000U;
            ((hal_i2c_init_struct*)p_struct)->address_format = I2C_ADDFORMAT_7BITS;
            ((hal_i2c_init_struct*)p_struct)->own_address1 = 0U;
            ((hal_i2c_init_struct*)p_struct)->dual_address = I2C_DUADEN_DISABLE;
            ((hal_i2c_init_struct*)p_struct)->own_address2 = 0U;
            ((hal_i2c_init_struct*)p_struct)->general_call = I2C_GCEN_DISABLE;
            ((hal_i2c_init_struct*)p_struct)->no_stretch = I2C_SCLSTRETCH_DISABLE;
            break;
        case HAL_I2C_DEV_STRUCT:
            /* initialize I2C device information structure with the default values */
            ((hal_i2c_dev_struct*)p_struct)->periph = 0;
            ((hal_i2c_dev_struct*)p_struct)->i2c_irq.error_handle = NULL;
            ((hal_i2c_dev_struct*)p_struct)->i2c_irq.event_handle = NULL;
            ((hal_i2c_dev_struct*)p_struct)->i2c_irq.rxframe_rise_handle = NULL;
            ((hal_i2c_dev_struct*)p_struct)->i2c_irq.rxframe_fall_handle = NULL;
            ((hal_i2c_dev_struct*)p_struct)->i2c_irq.txframe_rise_handle = NULL;
            ((hal_i2c_dev_struct*)p_struct)->i2c_irq.txframe_fall_handle = NULL;
            ((hal_i2c_dev_struct*)p_struct)->p_dma_rx = NULL;
            ((hal_i2c_dev_struct*)p_struct)->p_dma_tx = NULL;
            ((hal_i2c_dev_struct*)p_struct)->txbuffer.buffer = NULL;
            ((hal_i2c_dev_struct*)p_struct)->txbuffer.length = 0U;
            ((hal_i2c_dev_struct*)p_struct)->txbuffer.pos = 0U;
            ((hal_i2c_dev_struct*)p_struct)->rxbuffer.buffer = NULL;
            ((hal_i2c_dev_struct*)p_struct)->rxbuffer.length = 0U;
            ((hal_i2c_dev_struct*)p_struct)->rxbuffer.pos = 0U;
            ((hal_i2c_dev_struct*)p_struct)->rxbuffer.pos = 0U;
            ((hal_i2c_dev_struct*)p_struct)->error_state = HAL_I2C_ERROR_NONE;
            ((hal_i2c_dev_struct*)p_struct)->tx_state = I2C_STATE_READY;
            ((hal_i2c_dev_struct*)p_struct)->rx_state = I2C_STATE_READY;
            ((hal_i2c_dev_struct*)p_struct)->previous_state = HAL_I2C_PREVIOUS_STATE_NONE;
            ((hal_i2c_dev_struct*)p_struct)->last_error = HAL_I2C_ERROR_NONE;
            ((hal_i2c_dev_struct*)p_struct)->slave_address.device_address = 0U;
            ((hal_i2c_dev_struct*)p_struct)->slave_address.memory_address = 0U;
            ((hal_i2c_dev_struct*)p_struct)->slave_address.address_size = I2C_MEMORY_ADDRESS_8BIT;
            ((hal_i2c_dev_struct*)p_struct)->slave_address.address_complete = RESET;
            ((hal_i2c_dev_struct*)p_struct)->slave_address.address_count = 0U;
            ((hal_i2c_dev_struct*)p_struct)->slave_address.second_addressing = RESET;
            ((hal_i2c_dev_struct*)p_struct)->transfer_option = I2C_NO_OPTION_TRANSFER;
            break;
        case HAL_I2C_IRQ_STRUCT:
            /* initialize I2C irq structure with the default values */
            ((hal_i2c_irq_struct*)p_struct)->error_handle = NULL;
            ((hal_i2c_irq_struct*)p_struct)->event_handle = NULL;
            ((hal_i2c_irq_struct*)p_struct)->rxframe_rise_handle = NULL;
            ((hal_i2c_irq_struct*)p_struct)->rxframe_fall_handle = NULL;
            ((hal_i2c_irq_struct*)p_struct)->txframe_rise_handle = NULL;
            ((hal_i2c_irq_struct*)p_struct)->txframe_fall_handle = NULL;
        default:
            HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
            break;
    }
}

/*!
    \brief      deinitialize I2C
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2c_deinit(hal_i2c_dev_struct *i2c)
{
    uint32_t periph;
    
    periph = i2c->periph;
    if((I2C0 == periph) || (I2C1 == periph)){
        /* deinitialize the periph and the device information sturcture */
        i2c_deinit(periph);
        hal_i2c_struct_init(HAL_I2C_DEV_STRUCT, i2c);
        i2c->periph = periph;
    }else{
        HAL_DEBUGE("parameter [i2c->periph] value is invalid");
    }
}

/*!
    \brief      initialize I2C
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which I2C is initialized
    \param[in]  p_init: the initialization data needed to initialize I2C
                  duty_cycle: I2C_DTCY_2, I2C_DTCY_16_9
                  clock_speed: I2C clock speed, supports standard mode (up to 100 kHz), fast mode 
                               (up to 400 kHz) and fast mode plus (up to 1MHz)
                  address_format: I2C_ADDFORMAT_7BITS, I2C_ADDFORMAT_10BITS
                  own_address1: I2C address
                  dual_address: I2C_DUADEN_ENABLE, I2C_DUADEN_DISABLE
                  own_address2: the second address in dual-address mode
                  general_call: I2C_GCEN_ENABLE, I2C_GCEN_DISABLE
                  no_stretch: I2C_SCLSTRETCH_ENABLE, I2C_SCLSTRETCH_DISABLE
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_i2c_init(hal_i2c_dev_struct *i2c, uint32_t periph, \
                     hal_i2c_init_struct *p_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check I2C pointer and p_init address */
    if((NULL == i2c) && (NULL == p_init)){
        HAL_DEBUGE("pointer [i2c] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check periph parameter */
    if((I2C0 != periph) && (I2C1 != periph)){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }
    /* check periph value from I2C device struct */
    if(0 != i2c->periph){
        HAL_DEBUGI("periph value from i2c device struct has been rewrite");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    i2c->periph = periph;
    /* configure I2C clock */
    i2c_clock_config(periph, p_init->clock_speed, p_init->duty_cycle);
    /* configure I2C address */
    i2c_mode_addr_config(periph, I2C_I2CMODE_ENABLE, p_init->address_format, \
                         p_init->own_address1);
    /* configure dual-address mode */
    if(I2C_DUADEN_ENABLE == p_init->dual_address){
        i2c_dualaddr_enable(periph, p_init->own_address2);
    }else{
        i2c_dualaddr_disable(periph);
    }
    /* configure whether to stretch SCL low when data is not ready in slave mode */
    i2c_stretch_scl_low_config(periph, p_init->no_stretch);
    /* whether or not to response to a general call */
    i2c_slave_response_to_gcall_config(periph, p_init->general_call);
    /* enable I2C */
    i2c_enable(periph);
    /* enable acknowledge */
    i2c_ack_config(periph, I2C_ACK_ENABLE);
    i2c->tx_state = I2C_STATE_READY;
    i2c->rx_state = I2C_STATE_READY;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      I2C error interrupt handler
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2c_error_irq(hal_i2c_dev_struct *i2c)
{
    /* no acknowledge received */
    if(i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_AERR)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_AERR);
        i2c->error_state |= HAL_I2C_ERROR_AERR;
        i2c->last_error = HAL_I2C_ERROR_AERR;
    }

    /* over-run or under-run when SCL stretch is disabled */
    if(i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_OUERR)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_OUERR);
        i2c->error_state |= HAL_I2C_ERROR_OUERR;
        i2c->last_error = HAL_I2C_ERROR_OUERR;
    }

    /* arbitration lost */
    if(i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_LOSTARB)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_LOSTARB);
        i2c->error_state |= HAL_I2C_ERROR_LOSTARB;
        i2c->last_error = HAL_I2C_ERROR_LOSTARB;
    }

    /* bus error */
    if(i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_BERR)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_BERR);
        i2c->error_state |= HAL_I2C_ERROR_BERR;
        i2c->last_error = HAL_I2C_ERROR_BERR;
    }

    /* CRC value doesn't match */
    if(i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_PECERR)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_PECERR);
        i2c->error_state |= HAL_I2C_ERROR_PECERR;
        i2c->last_error = HAL_I2C_ERROR_PECERR;
    }
    
    if(HAL_I2C_ERROR_NONE != i2c->error_state){
        if(i2c->i2c_irq.error_handle != NULL){
            i2c->i2c_irq.error_handle(i2c);
            i2c->error_state = HAL_I2C_ERROR_NONE;
        }
    }
}

/*!
    \brief      I2C evevt interrupt handler
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2c_event_irq(hal_i2c_dev_struct *i2c)
{
    if(i2c->i2c_irq.event_handle != NULL){
        i2c->i2c_irq.event_handle(i2c);
        return;
    }
    
    /* rxframe rise interrupt handle */
    if(RESET != i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_RFR)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_RFR);
        if(NULL != i2c->i2c_irq.rxframe_rise_handle){
            i2c->i2c_irq.rxframe_rise_handle(i2c);
        }
        return;
    }
    /* rxframe fall interrupt handle */
    if(RESET != i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_RFF)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_RFF);
        if(NULL != i2c->i2c_irq.rxframe_fall_handle){
            i2c->i2c_irq.rxframe_fall_handle(i2c);
        }
        return;
    }
    /* txframe rise interrupt handle */
    if(RESET != i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_TFR)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_TFR);
        if(NULL != i2c->i2c_irq.txframe_rise_handle){
            i2c->i2c_irq.txframe_rise_handle(i2c);
        }
        return;
    }
    /* txframe fall interrupt handle */
    if(RESET != i2c_interrupt_flag_get(i2c->periph, I2C_INT_FLAG_TFF)){
        i2c_interrupt_flag_clear(i2c->periph, I2C_INT_FLAG_TFF);
        if(NULL != i2c->i2c_irq.txframe_fall_handle){
            i2c->i2c_irq.txframe_fall_handle(i2c);
        }
        return;
    }
}

/*!
    \brief      start I2C module function
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     
*/
void hal_i2c_start(hal_i2c_dev_struct *i2c)
{
    i2c_enable(i2c->periph);
}

/*!
    \brief      stop I2C module function
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     
*/
void hal_i2c_stop(hal_i2c_dev_struct *i2c)
{
    i2c_disable(i2c->periph);
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: the structure that contains callback handlers of I2C interrupt
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
void hal_i2c_irq_handle_set(hal_i2c_dev_struct *i2c, hal_i2c_irq_struct *p_irq)
{
    /* event interrupt handler set */
    if(NULL != p_irq->event_handle){
        i2c->i2c_irq.event_handle = p_irq->event_handle;
        i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
        i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    }else{
        i2c->i2c_irq.event_handle = NULL;
        i2c_interrupt_disable(i2c->periph, I2C_INT_BUF);
        i2c_interrupt_disable(i2c->periph, I2C_INT_EV);
    }
    /* error interrupt handler set */
    if(NULL != p_irq->error_handle){
        i2c->i2c_irq.error_handle = p_irq->error_handle;
        i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    }else{
        i2c->i2c_irq.error_handle = NULL;
        i2c_interrupt_disable(i2c->periph, I2C_INT_ERR);
    }
    /* RFR interrupt handler set */
    if(NULL != p_irq->rxframe_rise_handle){
        i2c->i2c_irq.rxframe_rise_handle = p_irq->rxframe_rise_handle;
        i2c_interrupt_enable(i2c->periph, I2C_INT_RFR);
    }else{
        i2c->i2c_irq.rxframe_rise_handle = NULL;
        i2c_interrupt_disable(i2c->periph, I2C_INT_RFR);
    }
    /* RFF interrupt handler set */
    if(NULL != p_irq->rxframe_fall_handle){
        i2c->i2c_irq.rxframe_fall_handle = p_irq->rxframe_fall_handle;
        i2c_interrupt_enable(i2c->periph, I2C_INT_RFF);
    }else{
        i2c->i2c_irq.rxframe_fall_handle = NULL;
        i2c_interrupt_disable(i2c->periph, I2C_INT_RFF);
    }
    /* TFR interrupt handler set */
    if(NULL != p_irq->txframe_rise_handle){
        i2c->i2c_irq.txframe_rise_handle = p_irq->txframe_rise_handle;
        i2c_interrupt_enable(i2c->periph, I2C_INT_TFR);
    }else{
        i2c->i2c_irq.txframe_rise_handle = NULL;
        i2c_interrupt_disable(i2c->periph, I2C_INT_TFR);
    }
    /* TFF interrupt handler set */
    if(NULL != p_irq->txframe_fall_handle){
        i2c->i2c_irq.txframe_fall_handle = p_irq->txframe_fall_handle;
        i2c_interrupt_enable(i2c->periph, I2C_INT_TFF);
    }else{
        i2c->i2c_irq.txframe_fall_handle = NULL;
        i2c_interrupt_disable(i2c->periph, I2C_INT_TFF);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2c_irq_handle_all_reset(hal_i2c_dev_struct *i2c)
{
    /* configure interrupt callback function to NULL */
    i2c->i2c_irq.event_handle = NULL;
    i2c->i2c_irq.error_handle = NULL;
    i2c->i2c_irq.rxframe_rise_handle = NULL;
    i2c->i2c_irq.rxframe_fall_handle = NULL;
    i2c->i2c_irq.txframe_rise_handle = NULL;
    i2c->i2c_irq.txframe_fall_handle = NULL;
}

/*!
    \brief      transmit amounts of data in master mode, poll transmit process and completed status,
                the function is blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_master_transmit_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* initialize transmit parameters */
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    data_length = i2c->txbuffer.length;
      
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    
    i2c->tx_state = I2C_STATE_BUSY;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    /* master sends device address for write request */
    if(HAL_ERR_NONE != _i2c_master_write(i2c, timeout_ms)){
        return HAL_ERR_TIMEOUT;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    while(i2c->txbuffer.pos < data_length){
        /* wait until the transmission data register is empty */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_TBE, SET, timeout_ms)){
            HAL_DEBUGW("i2c get TBE timeout");
            return HAL_ERR_TIMEOUT;
        }
        i2c_data_transmit(i2c->periph, *i2c->txbuffer.buffer);
        i2c->txbuffer.buffer++;
        i2c->txbuffer.pos++;
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(i2c->periph);
    while(I2C_CTL0(i2c->periph)&0x0200);
    i2c->tx_state = I2C_STATE_READY;
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data in master mode, poll receive process and completed 
                status, the function is blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be read 
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_master_receive_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, uint32_t timeout_ms)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* initialize receive parameters */
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer;
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }

    i2c->rx_state = I2C_STATE_BUSY;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    /* master sends device address for read request */
    if(HAL_ERR_NONE != _i2c_master_read(i2c, timeout_ms)){
        return HAL_ERR_TIMEOUT;
    }
    if(1 == i2c->rxbuffer.length){
        /* disable acknowledge */
        i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
        /* send a stop condition to I2C bus*/
        i2c_stop_on_bus(i2c->periph);
    }else if(2 == i2c->rxbuffer.length){
        /* send a NACK for the next data byte which will be received into the shift register */
        i2c_ackpos_config(i2c->periph, I2C_ACKPOS_NEXT);
        /* disable acknowledge */
        i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    }else{
        /* enable acknowledge */
        i2c_ack_config(i2c->periph, I2C_ACK_ENABLE);
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    }
    while(i2c->rxbuffer.length>0){
        if(i2c->rxbuffer.length <= 3){
            if(1 == i2c->rxbuffer.length){
                /* wait until the RBNE bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get RBNE timeout");
                    return HAL_ERR_TIMEOUT;
                }

                *i2c->rxbuffer.buffer = i2c_data_receive(i2c->periph);
                i2c->rxbuffer.buffer++;
                i2c->rxbuffer.length--;
            }else if(2 == i2c->rxbuffer.length){
                i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
                /* wait until the BTC bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_BTC, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get BTC timeout");
                    return HAL_ERR_TIMEOUT;
                }
                /* wait until the RBNE bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get RBNE timeout");
                    return HAL_ERR_TIMEOUT;
                }
                
                *i2c->rxbuffer.buffer = i2c_data_receive(i2c->periph);
                i2c->rxbuffer.buffer++;
                i2c->rxbuffer.length--;
                /* wait until the RBNE bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get RBNE timeout");
                    return HAL_ERR_TIMEOUT;
                }

                *i2c->rxbuffer.buffer = i2c_data_receive(i2c->periph);
                i2c->rxbuffer.buffer++;
                i2c->rxbuffer.length--;
                i2c_stop_on_bus(i2c->periph);
            }else{
                /* wait until the BTC bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_BTC, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get BTC timeout");
                    return HAL_ERR_TIMEOUT;
                }

                i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
                *i2c->rxbuffer.buffer = i2c_data_receive(i2c->periph);
                i2c->rxbuffer.buffer++;
                i2c->rxbuffer.length--;
                /* wait until the RBNE bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get RBNE timeout");
                    return HAL_ERR_TIMEOUT;
                }

                *i2c->rxbuffer.buffer = i2c_data_receive(i2c->periph);
                i2c->rxbuffer.buffer++;
                i2c->rxbuffer.length--;
                /* wait until the RBNE bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get RBNE timeout");
                    return HAL_ERR_TIMEOUT;
                }

                *i2c->rxbuffer.buffer = i2c_data_receive(i2c->periph);
                i2c->rxbuffer.buffer++;
                i2c->rxbuffer.length--;
                /* send a stop condition to I2C bus */
                i2c_stop_on_bus(i2c->periph);
            }
        }else{
            /* wait until the RBNE bit is set */
            if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
                HAL_DEBUGW("i2c get RBNE timeout");
                return HAL_ERR_TIMEOUT;
            }
            *i2c->rxbuffer.buffer = i2c_data_receive(i2c->periph);
            i2c->rxbuffer.buffer++;
            i2c->rxbuffer.length--;
        }
    }
    i2c->rx_state = I2C_STATE_READY;
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data in slave mode, poll transmit process and completed status,
                the function is blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_slave_transmit_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, uint32_t timeout_ms)
{
    uint32_t address_format;
    uint32_t data_length;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* check the address format */
    if(I2C_SADDR0(i2c->periph)&I2C_SADDR0_ADDFORMAT){
        address_format = I2C_ADDFORMAT_10BITS;
    }else{
        address_format = I2C_ADDFORMAT_7BITS;
    }
    /* initialize transmit parameters */
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    data_length = i2c->txbuffer.length;
    
    i2c->tx_state = I2C_STATE_BUSY;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c_ack_config(i2c->periph, I2C_ACK_ENABLE);
    /* wait until ADDSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    if(I2C_ADDFORMAT_10BITS == address_format){
        /* wait until ADDSEND bit is set */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
            HAL_DEBUGW("i2c get ADDSEND timeout");
            return HAL_ERR_TIMEOUT;
        }
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    }
    while(data_length > i2c->txbuffer.pos){
        /* wait until the TBE bit is set */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_TBE, SET, timeout_ms)){
            HAL_DEBUGW("i2c get TBE timeout");
            return HAL_ERR_TIMEOUT;
        }
        i2c_data_transmit(i2c->periph, *i2c->txbuffer.buffer);
        i2c->txbuffer.buffer++;
        i2c->txbuffer.pos++;
    }
    /* the master doesn't acknowledge for the last byte */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_AERR, SET, timeout_ms)){
        HAL_DEBUGW("i2c get AERR timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* clear the bit of AERR */
    i2c_flag_clear(i2c->periph, I2C_FLAG_AERR);
    /* disable acknowledge */
    i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
    i2c->tx_state = I2C_STATE_READY;
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data in slave mode, poll receive process and completed 
                status, the function is blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be read 
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_slave_receive_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize receive parameters */
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer;
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    data_length = i2c->rxbuffer.length;
    
    i2c->rx_state = I2C_STATE_BUSY;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c_ack_config(i2c->periph, I2C_ACK_ENABLE);
    /* wait until ADDSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    
    while(i2c->rxbuffer.pos < data_length){
        /* wait until the RBNE bit is set */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
            HAL_DEBUGW("i2c get RBNE timeout");
            return HAL_ERR_TIMEOUT;
        }
        *i2c->rxbuffer.buffer = i2c_data_receive(i2c->periph);
        i2c->rxbuffer.buffer++;
        i2c->rxbuffer.pos++;
    }
    /* wait until the STPDET bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_STPDET, SET, timeout_ms)){
        HAL_DEBUGW("i2c get stop timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* clear the STPDET bit */
    i2c_enable(i2c->periph);
    i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
    i2c->rx_state = I2C_STATE_READY;
    return HAL_ERR_NONE;
}

/*!
    \brief      write amounts of data to memory, poll transmit process and completed status,
                the function is blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_VAL, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_memory_write_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check whether the address size is legal */
    if((I2C_MEMORY_ADDRESS_8BIT != i2c->slave_address.address_size) && \
            (I2C_MEMORY_ADDRESS_16BIT != i2c->slave_address.address_size)){
        HAL_DEBUGE("parameter [address_size] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state whether it is busy or not */
    if(I2C_STATE_MEMORY_BUSY_TX == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* initialize transmit parameters */
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    data_length = i2c->txbuffer.length;
    
    i2c->tx_state = I2C_STATE_MEMORY_BUSY_TX;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    /* master sends device address for memory write request */
    if(HAL_ERR_NONE != _i2c_memory_write(i2c, timeout_ms)){
        return HAL_ERR_TIMEOUT;
    }
    
    while(i2c->txbuffer.pos < data_length){
        /* wait until the transmission data register is empty */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_TBE, SET, timeout_ms)){
            HAL_DEBUGW("i2c get TBE timeout");
            return HAL_ERR_TIMEOUT;
        }
        i2c_data_transmit(i2c->periph, *i2c->txbuffer.buffer);
        i2c->txbuffer.buffer++;
        i2c->txbuffer.pos++;
        /* wait until BTC bit is set */
        if((SET == i2c_flag_get(i2c->periph, I2C_FLAG_BTC)) && (i2c->txbuffer.pos != data_length)){
            HAL_DEBUGW("i2c get BTC timeout");
            return HAL_ERR_TIMEOUT;
        }
    }
    /* send a stop condition to I2C bus */
    i2c_stop_on_bus(i2c->periph);
    while(I2C_CTL0(i2c->periph)&0x0200);
    i2c->tx_state = I2C_STATE_READY;
    return HAL_ERR_NONE;
}

/*!
    \brief      read amounts of data from memory, poll receive process and completed status,
                the function is blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be read 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_VAL, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_memory_read_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, uint32_t timeout_ms)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check whether the address size is legal */
    if((I2C_MEMORY_ADDRESS_8BIT != i2c->slave_address.address_size) && \
                (I2C_MEMORY_ADDRESS_16BIT != i2c->slave_address.address_size)){
        HAL_DEBUGE("parameter [address_size] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state whether it is busy or not */
    if(I2C_STATE_MEMORY_BUSY_RX == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    
    i2c->rx_state = I2C_STATE_MEMORY_BUSY_RX;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    if(2 == length){
        /* send a NACK for the next data byte which will be received into the shift register */
        i2c_ackpos_config(i2c->periph, I2C_ACKPOS_NEXT);
    }
    /* master sends device address for memory read request */
    if(HAL_ERR_NONE != _i2c_memory_read(i2c, timeout_ms)){
        return HAL_ERR_TIMEOUT;
    }

    if(1 == length){
        /* disable acknowledge */
        i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
        /* send a stop condition to I2C bus */
        i2c_stop_on_bus(i2c->periph);
    }else if(2 == length){
        /* disable acknowledge */
        i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    }else{
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    }
    while(length > 0){
        if(length <= 3){
            if(1 == length){
                /* wait until RBNE bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get RBNE timeout");
                    return HAL_ERR_TIMEOUT;
                }
                
                (*p_buffer++) = i2c_data_receive(i2c->periph);
                length--;
            }else if(2 == length){
                i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
                /* wait until BTC bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_BTC, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get BTC timeout");
                    return HAL_ERR_TIMEOUT;
                }
                /* send a stop condition to I2C bus */
                i2c_stop_on_bus(i2c->periph);
                (*p_buffer++) = i2c_data_receive(i2c->periph);
                length--;
                (*p_buffer++) = i2c_data_receive(i2c->periph);
                length--;
            }else{
                /* wait until BTC bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_BTC, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get BTC timeout");
                    return HAL_ERR_TIMEOUT;
                }
                /* disable acknowledge */
                i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
                (*p_buffer++) = i2c_data_receive(i2c->periph);
                length--;
                /* wait until BTC bit is set */
                if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_BTC, SET, timeout_ms)){
                    HAL_DEBUGW("i2c get BTC timeout");
                    return HAL_ERR_TIMEOUT;
                }
                /* send a stop condition to I2C bus */
                i2c_stop_on_bus(i2c->periph);
                (*p_buffer++) = i2c_data_receive(i2c->periph);
                length--;
                (*p_buffer++) = i2c_data_receive(i2c->periph);
                length--;
            }
        }else{
            /* wait until RBNE bit is set */
            if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_RBNE, SET, timeout_ms)){
                HAL_DEBUGW("i2c get RBNE timeout");
                return HAL_ERR_TIMEOUT;
            }
            (*p_buffer++) = i2c_data_receive(i2c->periph);
            length--;
            /* wait until BTC bit is set */
            if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_BTC, SET, timeout_ms)){
                HAL_DEBUGW("i2c get BTC timeout");
                return HAL_ERR_TIMEOUT;
            }
            (*p_buffer++) = i2c_data_receive(i2c->periph);
            length--;
        }
    }
    i2c->rx_state = I2C_STATE_READY;
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data in master mode by interrupt method, the function 
                is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_master_transmit_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* the master waits until the I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* initialize transmit parameters */
    i2c->tx_state = I2C_STATE_BUSY;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    i2c->tx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_master_transmit_interrupt;
    
    /* the master sends a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);    
    /* enable the I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data in master mode by interrupt method, the function 
                is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be read 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_master_receive_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                        uint32_t length, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* initialize receive parameters */
    i2c->rx_state = I2C_STATE_BUSY;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer;
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    i2c->rx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_master_receive_interrupt;
    if(2 == i2c->rxbuffer.length){
        /* send a NACK for the next data byte which will be received into the shift register */
        i2c_ackpos_config(i2c->periph, I2C_ACKPOS_NEXT);
    }
    
    /* enable the I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    /* the master sends a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data in slave mode by interrupt method, the function is 
                non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_slave_transmit_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize transmit parameters */
    i2c->tx_state = I2C_STATE_BUSY;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    i2c->tx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_slave_transmit_interrupt;

    /* enable I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data in slave mode by interrupt method, the function 
                is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be receive
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_slave_receive_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    /* initialize transmit parameters */
    i2c->rx_state = I2C_STATE_BUSY;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer;
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    i2c->rx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_slave_receive_interrupt;
    /* enable I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    return HAL_ERR_NONE;
}

/*!
    \brief      write amounts of data to memory by interrupt method, the function is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be write 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_VAL, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_memory_write_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer, p_buffer address and length */
    if((NULL == i2c) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check whether the address size is legal */
    if((I2C_MEMORY_ADDRESS_8BIT != i2c->slave_address.address_size) && \
                (I2C_MEMORY_ADDRESS_16BIT != i2c->slave_address.address_size)){
        HAL_DEBUGE("parameter [address_size] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state whether it is busy or not */
    if(I2C_STATE_MEMORY_BUSY_TX == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    /* initialize transmit parameters */
    i2c->tx_state = I2C_STATE_MEMORY_BUSY_TX;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    i2c->tx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_memmory_write_interrupt;

    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);
    /* enable I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      read amounts of data from memory by interrupt method, the function is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be read
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_memory_read_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer, p_buffer address and length */
    if((NULL == i2c) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check whether the address size is legal */
    if((I2C_MEMORY_ADDRESS_8BIT != i2c->slave_address.address_size) && \
                (I2C_MEMORY_ADDRESS_16BIT != i2c->slave_address.address_size)){
        HAL_DEBUGE("parameter [address_size] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state whether it is busy or not */
    if(I2C_STATE_MEMORY_BUSY_RX == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize transmit parameters */
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer;
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    i2c->rx_state = I2C_STATE_MEMORY_BUSY_RX;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c->rx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_memory_read_interrupt;
    if(2 == i2c->rxbuffer.length){
        i2c_ackpos_config(i2c->periph, I2C_ACKPOS_NEXT);
    }    

    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);
    /* enable I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data in master mode by dma method, the function 
                is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_master_transmit_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer, p_buffer address and length */
    if((NULL == i2c) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check whether the pointer of DMA tx is NULL */
    if(NULL == i2c->p_dma_tx){
        HAL_DEBUGE("parameter [i2c->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check whether the tx state is ready */
    if(I2C_STATE_BUSY == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    /* initialize transmit parameters */
    i2c->tx_state = I2C_STATE_BUSY;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    i2c->txbuffer.buffer = (uint8_t *)p_buffer; 
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    i2c->tx_callback = (void *)p_user_func;
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _i2c_transmit_dma;
    dma_irq.error_handle = _i2c_dma_error; 
    if(NULL != i2c->p_dma_tx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = i2c->p_dma_tx->dma_irq.half_finish_handle;
    }
    /* master sends device address for write request */
    if(HAL_ERR_NONE != _i2c_master_write(i2c, I2C_TIMEOUT)){
        return HAL_ERR_TIMEOUT;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    /* enable I2C DMA */
    i2c_dma_enable(i2c->periph, I2C_DMA_ON);
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(i2c->p_dma_tx, (uint32_t)i2c->txbuffer.buffer, \
                           (uint32_t)&I2C_DATA(i2c->periph), length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(i2c->p_dma_tx->channel, DMA_FLAG_G);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data in master mode by dma method, the function is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be receive 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_master_receive_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer, p_buffer address and length */
    if((NULL == i2c) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check whether the pointer of DMA tx is NULL */
    if(NULL == i2c->p_dma_rx){
        HAL_DEBUGE("parameter [i2c->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check whether the rx state is ready */
    if(I2C_STATE_BUSY == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    /* initialize receive parameters */
    i2c->rx_state = I2C_STATE_BUSY;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer; 
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    i2c->rx_callback = (void *)p_user_func;
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _i2c_receive_dma;
    dma_irq.error_handle = _i2c_dma_error; 
    if(NULL != i2c->p_dma_rx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = i2c->p_dma_rx->dma_irq.half_finish_handle;
    }
    /* master sends device address for read request */
    if(HAL_ERR_NONE != _i2c_master_read(i2c, I2C_TIMEOUT)){
        return HAL_ERR_TIMEOUT;
    }
    if(1 == length){
        /* disable acknowledge */
        i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
    }else{
        /* configure the next DMA EOT is DMA last transfer */
        i2c_dma_last_transfer_config(i2c->periph, I2C_DMALST_ON);
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    /* enable I2C DMA */
    i2c_dma_enable(i2c->periph, I2C_DMA_ON);
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(i2c->p_dma_rx, (uint32_t)&I2C_DATA(i2c->periph),\
                            (uint32_t)i2c->rxbuffer.buffer, length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(i2c->p_dma_rx->channel, DMA_FLAG_G);

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data in slave mode by dma method, the function is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_slave_transmit_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func)
{
    hal_dma_irq_struct dma_irq;
    uint32_t address_format;
    
    /* check the address format */
    if(I2C_SADDR0(i2c->periph)&I2C_SADDR0_ADDFORMAT){
        address_format = I2C_ADDFORMAT_10BITS;
    }else{
        address_format = I2C_ADDFORMAT_7BITS;
    }
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer, p_buffer address and length */
    if((NULL == i2c) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check whether the pointer of DMA tx is NULL */
    if(NULL == i2c->p_dma_tx){
        HAL_DEBUGE("parameter [i2c->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check whether the tx state is ready */
    if(I2C_STATE_BUSY == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    /* initialize transmit parameters */
    i2c->tx_state = I2C_STATE_BUSY;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    i2c->txbuffer.buffer = (uint8_t *)p_buffer; 
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    i2c->tx_callback = (void *)p_user_func;
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _i2c_transmit_dma;
    dma_irq.error_handle = _i2c_dma_error; 
    if(NULL != i2c->p_dma_tx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = i2c->p_dma_tx->dma_irq.half_finish_handle;
    }
    /* enable acknowledge */
    i2c_ack_config(i2c->periph, I2C_ACK_ENABLE);
    /* wait until ADDSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, I2C_ADDR_TIMEOUT)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    /* wait until ADDSEND bit is set */
    if(I2C_ADDFORMAT_10BITS == address_format){
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, I2C_ADDR_TIMEOUT)){
            HAL_DEBUGW("i2c get ADDSEND timeout");
            return HAL_ERR_TIMEOUT;
        }
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    }
    /* enable I2C DMA */
    i2c_dma_enable(i2c->periph, I2C_DMA_ON);
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(i2c->p_dma_tx, (uint32_t)i2c->txbuffer.buffer, \
                           (uint32_t)&I2C_DATA(i2c->periph), length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(i2c->p_dma_tx->channel, DMA_FLAG_G);

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data in slave mode by dma method, the function is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be receive 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_slave_receive_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer, p_buffer address and length */
    if((NULL == i2c) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

    /* check whether the pointer of DMA tx is NULL */
    if(NULL == i2c->p_dma_rx){
        HAL_DEBUGE("parameter [i2c->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check whether the rx state is ready */
    if(I2C_STATE_BUSY == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    /* initialize receive parameters */
    i2c->rx_state = I2C_STATE_BUSY;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer; 
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    i2c->rx_callback = (void *)p_user_func;
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _i2c_receive_dma;
    dma_irq.error_handle = _i2c_dma_error; 
    if(NULL != i2c->p_dma_rx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = i2c->p_dma_rx->dma_irq.half_finish_handle;
    }

    /* enable acknowledge */
    i2c_ack_config(i2c->periph, I2C_ACK_ENABLE);
    /* wait until ADDSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, I2C_ADDR_TIMEOUT)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    /* enable I2C DMA */
    i2c_dma_enable(i2c->periph, I2C_DMA_ON);
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(i2c->p_dma_rx, (uint32_t)&I2C_DATA(i2c->periph), \
                            (uint32_t)i2c->rxbuffer.buffer, length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(i2c->p_dma_rx->channel, DMA_FLAG_G);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      write amounts of data to memory by dma method, the function is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be write 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_memory_write_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer, p_buffer address and length */
    if((NULL == i2c) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check whether the address size is legal */
    if((I2C_MEMORY_ADDRESS_8BIT != i2c->slave_address.address_size) && \
                (I2C_MEMORY_ADDRESS_16BIT != i2c->slave_address.address_size)){
        HAL_DEBUGE("parameter [address_size] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    /* initialize transmit parameters */
    i2c->tx_state = I2C_STATE_MEMORY_BUSY_TX;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    i2c->tx_callback = (void *)p_user_func;
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _i2c_transmit_dma;
    dma_irq.error_handle = _i2c_dma_error; 
    if(NULL != i2c->p_dma_tx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = i2c->p_dma_tx->dma_irq.half_finish_handle;
    }
    /* master sends device address for memory write request */
    if(HAL_ERR_NONE != _i2c_memory_write(i2c, I2C_TIMEOUT)){
        return HAL_ERR_TIMEOUT;
    }

    /* enable I2C DMA */
    i2c_dma_enable(i2c->periph, I2C_DMA_ON);
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(i2c->p_dma_tx, (uint32_t)i2c->txbuffer.buffer, \
                           (uint32_t)&I2C_DATA(i2c->periph), length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(i2c->p_dma_tx->channel, DMA_FLAG_G);

    return HAL_ERR_NONE;
}

/*!
    \brief      read amounts of data from memory by dma method, the function is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be read 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_memory_read_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer, p_buffer address and length */
    if((NULL == i2c) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    /* check whether the address size is legal */
    if((I2C_MEMORY_ADDRESS_8BIT != i2c->slave_address.address_size) && \
                    (I2C_MEMORY_ADDRESS_16BIT != i2c->slave_address.address_size)){
        HAL_DEBUGE("parameter [address_size] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* wait until I2C bus is idle */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
        HAL_DEBUGW("i2c busy timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    /* initialize receive parameters */
    i2c->rx_state = I2C_STATE_MEMORY_BUSY_RX;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer;
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    i2c->rx_callback = (void *)p_user_func;
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _i2c_transmit_dma;
    dma_irq.error_handle = _i2c_dma_error; 
    if(NULL != i2c->p_dma_rx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = i2c->p_dma_rx->dma_irq.half_finish_handle;
    }

    /* master sends device address for memory read request */
    if(HAL_ERR_NONE != _i2c_memory_read(i2c, I2C_TIMEOUT)){
        return HAL_ERR_TIMEOUT;
    }
    if(2 == length){
        /* send a NACK for the next data byte which will be received into the shift register */
        i2c_ackpos_config(i2c->periph, I2C_ACKPOS_NEXT);
    }
    if(1 == length){
        /* disable acknowledge */
        i2c_ack_config(i2c->periph, I2C_ACK_DISABLE);
    }else{
        /* configure the next DMA EOT as the DMA last transfer */
        i2c_dma_last_transfer_config(i2c->periph, I2C_DMALST_ON);
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    /* enable I2C DMA */
    i2c_dma_enable(i2c->periph, I2C_DMA_ON);
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(i2c->p_dma_rx, (uint32_t)&I2C_DATA(i2c->periph), \
                            (uint32_t)i2c->rxbuffer.buffer, length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(i2c->p_dma_rx->channel, DMA_FLAG_G);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      check whether the device is ready for access
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_NONE, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_device_ready_check(hal_i2c_dev_struct *i2c, uint32_t timeout_ms)
{
    __IO uint32_t val = 0;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check i2c pointer */
    if((NULL == i2c)){
        HAL_DEBUGE("parameter [i2c] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    while(1){
        while(i2c_flag_get(i2c->periph, I2C_FLAG_I2CBSY));
        /* wait until I2C bus is idle */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
            HAL_DEBUGW("i2c busy timeout");
            return HAL_ERR_TIMEOUT;
        }
        /* check if I2C is already enabled */
        if(I2C_CTL0_I2CEN != (I2C_CTL0(i2c->periph) & I2C_CTL0_I2CEN)){
            i2c_enable(i2c->periph);
        }
        
        /* send a start condition to I2C bus */
        i2c_start_on_bus(i2c->periph);
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_SBSEND, SET, timeout_ms)){
            HAL_DEBUGW("i2c get SBSEND timeout");
            return HAL_ERR_TIMEOUT;
        }
        /* wait until SBSEND bit is set */
        while(!i2c_flag_get(i2c->periph, I2C_FLAG_SBSEND));
        /* send slave address to I2C bus */
        i2c_master_addressing(i2c->periph, i2c->slave_address.device_address, I2C_TRANSMITTER);
        
        do{
            val = I2C_STAT0(i2c->periph);
        }while(0 == (val & (I2C_STAT0_ADDSEND | I2C_STAT0_AERR)));
        
        if(val & I2C_STAT0_ADDSEND){

            /* clear ADDSEND bit */
            i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(i2c->periph);
            /* wait until I2C bus is idle */
            if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
                HAL_DEBUGW("i2c busy timeout");
                return HAL_ERR_TIMEOUT;
            }
            return HAL_ERR_NONE;
        }else{
            /* clear AERR bit */
            i2c_flag_clear(i2c->periph, I2C_FLAG_AERR);
            /* send a stop condition to I2C bus */
            i2c_stop_on_bus(i2c->periph);
            while(I2C_CTL0(i2c->periph)&0x0200);
            
            /* wait until I2C bus is idle */
            if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
                HAL_DEBUGW("i2c busy timeout");
                return HAL_ERR_TIMEOUT;
            }
        }
    }
}

/*!
    \brief      serial transmit amounts of data in master mode by interrupt method, the function 
                is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_master_serial_transmit_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func)
{
    __IO uint32_t previous_state = HAL_I2C_PREVIOUS_STATE_NONE;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }

    if((I2C_FIRST_TRANSFER == i2c->transfer_option) || \
        (I2C_NO_OPTION_TRANSFER == i2c->transfer_option)){
        /* the master waits until the I2C bus is idle */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
            HAL_DEBUGW("i2c busy timeout");
            return HAL_ERR_TIMEOUT;
        }
    }
    /* check if I2C is already enabled */
    if(I2C_CTL0_I2CEN != (I2C_CTL0(i2c->periph) & I2C_CTL0_I2CEN)){
        i2c_enable(i2c->periph);
    }
    /* initialize transmit parameters */
    i2c->tx_state = I2C_STATE_BUSY;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    previous_state = i2c->previous_state;
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    i2c->tx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_master_transmit_interrupt;
    
    /* the master sends a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);

    /* enable the I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);

    return HAL_ERR_NONE;
}

/*!
    \brief      serial receive amounts of data in master mode by interrupt method, the function 
                is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be receive 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_BUSY, 
                details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_master_serial_receive_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func)
{
    __IO uint32_t previous_state = HAL_I2C_PREVIOUS_STATE_NONE;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    if(I2C_FIRST_TRANSFER == i2c->transfer_option){
        /* wait until I2C bus is idle */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_I2CBSY, RESET, I2C_BUSY_TIMEOUT)){
            HAL_DEBUGW("i2c busy timeout");
            return HAL_ERR_TIMEOUT;
        }
    }
    /* check if I2C is already enabled */
    if(I2C_CTL0_I2CEN != (I2C_CTL0(i2c->periph) & I2C_CTL0_I2CEN)){
        i2c_enable(i2c->periph);
    }
    /* initialize receive parameters */
    i2c->rx_state = I2C_STATE_BUSY;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    previous_state = i2c->previous_state;
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer;
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    i2c->rx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_master_receive_interrupt;
    if(2 == i2c->rxbuffer.length){
        /* send a NACK for the next data byte which will be received into the shift register */
        i2c_ackpos_config(i2c->periph, I2C_ACKPOS_NEXT);
    }
    
    /* the master sends a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);

    /* enable the I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      serial transmit amounts of data in slave mode by interrupt method, the function is 
                non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_slave_serial_transmit_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state whether it is in address listen mode */
    if(I2C_STATE_LISTEN != i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    /* check if I2C is already enabled */
    if(I2C_CTL0_I2CEN != (I2C_CTL0(i2c->periph) & I2C_CTL0_I2CEN)){
        i2c_enable(i2c->periph);
    }
    /* initialize transmit parameters */
    i2c->tx_state = I2C_STATE_BUSY_LISTEN;
    i2c->error_state = HAL_I2C_ERROR_NONE;
    i2c->txbuffer.buffer = (uint8_t *)p_buffer;
    i2c->txbuffer.length = length;
    i2c->txbuffer.pos = 0;
    i2c->tx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_slave_transmit_interrupt;

    /* enable I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    return HAL_ERR_NONE;
}

/*!
    \brief      serial receive amounts of data in slave mode by interrupt method, the function 
                is non-blocking
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be receive 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_slave_serial_receive_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == i2c) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [i2c] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }

#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state whether it is in address listen mode */
    if(I2C_STATE_LISTEN != i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    /* check if I2C is already enabled */
    if(I2C_CTL0_I2CEN != (I2C_CTL0(i2c->periph) & I2C_CTL0_I2CEN)){
        i2c_enable(i2c->periph);
    }
    /* initialize transmit parameters */
    i2c->rx_state = I2C_STATE_BUSY_LISTEN;
    i2c->last_error = HAL_I2C_ERROR_NONE;
    i2c->rxbuffer.buffer = (uint8_t *)p_buffer;
    i2c->rxbuffer.length = length;
    i2c->rxbuffer.pos = 0;
    i2c->rx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_slave_serial_receive_interrupt;

    /* enable I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_BUF);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    return HAL_ERR_NONE;
}

/*!
    \brief      enable address listen in slave mode by interrupt method
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_BUSY, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_address_listen_interrupt_enable(hal_i2c_dev_struct *i2c, hal_i2c_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == i2c){
        HAL_DEBUGE("parameter [i2c] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check the tx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->tx_state){
        HAL_DEBUGE("i2c tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    /* check the rx_state whether it is busy or not */
    if(I2C_STATE_BUSY == i2c->rx_state){
        HAL_DEBUGE("i2c rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    i2c->tx_state = I2C_STATE_LISTEN;
    i2c->rx_state = I2C_STATE_LISTEN;
    /* check if I2C is already enabled */
    if(I2C_CTL0_I2CEN != (I2C_CTL0(i2c->periph) & I2C_CTL0_I2CEN)){
        i2c_enable(i2c->periph);
    }
    /* enable acknowledge */
    i2c_ack_config(i2c->periph, I2C_ACK_ENABLE);
    
    i2c->rx_callback = (void *)p_user_func;
    i2c->i2c_irq.event_handle = _i2c_address_listen_interrupt;
    
    /* enable I2C interrupt */
    i2c_interrupt_enable(i2c->periph, I2C_INT_ERR);
    i2c_interrupt_enable(i2c->periph, I2C_INT_EV);
    return HAL_ERR_NONE;
}

/*!
    \brief      disable address listen in slave mode by interrupt method
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_BUSY, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2c_address_listen_interrupt_disable(hal_i2c_dev_struct *i2c)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == i2c){
        HAL_DEBUGE("parameter [i2c] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    if((I2C_STATE_LISTEN == i2c->tx_state) && (I2C_STATE_LISTEN == i2c->rx_state)){
        i2c->tx_state = I2C_STATE_READY;
        i2c->rx_state = I2C_STATE_READY;
        /* disable acknowledge */
        i2c_ack_config(i2c->periph, I2C_ACK_ENABLE);
        /* disable I2C interrupt */
        i2c_interrupt_disable(i2c->periph, I2C_INT_ERR);
        i2c_interrupt_disable(i2c->periph, I2C_INT_EV);
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      wait the flag status until timeout
    \param[in]  i2c_periph: I2Cx(x=0,1)
    \param[in]  flag: I2C flags, refer to i2c_flag_enum
    \param[in]  status: the status of I2C flag to wait 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h 
*/
static int32_t _i2c_wait_flag_timeout(uint32_t i2c_periph, i2c_flag_enum flag, \
                                      FlagStatus status, uint32_t timeout_ms)
{
    uint32_t tick_start;
    /* set timeout */
    tick_start = hal_basetick_count_get();
    /* wait flag status RESET */
    if(RESET == status){
        while(SET == i2c_flag_get(i2c_periph, flag)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("i2c get flag timeout");
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }else{
        /* wait flag status SET */
        while(RESET == i2c_flag_get(i2c_periph, flag)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("i2c get flag timeout");
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      master sends device address for write request
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h 
*/
static int32_t _i2c_master_write(hal_i2c_dev_struct *i2c, uint32_t timeout_ms)
{
    uint16_t first_byte, second_byte;
    uint32_t address_format;
    /* check the address format */
    if(I2C_SADDR0(i2c->periph)&I2C_SADDR0_ADDFORMAT){
        address_format = I2C_ADDFORMAT_10BITS;
    }else{
        address_format = I2C_ADDFORMAT_7BITS;
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);
    /* wait until SBSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_SBSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get SBSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    
    /* send slave address to I2C bus */
    if(I2C_ADDFORMAT_7BITS == address_format){
        /* 7-bit addressing */
        i2c_master_addressing(i2c->periph, i2c->slave_address.device_address, I2C_TRANSMITTER);
    }else{
        /* 10-bit addressing */
        first_byte = SLAVE10_FIRST_BYTE(i2c->slave_address.device_address);
        second_byte = SLAVE10_SECOND_BYTE(i2c->slave_address.device_address);
        
        i2c_master_addressing(i2c->periph, first_byte, I2C_TRANSMITTER);
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADD10SEND, SET, timeout_ms)){
            HAL_DEBUGW("i2c get ADD10SEND timeout");
            return HAL_ERR_TIMEOUT;
        }
        i2c_master_addressing(i2c->periph, second_byte, I2C_TRANSMITTER);
    }
    /* wait until ADDSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      event handler in I2C master transmit mode
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2c_master_transmit_interrupt(void *i2c)
{
    hal_i2c_dev_struct *p_i2c = i2c;
    uint16_t first_byte, second_byte;
    uint32_t address_format;
    hal_i2c_user_cb p_func = (hal_i2c_user_cb)p_i2c->tx_callback;
    uint32_t data_length;
    
    data_length = p_i2c->txbuffer.length;
    
    if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_SBSEND)){
        /* check the address format */
        if(I2C_SADDR0(p_i2c->periph)&I2C_SADDR0_ADDFORMAT){
            address_format = I2C_ADDFORMAT_10BITS;
        }else{
            address_format = I2C_ADDFORMAT_7BITS;
        }
        if(I2C_ADDFORMAT_10BITS == address_format){
            first_byte = SLAVE10_FIRST_BYTE(p_i2c->slave_address.device_address);
            /* send slave address first byte to I2C bus */
            i2c_master_addressing(p_i2c->periph, first_byte, I2C_TRANSMITTER);
        }else{
            /* send slave address */
            i2c_master_addressing(p_i2c->periph, p_i2c->slave_address.device_address, I2C_TRANSMITTER);
        }
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADD10SEND)){
        second_byte = SLAVE10_SECOND_BYTE(p_i2c->slave_address.device_address);
        /* send slave address second byte to I2C bus */
        i2c_master_addressing(p_i2c->periph, second_byte, I2C_TRANSMITTER);
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADDSEND)){
        /*clear ADDSEND bit */
        i2c_interrupt_flag_clear(p_i2c->periph, I2C_INT_FLAG_ADDSEND);
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_TBE)){
        if(data_length > p_i2c->txbuffer.pos){
            /* the master sends a data byte */
            i2c_data_transmit(p_i2c->periph, *p_i2c->txbuffer.buffer++);
            p_i2c->txbuffer.pos++;
        }else{
            if((I2C_NO_OPTION_TRANSFER != p_i2c->transfer_option) && \
                (I2C_LAST_TRANSFER != p_i2c->transfer_option)){
                /* disable the I2C interrupt */
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
                p_i2c->previous_state = HAL_I2C_PREVIOUS_STATE_TX;
            }else{
                /* the master sends a stop condition to I2C bus */
                i2c_stop_on_bus(p_i2c->periph);
                /* disable the I2C interrupt */
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
            }
            if(NULL != p_func){
                p_func(p_i2c);
            }
            p_i2c->tx_state = I2C_STATE_READY;
        }
    }
}

/*!
    \brief      DMA full finish handler in I2C master transmit mode
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     none
*/
static void _i2c_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2c_dev_struct *p_i2c;
    hal_i2c_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_i2c = (hal_i2c_dev_struct*)p_dma->p_periph;
    p_func = (hal_i2c_user_cb)p_i2c->tx_callback;
    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_i2c->txbuffer.pos = p_i2c->txbuffer.length;
        i2c_dma_enable(p_i2c->periph, I2C_DMA_OFF);
        p_i2c->tx_state = I2C_STATE_READY;
    }
    if(RESET != i2c_flag_get(p_i2c->periph, I2C_FLAG_MASTER)){
        i2c_stop_on_bus(p_i2c->periph);
        while(I2C_CTL0(p_i2c->periph)&0x0200);
    }else{
        if(RESET == i2c_flag_get(p_i2c->periph, I2C_FLAG_TR)){
            while(!i2c_flag_get(p_i2c->periph, I2C_FLAG_STPDET));
            /* clear the STPDET bit */
            i2c_enable(p_i2c->periph);
        }
    }

    if(NULL != p_func){
        p_func(p_i2c);
    }
}

/*!
    \brief      handle the I2C DMA error process
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     none
*/
static void _i2c_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2c_dev_struct *p_i2c;
    p_dma = (hal_dma_dev_struct*)dma;
    p_i2c = (hal_i2c_dev_struct*)p_dma->p_periph;

    if(I2C_STATE_BUSY == p_i2c->tx_state){
        /* transmit state is busy */
        p_i2c->error_state |= HAL_I2C_ERROR_DMATX;
        p_i2c->last_error = HAL_I2C_ERROR_DMATX;
        p_i2c->txbuffer.pos = p_i2c->txbuffer.length;
        /* disable DMA transmit and reset tx_state */
        i2c_dma_enable(p_i2c->periph, I2C_DMA_OFF);
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
        p_i2c->tx_state = I2C_STATE_READY;
    }else if(I2C_STATE_BUSY == p_i2c->rx_state){
        /* receive state is busy */
        p_i2c->error_state |= HAL_I2C_ERROR_DMATX;
        p_i2c->last_error = HAL_I2C_ERROR_DMATX;
        p_i2c->rxbuffer.pos = p_i2c->rxbuffer.length;
        /* disable DMA receive, PERR, ERR interrupt */
        i2c_dma_enable(p_i2c->periph, I2C_DMA_OFF);
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
        /* reset rx_state */
        p_i2c->rx_state = I2C_STATE_READY;
    }else{
        HAL_DEBUGE("i2c processor fatal error: dma error exception due to run state");
    }

    if(p_i2c->i2c_irq.error_handle != NULL){
        /* if there is a user error callback */
        p_i2c->i2c_irq.error_handle(p_i2c);
        p_i2c->error_state = HAL_I2C_ERROR_NONE;
    }
}

/*!
    \brief      master sends device address for read request
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h 
*/
static int32_t _i2c_master_read(hal_i2c_dev_struct *i2c, uint32_t timeout_ms)
{
    uint16_t first_byte, second_byte;
    uint32_t address_format;
    /* check the address format */
    if(I2C_SADDR0(i2c->periph)&I2C_SADDR0_ADDFORMAT){
        address_format = I2C_ADDFORMAT_10BITS;
    }else{
        address_format = I2C_ADDFORMAT_7BITS;
    }
    
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);
    /* wait until SBSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_SBSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get SBSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    
    /* send slave address to I2C bus */
    if(I2C_ADDFORMAT_7BITS == address_format){
        /* 7-bit addressing */
        i2c_master_addressing(i2c->periph, i2c->slave_address.device_address, I2C_RECEIVER);
    }else{
        /* 10-bit addressing */
        first_byte = SLAVE10_FIRST_BYTE(i2c->slave_address.device_address);
        second_byte = SLAVE10_SECOND_BYTE(i2c->slave_address.device_address);
        /* send slave address first byte to I2C bus */
        i2c_master_addressing(i2c->periph, first_byte, I2C_TRANSMITTER);
        /* wait until ADD10SEND bit is set */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADD10SEND, SET, timeout_ms)){
            HAL_DEBUGW("i2c get ADD10SEND timeout");
            return HAL_ERR_TIMEOUT;
        }
        /* send slave address 2nd byte to I2C bus */
        i2c_master_addressing(i2c->periph, second_byte, I2C_TRANSMITTER);
        /* wait until ADDSEND bit is set */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
            HAL_DEBUGW("i2c get ADDSEND timeout");
            return HAL_ERR_TIMEOUT;
        }
        /* clear ADDSEND bit */
        i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
        /* send a start condition to I2C bus */
        i2c_start_on_bus(i2c->periph);
        /* wait until SBSEND bit is set */
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_SBSEND, SET, timeout_ms)){
            HAL_DEBUGW("i2c get SBSEND timeout");
            return HAL_ERR_TIMEOUT;
        }
        /* send slave address first byte to I2C bus */
        i2c_master_addressing(i2c->periph, first_byte, I2C_RECEIVER);
    }
    /* wait until ADDSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      event handler in I2C master receive mode
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2c_master_receive_interrupt(void *i2c)
{
    hal_i2c_dev_struct *p_i2c = i2c;
    uint16_t first_byte, second_byte;
    hal_i2c_user_cb p_func = (hal_i2c_user_cb)p_i2c->rx_callback;
    uint32_t data_length;
    
    data_length = p_i2c->rxbuffer.length;

    if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_SBSEND)){
        if(!(I2C_SADDR0(p_i2c->periph)&I2C_SADDR0_ADDFORMAT)){
            /* the master sends slave address */
            i2c_master_addressing(p_i2c->periph, p_i2c->slave_address.device_address, I2C_RECEIVER);
        }else{
            first_byte = SLAVE10_FIRST_BYTE(p_i2c->slave_address.device_address);
            if(RESET == p_i2c->slave_address.second_addressing){
                i2c_master_addressing(p_i2c->periph, first_byte, I2C_TRANSMITTER);
                p_i2c->slave_address.second_addressing = SET;
            }else{
                i2c_master_addressing(p_i2c->periph, first_byte, I2C_RECEIVER);
            }
        }
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADD10SEND)){
        second_byte = SLAVE10_SECOND_BYTE(p_i2c->slave_address.device_address);
        i2c_master_addressing(p_i2c->periph, second_byte, I2C_TRANSMITTER);
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADDSEND)){
        
        if((1 == p_i2c->rxbuffer.length)||(2 == p_i2c->rxbuffer.length)){
            i2c_ack_config(p_i2c->periph, I2C_ACK_DISABLE);
        }
        if(!(I2C_SADDR0(p_i2c->periph)&I2C_SADDR0_ADDFORMAT)){
            /* clear the ADDSEND bit */
            i2c_interrupt_flag_clear(p_i2c->periph, I2C_INT_FLAG_ADDSEND);
        }else{
            if(0 == p_i2c->slave_address.address_count){
                /* clear the ADDSEND bit */
                i2c_interrupt_flag_clear(p_i2c->periph,I2C_INT_FLAG_ADDSEND);
                p_i2c->slave_address.address_count = 1;
                /* send a start condition to I2C bus */
                i2c_start_on_bus(p_i2c->periph);
            }else{
                /* clear the ADDSEND bit */
                i2c_interrupt_flag_clear(p_i2c->periph,I2C_INT_FLAG_ADDSEND);
            }
        }
    }
    else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_RBNE)){
        if(data_length > p_i2c->rxbuffer.pos){
            if(3 == data_length - p_i2c->rxbuffer.pos){
                /* wait until the second last data byte is received into the shift register */
                while(!i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_BTC));
                /* send a NACK for the last data byte */
                i2c_ack_config(p_i2c->periph, I2C_ACK_DISABLE);
            }
            /* read a data byte from I2C_DATA*/
            *p_i2c->rxbuffer.buffer++ = i2c_data_receive(p_i2c->periph);
            p_i2c->rxbuffer.pos++;
            if(0 == data_length - p_i2c->rxbuffer.pos){
                if((I2C_NO_OPTION_TRANSFER != p_i2c->transfer_option) && \
                    (I2C_LAST_TRANSFER != p_i2c->transfer_option)){
                    p_i2c->previous_state = HAL_I2C_PREVIOUS_STATE_TX;
                }else{
                    /* send a stop condition */
                    i2c_stop_on_bus(p_i2c->periph);
                }
                
                i2c_ack_config(p_i2c->periph, I2C_ACK_ENABLE);
                i2c_ackpos_config(p_i2c->periph, I2C_ACKPOS_CURRENT);
                /* disable the I2C interrupt */
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
                
                if(NULL != p_func){
                    p_func(p_i2c);
                }
                p_i2c->rx_state =   I2C_STATE_READY;
            }
        }
    }
}

/*!
    \brief      DMA full finish handler in I2C master receive mode
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     none
*/
static void _i2c_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2c_dev_struct *p_i2c;
    hal_i2c_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_i2c = (hal_i2c_dev_struct*)p_dma->p_periph;
    p_func = (hal_i2c_user_cb)p_i2c->rx_callback;
    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_i2c->rxbuffer.pos = p_i2c->rxbuffer.length;
        i2c_dma_enable(p_i2c->periph, I2C_DMA_OFF);
        p_i2c->rx_state = I2C_STATE_READY;
    }
    if(RESET != i2c_flag_get(p_i2c->periph, I2C_FLAG_MASTER)){
        i2c_stop_on_bus(p_i2c->periph);
        while(I2C_CTL0(p_i2c->periph)&0x0200);
    }else{
        while(!i2c_flag_get(p_i2c->periph, I2C_FLAG_STPDET));
        /* clear the STPDET bit */
        i2c_enable(p_i2c->periph);
    }
    if(NULL != p_func){
        p_func(p_i2c);
    }
}

/*!
    \brief      event handler in I2C slave transmit mode
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2c_slave_transmit_interrupt(void *i2c)
{
    hal_i2c_dev_struct *p_i2c = i2c;
    hal_i2c_user_cb p_func = (hal_i2c_user_cb)p_i2c->tx_callback;
    
    uint32_t data_length;
    
    data_length = p_i2c->txbuffer.length;

    if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADDSEND)){
        /* clear the ADDSEND bit */
        i2c_interrupt_flag_clear(p_i2c->periph, I2C_INT_FLAG_ADDSEND);
    }else if((i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_TBE))&&(!i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_AERR))){
        /* send a data byte */
        i2c_data_transmit(p_i2c->periph, *p_i2c->txbuffer.buffer++);
        p_i2c->txbuffer.pos++;
        if (p_i2c->txbuffer.pos == data_length){
            if(NULL != p_func){
                p_func(p_i2c);
            }
            
            /* disable I2C interrupt */
            i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
            i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
            i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
            
            if (I2C_STATE_BUSY_LISTEN == p_i2c->tx_state){
                p_i2c->tx_state = I2C_STATE_LISTEN;
            }
        }
    }
}

/*!
    \brief      event handler in I2C slave receive mode
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2c_slave_receive_interrupt(void *i2c)
{
    hal_i2c_dev_struct *p_i2c = i2c;
    hal_i2c_user_cb p_func = (hal_i2c_user_cb)p_i2c->rx_callback;
    uint32_t data_length;
    
    data_length = p_i2c->rxbuffer.length;
    
    if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADDSEND)){
        /* clear the ADDSEND bit */
        i2c_interrupt_flag_clear(p_i2c->periph, I2C_INT_FLAG_ADDSEND);
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_RBNE)){
        /* if reception data register is not empty ,I2C will read a data from I2C_DATA */
        *p_i2c->rxbuffer.buffer++ = i2c_data_receive(p_i2c->periph);
        p_i2c->rxbuffer.pos++;
        if((p_i2c->rxbuffer.pos == data_length) && (I2C_STATE_BUSY_LISTEN == p_i2c->rx_state)){
            p_i2c->rx_state = I2C_STATE_LISTEN;
            i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
            if(NULL != p_func){
                p_func(p_i2c);
            }
        }
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_STPDET)){
        /* clear the STPDET bit */
        i2c_enable(p_i2c->periph);
        /* disable I2C interrupt */
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
        i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
        if(NULL != p_func){
            p_func(p_i2c);
        }
    }
}

/*!
    \brief      event handler in I2C slave serial receive mode
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2c_slave_serial_receive_interrupt(void *i2c)
{
    hal_i2c_dev_struct *p_i2c = i2c;
    hal_i2c_user_cb p_func = (hal_i2c_user_cb)p_i2c->rx_callback;
    uint32_t data_length;
    
    data_length = p_i2c->rxbuffer.length;
    
    if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADDSEND)){
        /* clear the ADDSEND bit */
        i2c_interrupt_flag_clear(p_i2c->periph, I2C_INT_FLAG_ADDSEND);
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_RBNE)){
        /* if reception data register is not empty ,I2C will read a data from I2C_DATA */
        *p_i2c->rxbuffer.buffer++ = i2c_data_receive(p_i2c->periph);
        p_i2c->rxbuffer.pos++;
        
        if((p_i2c->rxbuffer.pos == data_length) && (I2C_STATE_BUSY_LISTEN == p_i2c->rx_state)){
            p_i2c->rx_state = I2C_STATE_LISTEN;
            if(NULL != p_func){
                p_func(p_i2c);
            }
            
            /* disable I2C interrupt */
            i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
            i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
            i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
            if(NULL != p_func){
                p_func(p_i2c);
            }
        }
    }
}

/*!
    \brief      master sends device address for memory write request
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h 
*/
static int32_t _i2c_memory_write(hal_i2c_dev_struct *i2c, uint32_t timeout_ms)
{
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);
    /* wait until SBSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_SBSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get SBSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c->periph, i2c->slave_address.device_address, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    /* wait until the transmission data register is empty*/
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_TBE, SET, timeout_ms)){
        HAL_DEBUGW("i2c get TBE timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* send memory address to write to */
    if(I2C_MEMORY_ADDRESS_8BIT == i2c->slave_address.address_size){
        /* one byte address */
        i2c_data_transmit(i2c->periph, I2C_MEMORY_ADDRESS_LSB(i2c->slave_address.memory_address));
    }else{
        /* two bytes address */
        i2c_data_transmit(i2c->periph, I2C_MEMORY_ADDRESS_MSB(i2c->slave_address.memory_address));
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_TBE, SET, timeout_ms)){
            HAL_DEBUGW("i2c get TBE timeout");
            return HAL_ERR_TIMEOUT;
        }
        i2c_data_transmit(i2c->periph, I2C_MEMORY_ADDRESS_LSB(i2c->slave_address.memory_address));
    }
    /* wait for BTC set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_BTC, SET, timeout_ms)){
        HAL_DEBUGW("i2c get BTC timeout");
        return HAL_ERR_TIMEOUT;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      master sends device address for memory read request
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h 
*/
static int32_t _i2c_memory_read(hal_i2c_dev_struct *i2c, uint32_t timeout_ms)
{
    /* enable acknowledge */
    i2c_ack_config(i2c->periph, I2C_ACK_ENABLE);
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);
    /* wait until SBSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_SBSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get SBSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c->periph, i2c->slave_address.device_address, I2C_TRANSMITTER);
    /* wait until ADDSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* clear ADDSEND bit */
    i2c_flag_clear(i2c->periph, I2C_FLAG_ADDSEND);
    /* wait until the TBE bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_TBE, SET, timeout_ms)){
        HAL_DEBUGW("i2c get TBE timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* send memory address to I2C bus */
    if(I2C_MEMORY_ADDRESS_8BIT == i2c->slave_address.address_size){
        i2c_data_transmit(i2c->periph, I2C_MEMORY_ADDRESS_LSB(i2c->slave_address.memory_address));
    }else{
        i2c_data_transmit(i2c->periph, I2C_MEMORY_ADDRESS_MSB(i2c->slave_address.memory_address));
        if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_TBE, SET, timeout_ms)){
            HAL_DEBUGW("i2c get TBE timeout");
            return HAL_ERR_TIMEOUT;
        }
        i2c_data_transmit(i2c->periph, I2C_MEMORY_ADDRESS_LSB(i2c->slave_address.memory_address));
    }
    /* wait until the BTC bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_BTC, SET, timeout_ms)){
        HAL_DEBUGW("i2c get BTC timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* send a start condition to I2C bus */
    i2c_start_on_bus(i2c->periph);
    /* wait until SBSEND bit is set */
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_SBSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get SBSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    /* send slave address to I2C bus */
    i2c_master_addressing(i2c->periph, i2c->slave_address.device_address, I2C_RECEIVER);
    if(HAL_ERR_NONE != _i2c_wait_flag_timeout(i2c->periph, I2C_FLAG_ADDSEND, SET, timeout_ms)){
        HAL_DEBUGW("i2c get ADDSEND timeout");
        return HAL_ERR_TIMEOUT;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      event handler of memory write in I2C master mode
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2c_memmory_write_interrupt(void *i2c)
{
    hal_i2c_dev_struct *p_i2c = i2c;
    hal_i2c_user_cb p_func = (hal_i2c_user_cb)p_i2c->tx_callback;
    uint32_t data_length;
    
    data_length = p_i2c->txbuffer.length;

    if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_SBSEND)){
        /* send slave address */
        i2c_master_addressing(p_i2c->periph, p_i2c->slave_address.device_address, I2C_TRANSMITTER);
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADDSEND)){
        /*clear ADDSEND bit */
        i2c_interrupt_flag_clear(p_i2c->periph, I2C_INT_FLAG_ADDSEND);
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_TBE)){
        if(RESET == p_i2c->slave_address.address_complete){
            if(I2C_MEMORY_ADDRESS_8BIT == p_i2c->slave_address.address_size){
                i2c_data_transmit(p_i2c->periph, I2C_MEMORY_ADDRESS_LSB(p_i2c->slave_address.memory_address));
                p_i2c->slave_address.address_complete = SET;
            }else{
                if(0 == p_i2c->slave_address.address_count){
                    i2c_data_transmit(p_i2c->periph, I2C_MEMORY_ADDRESS_MSB(p_i2c->slave_address.memory_address));
                    p_i2c->slave_address.address_count++;
                }else{
                    i2c_data_transmit(p_i2c->periph, I2C_MEMORY_ADDRESS_LSB(p_i2c->slave_address.memory_address));
                }
            }
            p_i2c->slave_address.address_complete = SET;
        }else{
           if(data_length > p_i2c->txbuffer.pos){
                /* the master sends a data byte */
                i2c_data_transmit(p_i2c->periph, *p_i2c->txbuffer.buffer++);
                p_i2c->txbuffer.pos++;
               
            }else{
                /* the master sends a stop condition to I2C bus */
                i2c_stop_on_bus(p_i2c->periph);
                /* disable the I2C interrupt */
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
                if(NULL != p_func){
                    p_func(p_i2c);
                }
                p_i2c->tx_state = I2C_STATE_READY;
            }
        }
    }
}

/*!
    \brief      event handler of memory read in I2C master mode
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2c_memory_read_interrupt(void *i2c)
{
    hal_i2c_dev_struct *p_i2c = i2c;
    hal_i2c_user_cb p_func = (hal_i2c_user_cb)p_i2c->rx_callback;
    uint32_t data_length;
    
    data_length = p_i2c->rxbuffer.length;
    
    if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_SBSEND)){
        if(RESET == p_i2c->slave_address.second_addressing){
            i2c_master_addressing(p_i2c->periph, p_i2c->slave_address.device_address, I2C_TRANSMITTER);
            
        }else{
            /* the master sends slave address */
            i2c_master_addressing(p_i2c->periph, p_i2c->slave_address.device_address, I2C_RECEIVER);
        }
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADDSEND)){
        if(RESET == p_i2c->slave_address.second_addressing){
            /* clear the ADDSEND bit */
            i2c_interrupt_flag_clear(p_i2c->periph,I2C_INT_FLAG_ADDSEND);
        }else{
            if((1 == data_length - p_i2c->rxbuffer.pos)|| \
                (2 == data_length - p_i2c->rxbuffer.pos)){
                /* clear the ACKEN before the ADDSEND is cleared */
                i2c_ack_config(p_i2c->periph, I2C_ACK_DISABLE);
                /* clear the ADDSEND bit */
                i2c_interrupt_flag_clear(p_i2c->periph,I2C_INT_FLAG_ADDSEND);
            }else{
                /* clear the ADDSEND bit */
                i2c_interrupt_flag_clear(p_i2c->periph,I2C_INT_FLAG_ADDSEND);
            }
        }
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_RBNE)){
        if(data_length > p_i2c->rxbuffer.pos){
            if(3 == data_length - p_i2c->rxbuffer.pos){
                /* wait until the second last data byte is received into the shift register */
                while(!i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_BTC));
                /* send a NACK for the last data byte */
                i2c_ack_config(p_i2c->periph, I2C_ACK_DISABLE);
            }
            /* read a data byte from I2C_DATA*/
            *p_i2c->rxbuffer.buffer++ = i2c_data_receive(p_i2c->periph);
            p_i2c->rxbuffer.pos++;
            if(0 == data_length - p_i2c->rxbuffer.pos){
                /* send a stop condition */
                i2c_stop_on_bus(p_i2c->periph);
                i2c_ack_config(p_i2c->periph, I2C_ACK_ENABLE);
                i2c_ackpos_config(p_i2c->periph, I2C_ACKPOS_CURRENT);
                /* disable the I2C interrupt */
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_BUF);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);
                i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
                if(NULL != p_func){
                    p_func(p_i2c);
                }
            }
        }
    }else if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_TBE)){
            if(I2C_MEMORY_ADDRESS_8BIT == p_i2c->slave_address.address_size){
                i2c_data_transmit(p_i2c->periph, I2C_MEMORY_ADDRESS_LSB(p_i2c->slave_address.memory_address));
                p_i2c->slave_address.second_addressing = SET;
            }else{
                if(0 == p_i2c->slave_address.address_count){
                    i2c_data_transmit(p_i2c->periph, I2C_MEMORY_ADDRESS_MSB(p_i2c->slave_address.memory_address));
                    p_i2c->slave_address.address_count++;
                }else{
                    i2c_data_transmit(p_i2c->periph, I2C_MEMORY_ADDRESS_LSB(p_i2c->slave_address.memory_address));
                }
            }
            /* wait until BTC bit is set */
            while(!i2c_flag_get(p_i2c->periph, I2C_FLAG_BTC));
            /* send a start condition to I2C bus */
            i2c_start_on_bus(p_i2c->periph);
    }
}

/*!
    \brief      event handler for address listen in slave mode
    \param[in]  i2c: I2C device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2c_address_listen_interrupt(void *i2c)
{
    hal_i2c_dev_struct *p_i2c = i2c;
    hal_i2c_user_cb p_func = (hal_i2c_user_cb)p_i2c->rx_callback;
    
    i2c_interrupt_disable(p_i2c->periph, I2C_INT_ERR);
    i2c_interrupt_disable(p_i2c->periph, I2C_INT_EV);

//    if(i2c_interrupt_flag_get(p_i2c->periph, I2C_INT_FLAG_ADDSEND)){
//        i2c_interrupt_flag_clear(p_i2c->periph, I2C_INT_FLAG_ADDSEND);
        /* check transfer direction in p_func, so that the user knows 
        which function to call next(hal_i2c_slave_serial_transmit_interrupt
        or hal_i2c_slave_serial_receive_interrupt) */
        if(NULL != p_func){
            p_func(p_i2c);
        }
//    }
}
