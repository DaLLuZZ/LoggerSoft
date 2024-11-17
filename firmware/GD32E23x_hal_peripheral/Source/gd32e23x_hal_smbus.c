/*!
    \file    gd32e23x_hal_smbus.c
    \brief   SMBUS driver
    
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

/* wait the flag status until timeout */
static int32_t _smbus_wait_flag_timeout(uint32_t smbus_periph, i2c_flag_enum flag, \
                                      FlagStatus status, uint32_t timeout_ms);
/* event handler in SMBUS master transmit mode */
static void _smbus_master_transmit_interrupt(void *smbus);
/* event handler in SMBUS slave transmit mode */
static void _smbus_slave_transmit_interrupt(void *smbus);
/* event handler in SMBUS master receive mode */
static void _smbus_master_receive_interrupt(void *smbus);
/* event handler in SMBUS slave receive mode */
static void _smbus_slave_receive_interrupt(void *smbus);

/*!
    \brief      initialize the SMBUS structure with the default values
    \param[in]  struct_type: refer to hal_smbus_struct_type_enum
    \param[in]  p_struct: point to SMBUS structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_smbus_struct_init(hal_smbus_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(hal_struct_type){
    case HAL_SMBUS_INIT_STRUCT:
        /* initialize SMBUS initialization structure with the default values */
        ((hal_smbus_init_struct*)p_struct)->own_address1 = 0;
        ((hal_smbus_init_struct*)p_struct)->own_address2 = 0;
        ((hal_smbus_init_struct*)p_struct)->dual_address = SMBUS_DUADEN_DISABLE;
        ((hal_smbus_init_struct*)p_struct)->clock_speed = 100000;
        ((hal_smbus_init_struct*)p_struct)->address_format = SMBUS_ADDFORMAT_7BITS;
        ((hal_smbus_init_struct*)p_struct)->smbus_type = SMBUS_DEVICE;
        ((hal_smbus_init_struct*)p_struct)->smbus_pec = SMBUS_PEC_DISABLE;
        ((hal_smbus_init_struct*)p_struct)->smbus_pec_transfer = SMBUS_PECTRANS_DISABLE;
        ((hal_smbus_init_struct*)p_struct)->smbus_arp = SMBUS_ARP_DISABLE;
        ((hal_smbus_init_struct*)p_struct)->general_call = SMBUS_GCEN_DISABLE;
        ((hal_smbus_init_struct*)p_struct)->no_stretch = SMBUS_SCLSTRETCH_DISABLE;
    
        break;
    
    case HAL_SMBUS_DEV_STRUCT:
        /* initialize SMBUS device information structure with the default values */
        ((hal_smbus_dev_struct*)p_struct)->periph = 0;
        ((hal_smbus_dev_struct*)p_struct)->smbus_irq.event_handle = NULL;
        ((hal_smbus_dev_struct*)p_struct)->smbus_irq.error_handle = NULL;
        ((hal_smbus_dev_struct*)p_struct)->txbuffer.buffer = NULL;
        ((hal_smbus_dev_struct*)p_struct)->txbuffer.length = 0;
        ((hal_smbus_dev_struct*)p_struct)->txbuffer.pos = 0;
        ((hal_smbus_dev_struct*)p_struct)->rxbuffer.buffer = NULL;
        ((hal_smbus_dev_struct*)p_struct)->rxbuffer.length = 0;
        ((hal_smbus_dev_struct*)p_struct)->rxbuffer.pos = 0;
        ((hal_smbus_dev_struct*)p_struct)->error_state = HAL_SMBUS_ERROR_NONE;
        ((hal_smbus_dev_struct*)p_struct)->tx_state = SMBUS_STATE_FREE;
        ((hal_smbus_dev_struct*)p_struct)->rx_state = SMBUS_STATE_FREE;
        ((hal_smbus_dev_struct*)p_struct)->last_error = HAL_SMBUS_ERROR_NONE;
        ((hal_smbus_dev_struct*)p_struct)->priv = NULL;      /* priv data */
    
        break;
    
    case HAL_SMBUS_IRQ_STRUCT:
        /* initialize SMBS irq structure with the default values */
        ((hal_smbus_irq_struct*)p_struct)->error_handle = NULL;
        ((hal_smbus_irq_struct*)p_struct)->event_handle = NULL;
    
        break;
        
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize SMBUS
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smbus_deinit(hal_smbus_dev_struct *smbus)
{
    uint32_t periph;
    
    periph = smbus->periph;
    if((I2C0 == periph) || (I2C1 == periph)){
        i2c_deinit(periph);
        hal_smbus_struct_init(HAL_SMBUS_DEV_STRUCT, smbus);
        smbus->periph = periph;
    }else{
        HAL_DEBUGE("parameter [smbus->periph] value is invalid");
    }
}

/*!
    \brief      initialize SMBUS registers
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which SMBUS is initialized
    \param[in]  p_init: the initialization data needed to initialize SMBUS
                  own_address1: SMBUS address
                  own_address2: the second address in dual-address mode
                  clock_speed: SMBUS clock speed, supports standard mode (up to 100 kHz)
                  address_format: SMBUS_ADDFORMAT_7BITS, SMBUS_ADDFORMAT_10BITS
                  smbus_type: SMBUS_DEVICE, SMBUS_HOST
                  dual_address: SMBUS_DUADEN_ENABLE, SMBUS_DUADEN_DISABLE
                  smbus_pec: SMBUS_PEC_ENABLE, SMBUS_PEC_DISABLE
                  smbus_pec_transfer: SMBUS_PECTRANS_ENABLE, SMBUS_PECTRANS_DISABLE
                  smbus_arp: SMBUS_ARP_ENABLE, SMBUS_ARP_DISABLE
                  general_call: SMBUS_GCEN_ENABLE, SMBUS_GCEN_DISABLE
                  no_stretch: SMBUS_SCLSTRETCH_ENABLE, SMBUS_SCLSTRETCH_DISABLE
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_smbus_init(hal_smbus_dev_struct *smbus, uint32_t periph, \
                      hal_smbus_init_struct *p_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == smbus) && (NULL == p_init)){
        HAL_DEBUGE("pointer [smbus] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    if((I2C0 != periph) && (I2C1 != periph)){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }
    
    if(0 != smbus->periph){
        HAL_DEBUGI("periph value from smbus device struct has been rewrite");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    smbus->periph = periph;
    
    /* disable peripheral */
    i2c_disable(periph);
    
    i2c_clock_config(periph,p_init->clock_speed,I2C_DTCY_2);
    
    i2c_mode_addr_config(periph, I2C_SMBUSMODE_ENABLE, p_init->address_format, p_init->own_address1);
    
    /* configure dual-address mode */
    if (SMBUS_DUADEN_ENABLE == p_init->dual_address){
        i2c_dualaddr_enable(periph, p_init->own_address2);
    } else {
        i2c_dualaddr_disable(periph);
    }
    /* configure whether to stretch SCL low when data is not ready in slave mode */
    i2c_stretch_scl_low_config(periph, p_init->no_stretch);
    /* whether or not to response to a general call */
    i2c_slave_response_to_gcall_config(periph, p_init->general_call);
    
    i2c_smbus_type_config(periph, p_init->smbus_type);
    
    i2c_pec_enable(periph, p_init->smbus_pec);
    
    i2c_smbus_arp_enable(periph, p_init->smbus_arp);
        
    /* enable peripheral */
    i2c_enable(periph);
    /* enable acknowledge */
    i2c_ack_config(periph, SMBUS_ACK_ENABLE);
    
    smbus->tx_state = SMBUS_STATE_READY;
    smbus->rx_state = SMBUS_STATE_READY;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      SMBUS evevt interrupt handler
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smbus_event_irq(hal_smbus_dev_struct *smbus)
{
    if(smbus->smbus_irq.event_handle != NULL){
        smbus->smbus_irq.event_handle(smbus);
    }
}

/*!
    \brief      SMBUS error interrupt handler
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smbus_error_irq(hal_smbus_dev_struct *smbus)
{
    /* PEC error */
    if(i2c_interrupt_flag_get(smbus->periph, I2C_INT_FLAG_PECERR) != RESET){
        i2c_interrupt_flag_clear(smbus->periph, I2C_INT_FLAG_PECERR);
        smbus->error_state |= HAL_SMBUS_ERROR_PECERR;
        smbus->last_error = HAL_SMBUS_ERROR_PECERR;
    }

    /* BERR error */
    if(i2c_interrupt_flag_get(smbus->periph, I2C_INT_FLAG_BERR) != RESET){
        i2c_interrupt_flag_clear(smbus->periph, I2C_INT_FLAG_BERR);
        smbus->error_state |= HAL_SMBUS_ERROR_BERR;
        smbus->last_error = HAL_SMBUS_ERROR_BERR;
    }

    /* LOSTARB error */
    if(i2c_interrupt_flag_get(smbus->periph, I2C_INT_FLAG_LOSTARB) != RESET){
        i2c_interrupt_flag_clear(smbus->periph, I2C_INT_FLAG_LOSTARB);
        smbus->error_state |= HAL_SMBUS_ERROR_LOSTARB;
        smbus->last_error = HAL_SMBUS_ERROR_LOSTARB;
    }

    /* AERR error */
    if(i2c_interrupt_flag_get(smbus->periph, I2C_INT_FLAG_AERR) != RESET){ 
        i2c_interrupt_flag_clear(smbus->periph, I2C_INT_FLAG_AERR);
        smbus->error_state |= HAL_SMBUS_ERROR_AERR;
        smbus->last_error = HAL_SMBUS_ERROR_AERR;
    }
    
    /* OUERR error */
    if(i2c_interrupt_flag_get(smbus->periph, I2C_INT_FLAG_OUERR) != RESET){ 
        i2c_interrupt_flag_clear(smbus->periph, I2C_INT_FLAG_OUERR);
        smbus->error_state |= HAL_SMBUS_ERROR_OUERR;
        smbus->last_error = HAL_SMBUS_ERROR_OUERR;
    }
    
    /* SMBUS timeout */
    if(i2c_interrupt_flag_get(smbus->periph, I2C_INT_FLAG_SMBTO) != RESET){ 
        i2c_interrupt_flag_clear(smbus->periph, I2C_INT_FLAG_SMBTO);
        smbus->error_state |= HAL_SMBUS_ERROR_SMBTO;
        smbus->last_error = HAL_SMBUS_ERROR_SMBTO;
    }
    
    /* SMBUS alert */
    if(i2c_interrupt_flag_get(smbus->periph, I2C_INT_FLAG_SMBALT) != RESET){ 
        i2c_interrupt_flag_clear(smbus->periph, I2C_INT_FLAG_SMBALT);
        smbus->error_state |= HAL_SMBUS_ERROR_SMBALT;
        smbus->last_error = HAL_SMBUS_ERROR_SMBALT;
    }
    
    /* SMBUS no error */
    if(smbus->error_state != HAL_SMBUS_ERROR_NONE){
        if(smbus->smbus_irq.error_handle != NULL){
            smbus->smbus_irq.error_handle(smbus);
            smbus->error_state = HAL_SMBUS_ERROR_NONE;
        }
    }
}

/*!
    \brief      start SMBUS module function
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     
*/
void hal_smbus_start(hal_smbus_dev_struct *smbus)
{
    i2c_enable(smbus->periph);
}

/*!
    \brief      stop SMBUS module function
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     
*/
void hal_smbus_stop(hal_smbus_dev_struct *smbus)
{
    i2c_disable(smbus->periph);
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: the structure that contains callback handlers of SMBUS interrupt
    \param[out] none
    \retval     none
*/
void hal_smbus_irq_handle_set(hal_smbus_dev_struct *smbus, hal_smbus_irq_struct *p_irq)
{
    /* event handle */
    if(NULL != p_irq->event_handle){
        smbus->smbus_irq.event_handle = p_irq->event_handle;
        i2c_interrupt_enable(smbus->periph, I2C_INT_EV);
        i2c_interrupt_enable(smbus->periph, I2C_INT_BUF);
    }else{
        smbus->smbus_irq.event_handle = NULL;
        i2c_interrupt_disable(smbus->periph, I2C_INT_EV);
        i2c_interrupt_disable(smbus->periph, I2C_INT_BUF);
    }
    
    /* error handle */
    if(NULL != p_irq->error_handle){
        smbus->smbus_irq.error_handle = p_irq->error_handle;
        i2c_interrupt_enable(smbus->periph, I2C_INT_ERR);
    }else{
        smbus->smbus_irq.error_handle = NULL;
        i2c_interrupt_disable(smbus->periph, I2C_INT_ERR);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_smbus_irq_handle_all_reset(hal_smbus_dev_struct *smbus)
{
    smbus->smbus_irq.event_handle = NULL;
    smbus->smbus_irq.error_handle = NULL;
}

/*!
    \brief      SMBUS master transmit in interrupt mode
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_smbus_master_transmit_interrupt(hal_smbus_dev_struct *smbus, uint8_t *p_buffer, \
                                     uint32_t length, hal_smbus_user_cb p_user_func)
{
    __IO uint32_t timeout_ms = HAL_TIMEOUT_FOREVER;
    
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == smbus) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [smbus] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* SMBUS state is busy */
    if(SMBUS_STATE_BUSY == smbus->tx_state){
        HAL_DEBUGE("smbus tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* update the value of SMBUS dev struct */
    smbus->error_state = HAL_SMBUS_ERROR_NONE;
    smbus->txbuffer.buffer = (uint8_t *)p_buffer; 
    smbus->txbuffer.length = length;
    smbus->txbuffer.pos = 0;

    /* SMBUS state is ready */
    if(SMBUS_STATE_READY == smbus->tx_state){
        smbus->tx_state = SMBUS_STATE_BUSY;
        smbus->tx_callback = (void*)p_user_func;
        smbus->smbus_irq.event_handle = _smbus_master_transmit_interrupt;
        
        /* enable the I2C interrupt */
        i2c_interrupt_enable(smbus->periph, I2C_INT_BUF);
        i2c_interrupt_enable(smbus->periph, I2C_INT_EV);
        
        /* the master waits until the I2C bus is idle */
        if(HAL_ERR_NONE != _smbus_wait_flag_timeout(smbus->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
            HAL_DEBUGW("i2c busy timeout");
            return HAL_ERR_TIMEOUT;
        }
        /* the master sends a start condition to I2C bus */
        i2c_start_on_bus(smbus->periph);
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      SMBUS master receive with interrupt
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be read 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_smbus_master_receive_interrupt(hal_smbus_dev_struct *smbus, uint8_t *p_buffer, \
                                    uint32_t length, hal_smbus_user_cb p_user_func)
{
    __IO uint32_t timeout_ms = HAL_TIMEOUT_FOREVER;
    
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == smbus) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [smbus] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* SMBUS state is busy */
    if(SMBUS_STATE_BUSY == smbus->rx_state){
        HAL_DEBUGE("smbus tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* update the values of SMBUS dev struct */
    smbus->error_state = HAL_SMBUS_ERROR_NONE;
    smbus->rxbuffer.buffer = (uint8_t *)p_buffer; 
    smbus->rxbuffer.length = length;
    smbus->rxbuffer.pos = 0;

    /* SMBUS state is ready */
    if(SMBUS_STATE_READY == smbus->rx_state){
        /* wait until I2C bus is idle */
        if(HAL_ERR_NONE != _smbus_wait_flag_timeout(smbus->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
            HAL_DEBUGW("i2c busy timeout");
            return HAL_ERR_TIMEOUT;
        }
        
        /* update the values of SMBUS dev struct */
        smbus->rx_state = SMBUS_STATE_BUSY;
        smbus->rx_callback = (void*)p_user_func;
        smbus->smbus_irq.event_handle = _smbus_master_receive_interrupt;
        
        /* enable the I2C interrupt */
        i2c_interrupt_enable(smbus->periph, I2C_INT_BUF);
        i2c_interrupt_enable(smbus->periph, I2C_INT_EV);
        
        /* wait until I2C bus is idle */
        if(HAL_ERR_NONE != _smbus_wait_flag_timeout(smbus->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
            HAL_DEBUGW("i2c busy timeout");
            return HAL_ERR_TIMEOUT;
        }
        
        if(smbus->rxbuffer.length == 2){
            /* send a NACK for the next data byte which will be received into the shift register */
            i2c_ackpos_config(I2C0, I2C_ACKPOS_NEXT);
        }

        /* the master sends a start condition to I2C bus */
        i2c_start_on_bus(smbus->periph);
        
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      SMBUS slave transmit in interrupt mode
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_smbus_slave_transmit_interrupt(hal_smbus_dev_struct *smbus, uint8_t *p_buffer, \
                                     uint32_t length, hal_smbus_user_cb p_user_func)
{
    __IO uint32_t timeout_ms = HAL_TIMEOUT_FOREVER;
    
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == smbus) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [smbus] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* SMBUS state is busy */
    if(SMBUS_STATE_BUSY == smbus->tx_state){
        HAL_DEBUGE("smbus tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* update the values of SMBUS dev struct */
    smbus->error_state = HAL_SMBUS_ERROR_NONE;
    smbus->txbuffer.buffer = (uint8_t *)p_buffer; 
    smbus->txbuffer.length = length;
    smbus->txbuffer.pos = 0;

    /* SMBUS state is ready */
    if(SMBUS_STATE_READY == smbus->tx_state){
        if(HAL_ERR_NONE != _smbus_wait_flag_timeout(smbus->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
            HAL_DEBUGW("i2c busy timeout");
            return HAL_ERR_TIMEOUT;
        }
        
        /* update the values of SMBUS dev struct */
        smbus->tx_state = SMBUS_STATE_BUSY;
        smbus->tx_callback = (void*)p_user_func;
        smbus->smbus_irq.event_handle = _smbus_slave_transmit_interrupt;
        
        /* enable the I2C interrupt */
        i2c_interrupt_enable(smbus->periph, I2C_INT_BUF);
        i2c_interrupt_enable(smbus->periph, I2C_INT_EV);
        
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      SMBUS slave receive with interrupt
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be read 
    \param[in]  p_user_func: call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_smbus_slave_receive_interrupt(hal_smbus_dev_struct *smbus, uint8_t *p_buffer, \
                                    uint32_t length, hal_smbus_user_cb p_user_func)
{
    __IO uint32_t timeout_ms = HAL_TIMEOUT_FOREVER;
    
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == smbus) || (NULL == p_buffer) || (0 == length)){
        HAL_DEBUGE("parameter [smbus] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* SMBUS state is busy */
    if(SMBUS_STATE_BUSY == smbus->rx_state){
        HAL_DEBUGE("smbus tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* update the values of SMBUS dev struct */
    smbus->error_state = HAL_SMBUS_ERROR_NONE;
    smbus->rxbuffer.buffer = (uint8_t *)p_buffer; 
    smbus->rxbuffer.length = length;
    smbus->rxbuffer.pos = 0;

    /* SMBUS state is ready */
    if(SMBUS_STATE_READY == smbus->rx_state){
        if(HAL_ERR_NONE != _smbus_wait_flag_timeout(smbus->periph, I2C_FLAG_I2CBSY, RESET, timeout_ms)){
            HAL_DEBUGW("i2c busy timeout");
            return HAL_ERR_TIMEOUT;
        }
        
        /* update the values of SMBUS dev struct */
        smbus->rx_state = SMBUS_STATE_BUSY;
        smbus->rx_callback = (void*)p_user_func;
        smbus->smbus_irq.event_handle = _smbus_slave_receive_interrupt;
        
        /* enable the I2C interrupt */
        i2c_interrupt_enable(smbus->periph, I2C_INT_BUF);
        i2c_interrupt_enable(smbus->periph, I2C_INT_EV);
        
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      Enable the SMBUS alert mode with Interrupt.
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_smbus_enable_alert_interrupt(hal_smbus_dev_struct *smbus)
{
    /* Enable SMBus alert */
    i2c_smbus_issue_alert(smbus->periph,I2C_SALTSEND_ENABLE);

    /* Clear ALERT flag */
    i2c_flag_clear(smbus->periph,I2C_FLAG_SMBALT);

    /* Enable Alert Interrupt */
    i2c_interrupt_enable(smbus->periph,I2C_INT_ERR);

    return HAL_ERR_NONE;
}

/*!
    \brief      Disable the SMBUS alert mode with Interrupt.
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_smbus_disable_alert_interrupt(hal_smbus_dev_struct *smbus)
{
    /* Enable SMBus alert */
    i2c_smbus_issue_alert(smbus->periph,I2C_SALTSEND_DISABLE);

    /* Disable Alert Interrupt */
    i2c_interrupt_disable(smbus->periph,I2C_INT_ERR);

    return HAL_ERR_NONE;
}

/*!
    \brief      wait the flag status until timeout
    \param[in]  smbus_periph: I2Cx(x=0,1)
    \param[in]  flag: I2C flags, refer to i2c_flag_enum
    \param[in]  status: the status of I2C flag to wait 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     none
*/
static int32_t _smbus_wait_flag_timeout(uint32_t smbus_periph, i2c_flag_enum flag, FlagStatus status, uint32_t timeout_ms)
{
    uint32_t tick_start = 0;
    
    tick_start = hal_basetick_count_get();
    /* wait flag status RESET */
    if(RESET == status){
        while(SET == i2c_flag_get(smbus_periph, flag)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("i2c get flag timeout");
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
    }else{
        /* wait flag status SET */
        while(RESET == i2c_flag_get(smbus_periph, flag)){
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
    \brief      event handler in SMBUS master transmit mode
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smbus_master_transmit_interrupt(void *smbus)
{
    uint32_t length = 0;
    
    hal_smbus_dev_struct *p_smbus = smbus;
    hal_smbus_user_cb p_func = (hal_smbus_user_cb)p_smbus->tx_callback;
    
    length = p_smbus->txbuffer.length;
    
    if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_SBSEND)){
        /* send slave address */
        i2c_master_addressing(p_smbus->periph, p_smbus->slave_address, SMBUS_TRANSMITTER);
    }else if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_ADDSEND)){
        /*clear ADDSEND bit */
        i2c_interrupt_flag_clear(p_smbus->periph, I2C_INT_FLAG_ADDSEND);
    }else if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_TBE)){
        if(p_smbus->txbuffer.pos < length){
            /* transmit smbus data */
            i2c_data_transmit(p_smbus->periph, (*p_smbus->txbuffer.buffer & (uint8_t)0xFFU));
            
            p_smbus->txbuffer.buffer++;
            p_smbus->txbuffer.pos++;

        }else if(p_smbus->txbuffer.pos == length){
            /* transfer the pec value */
            p_smbus->txbuffer.pos++;
            i2c_pec_transfer_enable(p_smbus->periph, p_smbus->smbus_init.smbus_pec_transfer);
        }else{
            /* the master sends a stop condition to I2C bus */
            i2c_stop_on_bus(p_smbus->periph);
            /* enable the I2C interrupt */
            i2c_interrupt_disable(p_smbus->periph, I2C_INT_EV);
            i2c_interrupt_disable(p_smbus->periph, I2C_INT_BUF);
            p_smbus->smbus_irq.event_handle = NULL;
            
            p_smbus->tx_state = SMBUS_STATE_READY;
        }
    }
    
    if(NULL != p_func){
        p_func(p_smbus);
    }

}

/*!
    \brief      event handler in SMBUS slave transmit mode
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smbus_slave_transmit_interrupt(void *smbus)
{
    uint32_t length = 0;

    hal_smbus_dev_struct *p_smbus = smbus;
    
    length = p_smbus->txbuffer.length;
    
    if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_ADDSEND)){
        /* clear the ADDSEND bit */
        i2c_interrupt_flag_clear(p_smbus->periph, I2C_INT_FLAG_ADDSEND);
    }else if((i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_TBE))&&(!i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_AERR))){
        if (p_smbus->txbuffer.pos < length){
                /* transmit smbus data */
                i2c_data_transmit(p_smbus->periph, (*p_smbus->txbuffer.buffer & (uint8_t)0xFFU));
                
                p_smbus->txbuffer.buffer++;
                p_smbus->txbuffer.pos++;
        }else if(p_smbus->txbuffer.pos == length){
                /* transfer the pec value */
                i2c_pec_transfer_enable(p_smbus->periph, p_smbus->smbus_init.smbus_pec_transfer);
            
            i2c_interrupt_disable(p_smbus->periph, I2C_INT_BUF);
            i2c_interrupt_disable(p_smbus->periph, I2C_INT_EV);
            p_smbus->smbus_irq.event_handle = NULL;
            p_smbus->tx_state = SMBUS_STATE_READY;
        }
    }
}

/*!
    \brief      event handler in SMBUS master receive mode
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smbus_master_receive_interrupt(void *smbus)
{
    uint32_t length = 0;

    hal_smbus_dev_struct *p_smbus = smbus;
    
    length = p_smbus->rxbuffer.length;
    
    if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_SBSEND)){
        /* the master sends slave address */
        i2c_master_addressing(p_smbus->periph, p_smbus->slave_address, SMBUS_RECEIVER);
    }else if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_ADDSEND)){
        if((length == 2)||(length == 1)){
            /* clear the ACKEN before the ADDSEND is cleared, PEC mode do not need */
            if (p_smbus->smbus_init.smbus_pec_transfer != SMBUS_PECTRANS_ENABLE){
                i2c_ack_config(p_smbus->periph, I2C_ACK_DISABLE);
            }
            
            /* clear the ADDSEND bit */
            i2c_interrupt_flag_clear(p_smbus->periph,I2C_INT_FLAG_ADDSEND);
        }else{
            /* clear the ADDSEND bit */
            i2c_interrupt_flag_clear(p_smbus->periph,I2C_INT_FLAG_ADDSEND);
        }
    }
    else if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_RBNE)){
        if(p_smbus->rxbuffer.pos < length){
            if(p_smbus->rxbuffer.pos == length - 3){
                /* wait until the second last data byte is received into the shift register */
                while(!i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_BTC));
                /* send a NACK for the last data byte, PEC mode do not need */
                if (p_smbus->smbus_init.smbus_pec_transfer != SMBUS_PECTRANS_ENABLE){
                    i2c_ack_config(p_smbus->periph, I2C_ACK_DISABLE);
                }
            }
            /* read a data byte from I2C_DATA*/
            *p_smbus->rxbuffer.buffer = i2c_data_receive(p_smbus->periph);
            
            p_smbus->rxbuffer.buffer++;
            p_smbus->rxbuffer.pos++;
            
            if(p_smbus->rxbuffer.pos == length){
                hal_smbus_user_cb p_func = (hal_smbus_user_cb)p_smbus->rx_callback;
                /* wait until the RBNE bit is set */
                /* check the received PEC value */
                i2c_pec_transfer_enable(p_smbus->periph, p_smbus->smbus_init.smbus_pec_transfer);
                
                /* send a stop condition */
                i2c_stop_on_bus(p_smbus->periph);
                i2c_ack_config(p_smbus->periph, I2C_ACK_ENABLE);
                i2c_ackpos_config(p_smbus->periph, I2C_ACKPOS_CURRENT);
                
                p_smbus->rx_state = SMBUS_STATE_READY;
                p_smbus->smbus_irq.event_handle = NULL;
                /* disable the I2C0 interrupt */
                i2c_interrupt_disable(p_smbus->periph, I2C_INT_BUF);
                i2c_interrupt_disable(p_smbus->periph, I2C_INT_EV);
                
                if(NULL != p_func){
                    p_func(p_smbus);
                }
            }
        }
    }
}

/*!
    \brief      event handler in SMBUS slave receive mode
    \param[in]  smbus: SMBUS device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _smbus_slave_receive_interrupt(void *smbus)
{
    uint32_t length = 0;

    hal_smbus_dev_struct *p_smbus = smbus;
    
    length = p_smbus->txbuffer.length;
    
    if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_ADDSEND)){
        /* clear the ADDSEND bit */
        i2c_interrupt_flag_clear(p_smbus->periph, I2C_INT_FLAG_ADDSEND);
    }else if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_RBNE)){
        /* if reception data register is not empty ,I2Cx will read a data from I2C_DATA */
        *p_smbus->rxbuffer.buffer = i2c_data_receive(p_smbus->periph);
        p_smbus->rxbuffer.buffer++;
        p_smbus->rxbuffer.pos++;
        
        if(p_smbus->rxbuffer.pos == length){
            /* check the received PEC value */
            i2c_pec_transfer_enable(p_smbus->periph, p_smbus->smbus_init.smbus_pec_transfer);
        }
    }else if(i2c_interrupt_flag_get(p_smbus->periph, I2C_INT_FLAG_STPDET)){
        hal_smbus_user_cb p_func = (hal_smbus_user_cb)p_smbus->rx_callback;
        /* clear the STPDET bit */
        i2c_enable(p_smbus->periph);
        
        p_smbus->smbus_irq.event_handle = NULL;
        p_smbus->rx_state = SMBUS_STATE_READY;
        
        /* disable I2Cx interrupt */
        i2c_interrupt_disable(p_smbus->periph, I2C_INT_BUF);
        i2c_interrupt_disable(p_smbus->periph, I2C_INT_EV);
        
        if(NULL != p_func){
            p_func(p_smbus);
        }
    }
}
