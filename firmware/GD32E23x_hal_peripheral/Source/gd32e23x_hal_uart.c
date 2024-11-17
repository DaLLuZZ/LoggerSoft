/*!
    \file    gd32e23x_hal_uart.c
    \brief   UART driver
    
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

static uint16_t _uart_data_bit_mask_get(hal_uart_dev_struct *uart);
static FlagStatus _uart_error_flag_get(hal_uart_dev_struct *uart);
static void _uart_transmit_complete_interrupt(void *uart);
static void _uart_transmit_interrupt(void *uart);
static void _uart_receive_interrupt(void *uart);
static void _uart_transmit_dma(void *dma);
static void _uart_receive_dma(void *dma);
static void _uart_dma_error(void *dma);

/*!
    \brief      initialize the UART struct with the default values, note that this function must be
                called after the structure is created
    \param[in]  hal_struct_type: type of UART struct for initialization
      \arg        HAL_UART_INIT_STRUCT: initialization structure
      \arg        HAL_UART_INIT_EX_STRUCT: initialization extend structure
      \arg        HAL_UART_DEV_STRUCT: device information structure
      \arg        HAL_UART_USER_CALLBCAK_STRUCT: user callback structure
      \arg        HAL_UART_IRQ_INIT_STRUCT: interrupt callback initialization structure
    \param[in]  p_struct: structure pointer
    \param[out] none
    \retval     none
*/
void hal_uart_struct_init(hal_uart_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(hal_struct_type){
    case HAL_UART_INIT_STRUCT:
        /* initialize uart initialization structure with the default values */
        ((hal_uart_init_struct*)p_struct)->work_mode = UART_WORK_MODE_ASYN;
        ((hal_uart_init_struct*)p_struct)->baudrate = 115200U;
        ((hal_uart_init_struct*)p_struct)->direction = UART_DIRECTION_RX_TX;
        ((hal_uart_init_struct*)p_struct)->over_sample = UART_OVER_SAMPLE_16;
        ((hal_uart_init_struct*)p_struct)->sample_method = UART_THREE_SAMPLE_BIT;
        ((hal_uart_init_struct*)p_struct)->parity = UART_PARITY_NONE;
        ((hal_uart_init_struct*)p_struct)->stop_bit = UART_STOP_BIT_1;
        ((hal_uart_init_struct*)p_struct)->word_length = UART_WORD_LENGTH_8BIT;
        ((hal_uart_init_struct*)p_struct)->hardware_flow = UART_HARDWARE_FLOW_NONE;
    
        ((hal_uart_init_struct*)p_struct)->lin_mode.break_frame_length = UART_LIN_BREAK_DETECTION_10BIT;
        ((hal_uart_init_struct*)p_struct)->multiprocessor_mode.address = 0U;
        ((hal_uart_init_struct*)p_struct)->multiprocessor_mode.addr_length = UART_MULTIPROCESSOR_ADDRESS_4BIT;
        ((hal_uart_init_struct*)p_struct)->multiprocessor_mode.wakeup_mode = UART_MULTIPROCESSOR_WAKEUP_IDLE;
        ((hal_uart_init_struct*)p_struct)->rs485_mode.de_assertion_time = 0U;
        ((hal_uart_init_struct*)p_struct)->rs485_mode.de_deassertion_time = 0U;
        ((hal_uart_init_struct*)p_struct)->rs485_mode.de_polarity = UART_RS485_DE_POLARITY_HIGH;
        break;
    
    case HAL_UART_INIT_EX_STRUCT:
        /* initialize uart extend initialization structure with the default values */
        ((hal_uart_init_ex_struct*)p_struct)->data_bit_invert = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->first_bit_msb = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->overrun_disable = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->rx_error_dma_stop = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->rx_level_invert = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->tx_level_invert = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->tx_rx_swap = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->autobaud.use = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->autobaud.detection_mode = UART_AUTOBAUD_BY_BIT_SEQUENCE_1XXXXXXX;
        ((hal_uart_init_ex_struct*)p_struct)->wakeup.use = DISABLE;
        ((hal_uart_init_ex_struct*)p_struct)->wakeup.wakeup_by = UART_WAKEUP_BY_ADDRESS;
        break;
    
    case HAL_UART_DEV_STRUCT:
        /* initialize uart device information structure with the default values */
        ((hal_uart_dev_struct*)p_struct)->periph = 0U;
        ((hal_uart_dev_struct*)p_struct)->uart_irq.error_handle = NULL;
        ((hal_uart_dev_struct*)p_struct)->uart_irq.receive_complete_handle = NULL;
        ((hal_uart_dev_struct*)p_struct)->uart_irq.transmit_complete_handle = NULL;
        ((hal_uart_dev_struct*)p_struct)->uart_irq.transmit_ready_handle = NULL;
        ((hal_uart_dev_struct*)p_struct)->uart_irq.wakeup_handle = NULL;
        ((hal_uart_dev_struct*)p_struct)->p_dma_rx = NULL;
        ((hal_uart_dev_struct*)p_struct)->p_dma_tx = NULL;
        ((hal_uart_dev_struct*)p_struct)->txbuffer.buffer = NULL;
        ((hal_uart_dev_struct*)p_struct)->txbuffer.length = 0U;
        ((hal_uart_dev_struct*)p_struct)->txbuffer.pos = 0U;
        ((hal_uart_dev_struct*)p_struct)->rxbuffer.buffer = NULL;
        ((hal_uart_dev_struct*)p_struct)->rxbuffer.length = 0U;
        ((hal_uart_dev_struct*)p_struct)->rxbuffer.pos = 0U;
        ((hal_uart_dev_struct*)p_struct)->error_state = HAL_USART_ERROR_NONE;
        ((hal_uart_dev_struct*)p_struct)->tx_state = UART_STATE_FREE;
        ((hal_uart_dev_struct*)p_struct)->rx_state = UART_STATE_FREE;
        ((hal_uart_dev_struct*)p_struct)->last_error = HAL_USART_ERROR_NONE;
        ((hal_uart_dev_struct*)p_struct)->priv = NULL;
        break;
    
    case HAL_UART_USER_CALLBCAK_STRUCT:
        /* initialize user callback structure with the default values */
        ((hal_uart_user_callback_struct*)p_struct)->complete_func = NULL;
        ((hal_uart_user_callback_struct*)p_struct)->error_func = NULL;
        break;
    
    case HAL_UART_IRQ_INIT_STRUCT:
        /* initialize interrupt callback structure with the default values */
        ((hal_uart_irq_struct*)p_struct)->address_match_handle = NULL;
        ((hal_uart_irq_struct*)p_struct)->cts_change_handle = NULL;
        ((hal_uart_irq_struct*)p_struct)->error_handle = NULL;
        ((hal_uart_irq_struct*)p_struct)->idle_line_detected_handle = NULL;
        ((hal_uart_irq_struct*)p_struct)->lin_break_detected_handle = NULL;
        ((hal_uart_irq_struct*)p_struct)->receive_complete_handle = NULL;
        ((hal_uart_irq_struct*)p_struct)->transmit_complete_handle = NULL;
        ((hal_uart_irq_struct*)p_struct)->transmit_ready_handle = NULL;
        ((hal_uart_irq_struct*)p_struct)->wakeup_handle = NULL;
        break;
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize the UART
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_uart_deinit(hal_uart_dev_struct *uart)
{
    uint32_t periph;
    
    periph = uart->periph;
    if((USART0 == periph) || (USART1 == periph)){
        /* deinitialize the periph and the device information sturcture */
        usart_deinit(periph);
        hal_uart_struct_init(HAL_UART_DEV_STRUCT, uart);
        uart->periph = periph;
    }else{
        HAL_DEBUGE("parameter [uart->periph] value is invalid");
    }
}

/*!
    \brief      initialize the UART with specified values
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is initialized
    \param[in]  p_init: the initialization data needed to initialize UART
                  work_mode: UART_WORK_MODE_ASYN, UART_WORK_MODE_SINGLE_WIRE, UART_WORK_MODE_MULTIPROCESSCOR
                             UART_WORK_MODE_LIN, UART_WORK_MODE_RS485
                  baudrate: communication baudrate
                  parity: UART_PARITY_NONE, UART_PARITY_EVEN, UART_PARITY_ODD
                  word_length: UART_WORD_LENGTH_8BIT, UART_WORD_LENGTH_9BIT
                  stop_bit: UART_STOP_BIT_1, UART_STOP_BIT_0_5, UART_STOP_BIT_2, UART_STOP_BIT_1_5
                  over_sample: UART_OVER_SAMPLE_8, UART_OVER_SAMPLE_16
                  sample_method: UART_THREE_SAMPLE_BIT, UART_ONE_SAMPLE_BIT
                  direction: UART_DIRECTION_RX_TX, UART_DIRECTION_RX_ONLY, UART_DIRECTION_TX_ONLY
                  hardware_flow: UART_HARDWARE_FLOW_NONE, UART_HARDWARE_FLOW_RTS_ONLY, UART_HARDWARE_FLOW_CTS_ONLY
                                 UART_HARDWARE_FLOW_RTS_CTS
                  lin_mode: LIN mode struct (USART0 only)
                    break_frame_length: UART_LIN_BREAK_DETECTION_10BIT, UART_LIN_BREAK_DETECTION_11BIT
                  rs485_mode: RS485 mode struct
                    de_polarity: UART_RS485_DE_POLARITY_HIGH, UART_RS485_DE_POLARITY_LOW
                    de_assertion_time: 0 - 31
                    de_deassertion_time: 0 - 31
                  multiprocessor_mode: multi-processor mode struct
                    wakeup_mode: UART_MULTIPROCESSOR_WAKEUP_IDLE, UART_MULTIPROCESSOR_WAKEUP_ADDRESS
                    address: 0 - 15(4-bit address detection), 0 - 255(full-bit address detection)
                    addr_length: UART_MULTIPROCESSOR_ADDRESS_4BIT, UART_MULTIPROCESSOR_ADDRESS_FULLBIT
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_uart_init(hal_uart_dev_struct *uart, uint32_t periph, \
                      hal_uart_init_struct *p_init)
{
    uint32_t reg_temp;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check UART pointer and p_init address */
    if((NULL == uart) || (NULL == p_init)){
        HAL_DEBUGE("pointer [uart] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check periph parameter */
    if((USART0 != periph) && (USART1 != periph)){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }
    
    /* check periph value from UART device struct */
    if(0 != uart->periph){
        HAL_DEBUGI("periph value from uart device struct has been rewrite");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    uart->periph = periph;
    uart->work_mode = p_init->work_mode;
    
    usart_disable(periph);
    
    /* CTL0 register configure */
    reg_temp = USART_CTL0(periph);
    reg_temp &= ~(USART_CTL0_PM | USART_CTL0_PCEN | USART_CTL0_WL | USART_CTL0_OVSMOD | \
                  USART_CTL0_REN | USART_CTL0_TEN);
    reg_temp |= (p_init->direction | p_init->over_sample | p_init->parity | \
                 p_init->word_length);
    USART_CTL0(periph) = reg_temp;
    
    /* CTL1 register configure */
    reg_temp = USART_CTL1(periph);
    reg_temp &= ~(USART_CTL1_STB);
    reg_temp |= (p_init->stop_bit);
    USART_CTL1(periph) = reg_temp;
    
    /* CTL2 register configure */
    reg_temp = USART_CTL2(periph);
    reg_temp &= ~(USART_CTL2_CTSEN | USART_CTL2_RTSEN | USART_CTL2_OSB);
    reg_temp |= (p_init->hardware_flow | p_init->sample_method);
    USART_CTL2(periph) = reg_temp;
    
    /* boudrate configure */
    usart_baudrate_set(periph, p_init->baudrate);
    /* get the data bit mask */
    uart->data_bit_mask = _uart_data_bit_mask_get(uart);
    /* disable the SMARTCARD mode, Half-Duplex mode, IRDA mode, LIN mode and clock */
    usart_smartcard_mode_disable(periph);
    usart_halfduplex_disable(periph);
    usart_irda_mode_disable(periph);
    usart_clock_disable(periph);
    usart_lin_mode_disable(periph);
    
    if(UART_WORK_MODE_ASYN != p_init->work_mode){
        switch(p_init->work_mode){
        case UART_WORK_MODE_MULTIPROCESSCOR:
            /* multiprocessor mode configure */
            usart_mute_mode_wakeup_config(periph, p_init->multiprocessor_mode.wakeup_mode);
            
            if(UART_MULTIPROCESSOR_WAKEUP_IDLE != p_init->multiprocessor_mode.wakeup_mode){
                usart_address_detection_mode_config(periph, p_init->multiprocessor_mode.addr_length);
                usart_address_config(periph, p_init->multiprocessor_mode.address);
            }
            /* enable mute mode */
            usart_mute_mode_enable(periph);
            break;
        case UART_WORK_MODE_LIN:
            /* LIN mode configure */
            usart_lin_break_detection_length_config(periph, p_init->lin_mode.break_frame_length);
            usart_lin_mode_enable(periph);
            break;
        case UART_WORK_MODE_RS485:
            p_init->rs485_mode.de_assertion_time &= BITS(0,4);
            p_init->rs485_mode.de_deassertion_time &= BITS(0,4);
        
            /* RS485 mode configure */
            usart_driver_assertime_config(periph, p_init->rs485_mode.de_assertion_time);
            usart_driver_deassertime_config(periph, p_init->rs485_mode.de_deassertion_time);
            usart_depolarity_config(periph, p_init->rs485_mode.de_polarity);
            /* enable RS485 driver */
            usart_rs485_driver_enable(periph);
            break;
        case UART_WORK_MODE_SINGLE_WIRE:
            /* enable single wire(half-duplex) mode */
            usart_halfduplex_enable(periph);
            break;
        default:
            break;
        } 
    }
    
    /* reset the Rx and Tx state */
    uart->tx_state = UART_STATE_FREE;
    uart->rx_state = UART_STATE_FREE;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      initialize the UART with specified values when using extended functions
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is initialized
    \param[in]  p_init: the initialization data needed to initialize UART using extended functions
                  first_bit_msb: ENABLE, DISABLE
                  tx_rx_swap: ENABLE, DISABLE
                  rx_level_invert: ENABLE, DISABLE
                  tx_level_invert: ENABLE, DISABLE
                  data_bit_invert: ENABLE, DISABLE
                  overrun_disable: ENABLE, DISABLE
                  rx_error_dma_stop: ENABLE, DISABLE
                  autobaud: auto baudrate detection struct
                    use: ENABLE, DISABLE
                    detection_mode: UART_AUTOBAUD_BY_BIT_SEQUENCE_1XXXXXXX, UART_AUTOBAUD_BY_BIT_SEQUENCE_10XXXXXX
                  wakeup: wakeup mode struct
                    use: ENABLE, DISABLE
                    wakeup_by: UART_WAKEUP_BY_ADDRESS, UART_WAKEUP_BY_START_BIT, UART_WAKEUP_BY_RBNE_SET
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_uart_init_ex(hal_uart_dev_struct *uart, uint32_t periph, \
                      hal_uart_init_ex_struct *p_init)
{
    uint32_t reg_temp;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_init)){
        HAL_DEBUGE("pointer [uart] or [p_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameters */
    if((USART0 != periph) && (USART1 != periph)){
        HAL_DEBUGE("parameter [periph] value is invalid"); 
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    uart->periph = periph;
    usart_disable(periph);
    
    /* configure the data bit invert function */
    if(ENABLE == p_init->data_bit_invert){
        USART_CTL1(periph) |= USART_CTL1_DINV;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_DINV);
    }
    
    /* configure the rx level invert function */
    if(ENABLE == p_init->rx_level_invert){
        USART_CTL1(periph) |= USART_CTL1_RINV;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_RINV);
    }
    
    /* configure the tx level invert function */
    if(ENABLE == p_init->tx_level_invert){
        USART_CTL1(periph) |= USART_CTL1_TINV;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_TINV);
    }
    
    /* configure the tx rx swap function */
    if(ENABLE == p_init->tx_rx_swap){
        USART_CTL1(periph) |= USART_CTL1_STRP;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_STRP);
    }
    
    /* configure the first bit MSB function */
    if(ENABLE == p_init->first_bit_msb){
        USART_CTL1(periph) |= USART_CTL1_MSBF;
    }else{
        USART_CTL1(periph) &= ~(USART_CTL1_MSBF);
    }
    
    /* configure the overrun function */
    if(ENABLE == p_init->overrun_disable){
        USART_CTL2(periph) |= USART_CTL2_OVRD;
    }else{
        USART_CTL2(periph) &= ~(USART_CTL2_OVRD);
    }
    
    /* configure the rx error DMA stop function */
    if(ENABLE == p_init->rx_error_dma_stop){
        USART_CTL2(periph) |= USART_CTL2_DDRE;
    }else{
        USART_CTL2(periph) &= ~(USART_CTL2_DDRE);
    }
    
    /* configure the auto baudrate function */
    if(ENABLE == p_init->autobaud.use){
        reg_temp = USART_CTL1(periph);
        reg_temp &= ~(USART_CTL1_ABDM);
        reg_temp |= p_init->autobaud.detection_mode;
        USART_CTL1(periph) = reg_temp; 
    }
    
    /* configure the wakeup from deep-sleep function */
    if(ENABLE == p_init->wakeup.use){
        reg_temp = USART_CTL2(periph);
        reg_temp &= ~(USART_CTL2_WUM);
        reg_temp |= p_init->wakeup.wakeup_by;
        USART_CTL2(periph) = reg_temp;
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start UART module, note that this function must be called to start USART after initialization
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_uart_start(hal_uart_dev_struct *uart)
{
    usart_enable(uart->periph);
}

/*!
    \brief      stop UART module
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_uart_stop(hal_uart_dev_struct *uart)
{
    usart_disable(uart->periph);
}

/*!
    \brief      handle the UART interrupts
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_uart_irq(hal_uart_dev_struct *uart)
{
    if(RESET == _uart_error_flag_get(uart)){
        /* check whether USART is in receiver mode or not */
        if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_RBNE)){
            if(NULL != uart->uart_irq.receive_complete_handle){
                uart->uart_irq.receive_complete_handle(uart);
            }
            return;
        }
    }else{
        /* check whether the PERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_PERR)){
            usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_PERR);
            uart->error_state |= HAL_USART_ERROR_PERR;
            uart->last_error = HAL_USART_ERROR_PERR;
        }
        
        /* check whether the NERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_ERR_NERR)){
            usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_NERR);
            uart->error_state |= HAL_USART_ERROR_NERR;
            uart->last_error = HAL_USART_ERROR_NERR;
        }
        
        /* check whether the FERR flag is set or not */
        if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_ERR_FERR)){
            usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_FERR);
            uart->error_state |= HAL_USART_ERROR_FERR;
            uart->last_error = HAL_USART_ERROR_FERR;
        }
        
        /* check whether the ERR ORERR is set or not */
        if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_ERR_ORERR)){ 
            usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_ORERR);
            uart->error_state |= HAL_USART_ERROR_ORERR;
            uart->last_error = HAL_USART_ERROR_ORERR;
        }
        
        /* check whether RBNE ORERR is set or not */
        if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_RBNE_ORERR)){ 
            usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_RBNE_ORERR);
            uart->error_state |= HAL_USART_ERROR_ORERR;
            uart->last_error = HAL_USART_ERROR_ORERR;
        }
        
        /* check whether error state is none or not */
        if(HAL_USART_ERROR_NONE != uart->error_state){
            if(uart->uart_irq.error_handle != NULL){
                uart->uart_irq.error_handle(uart);
                uart->error_state = HAL_USART_ERROR_NONE;
            }
            return;
        }
    }
    
    /* multi-processor mode interrupt handle */
    if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_IDLE)){
        usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_IDLE);
        if(NULL != uart->uart_irq.idle_line_detected_handle){
            uart->uart_irq.idle_line_detected_handle(uart);
        }
    }
    
    /* address match interrput handle */
    if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_AM)){
        usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_AM);
        if(NULL != uart->uart_irq.address_match_handle){
            uart->uart_irq.address_match_handle(uart);
        }
    }
    
    /* LIN mode interrupt handle */
    if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_LBD)){
        usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_LBD);
        if(NULL != uart->uart_irq.lin_break_detected_handle){
            uart->uart_irq.lin_break_detected_handle(uart);
        }
    }
    
    /* wakeup from deepsleep mode interrupt handle */
    if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_WU)){
        usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_WU);
        uart->rx_state = UART_STATE_FREE;
        uart->tx_state = UART_STATE_FREE;
        if(NULL != uart->uart_irq.wakeup_handle){
            uart->uart_irq.wakeup_handle(uart);
        }
    }
    /* hardware flow mode interrupt handle */
    if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_CTS)){
        usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_CTS);
        if(NULL != uart->uart_irq.cts_change_handle){
            uart->uart_irq.cts_change_handle(uart);
        }
    }
    /* transmitter buffer empty interrupt handle */
    if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_TBE)){
        if(NULL != uart->uart_irq.transmit_ready_handle){
            uart->uart_irq.transmit_ready_handle(uart);
        }
        
        return;
    }
    /* transmission complete interrupt handle */
    if(RESET != usart_interrupt_flag_get(uart->periph, USART_INT_FLAG_TC)){
        usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_TC);
        
        if(NULL != uart->uart_irq.transmit_complete_handle){
            uart->uart_irq.transmit_complete_handle(uart);
        }
        
        return;
    }
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  uart: UART device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to UART interrupt callback functions structure
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
void hal_uart_irq_handle_set(hal_uart_dev_struct *uart, hal_uart_irq_struct *p_irq)
{
    /* set user-defined address match interrupt callback */
    if(NULL != p_irq->address_match_handle){
        uart->uart_irq.address_match_handle = p_irq->address_match_handle;
        usart_interrupt_enable(uart->periph, USART_INT_AM);
    }else{
        uart->uart_irq.address_match_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_AM);
    }
    
    /* set user-defined CTS change interrupt callback */
    if(NULL != p_irq->cts_change_handle){
        uart->uart_irq.cts_change_handle = p_irq->cts_change_handle;
        usart_interrupt_enable(uart->periph, USART_INT_CTS);
    }else{
        uart->uart_irq.cts_change_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_CTS);
    }
    
    /* set user-defined error interrupt callback */
    if(NULL != p_irq->error_handle){
        uart->uart_irq.error_handle = p_irq->error_handle;
        usart_interrupt_enable(uart->periph, USART_INT_ERR);
        usart_interrupt_enable(uart->periph, USART_INT_PERR);
    }else{
        uart->uart_irq.error_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_ERR);
        usart_interrupt_disable(uart->periph, USART_INT_PERR);
    }
    
    /* set user-defined idle line detected interrupt callback */
    if(NULL != p_irq->idle_line_detected_handle){
        uart->uart_irq.idle_line_detected_handle = p_irq->idle_line_detected_handle;
        usart_interrupt_enable(uart->periph, USART_INT_IDLE);
    }else{
        uart->uart_irq.idle_line_detected_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_IDLE);
    }
    
    /* set user-defined LIN break detected interrupt callback */
    if(NULL != p_irq->lin_break_detected_handle){
        uart->uart_irq.lin_break_detected_handle = p_irq->lin_break_detected_handle;
        usart_interrupt_enable(uart->periph, USART_INT_LBD);
    }else{
        uart->uart_irq.lin_break_detected_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_LBD);
    }
    
    /* set user-defined receive complete interrupt callback */
    if(NULL != p_irq->receive_complete_handle){
        uart->uart_irq.receive_complete_handle = p_irq->receive_complete_handle;
        usart_interrupt_enable(uart->periph, USART_INT_RBNE);
    }else{
        uart->uart_irq.receive_complete_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_RBNE);
    }
    
    /* set user-defined transmit complete interrupt callback */
    if(NULL != p_irq->transmit_complete_handle){
        uart->uart_irq.transmit_complete_handle = p_irq->transmit_complete_handle;
        usart_interrupt_enable(uart->periph, USART_INT_TC);
    }else{
        uart->uart_irq.transmit_complete_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_TC);
    }
    
    /* set user-defined transmit ready interrupt callback */
    if(NULL != p_irq->transmit_ready_handle){
        uart->uart_irq.transmit_ready_handle = p_irq->transmit_ready_handle;
        usart_interrupt_enable(uart->periph, USART_INT_TBE);
    }else{
        uart->uart_irq.transmit_ready_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_TBE);
    }
    
    /* set user-defined wakeup interrupt callback */
    if(NULL != p_irq->wakeup_handle){
        uart->uart_irq.wakeup_handle = p_irq->wakeup_handle;
        usart_interrupt_enable(uart->periph, USART_INT_WU);
    }else{
        uart->uart_irq.wakeup_handle = NULL;
        usart_interrupt_disable(uart->periph, USART_INT_WU);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_uart_irq_handle_all_reset(hal_uart_dev_struct *uart)
{
    /* configure interrupt callback function to NULL */
    uart->uart_irq.address_match_handle = NULL;
    uart->uart_irq.cts_change_handle = NULL;
    uart->uart_irq.error_handle = NULL;
    uart->uart_irq.idle_line_detected_handle = NULL;
    uart->uart_irq.lin_break_detected_handle = NULL;
    uart->uart_irq.receive_complete_handle = NULL;
    uart->uart_irq.transmit_complete_handle = NULL;
    uart->uart_irq.transmit_ready_handle = NULL;
    uart->uart_irq.wakeup_handle = NULL;
}

/*!
    \brief      transmit amounts of data, poll transmit process and completed status
                the function is blocking
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_uart_transmit_poll(hal_uart_dev_struct *uart, void *p_buffer, \
                                uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    uint32_t temp_val;
    uint32_t tick_start;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state){
        HAL_DEBUGE("uart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_WL)){
        if(RESET == (USART_CTL0(uart->periph) & USART_CTL0_PCEN)){
            data_length = 2U;
        }
    }
    
    /* initialize transmit parameters */
    uart->error_state = HAL_USART_ERROR_NONE;
    uart->txbuffer.buffer = (uint8_t *)p_buffer; 
    uart->txbuffer.length = length;
    uart->txbuffer.pos = 0U;
    uart->tx_state = UART_STATE_BUSY;
    /* configure timeout */
    tick_start = hal_basetick_count_get();
    
    temp_val = uart->txbuffer.pos;
    while(temp_val < uart->txbuffer.length){
        /* wait for transmit buffer empty */
        while(RESET == usart_flag_get(uart->periph, USART_FLAG_TBE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("uart transmit timeout");
                    /* reset the state */
                    uart->rx_state = UART_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        
        /* send the data to be transmitted */
        if(1U == data_length){
            usart_data_transmit(uart->periph, (*uart->txbuffer.buffer & (uint8_t)0xFFU));
            uart->txbuffer.buffer++;
        }else{
            usart_data_transmit(uart->periph, (*(uint16_t*)uart->txbuffer.buffer & (uint16_t)0x1FFU));
            uart->txbuffer.buffer += data_length;
        }
        /* change the transmit pointer */
        uart->txbuffer.pos++;
        temp_val = uart->txbuffer.pos;
    }
    
    /* wait for transmit complete */
    while(RESET == usart_flag_get(uart->periph, USART_FLAG_TC)){
        if(HAL_TIMEOUT_FOREVER != timeout_ms){
            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                HAL_DEBUGW("uart transmit timeout");
                return HAL_ERR_TIMEOUT;
            }
        }
    }
    
    /* change the Tx state to free */
    uart->tx_state = UART_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data, poll receive process and completed status
                the function is blocking
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  timeout_ms: timeout duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_uart_receive_poll(hal_uart_dev_struct *uart, void *p_buffer, \
                               uint32_t length, uint32_t timeout_ms)
{
    uint32_t data_length;
    uint32_t temp_val;
    uint32_t tick_start;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state){
        HAL_DEBUGE("uart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* calculate the data length */
    data_length = 1U;
    if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_WL)){
        if(RESET == (USART_CTL0(uart->periph) & USART_CTL0_PCEN)){
            data_length = 2U;
        }
    }
    
    /* initialize receive parameters */
    uart->error_state = HAL_USART_ERROR_NONE;
    uart->rxbuffer.buffer = (uint8_t *)p_buffer; 
    uart->rxbuffer.length = length;
    uart->rxbuffer.pos = 0U;
    uart->data_bit_mask = _uart_data_bit_mask_get(uart);
    uart->rx_state = UART_STATE_BUSY;
    /* configure timeout */
    tick_start = hal_basetick_count_get();
    
    temp_val = uart->rxbuffer.pos;
    while(temp_val < uart->rxbuffer.length){
        /* wait for read data buffer not empty */
        while(RESET == usart_flag_get(uart->periph, USART_FLAG_RBNE)){
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    HAL_DEBUGW("uart receive timeout");
                    /* reset the state */
                    uart->rx_state = UART_STATE_FREE;
                    return HAL_ERR_TIMEOUT;
                }            
            }
        }
        
        /* read data from data register */
        if(1 == data_length){
            *uart->rxbuffer.buffer = (uint8_t)(usart_data_receive(uart->periph) & uart->data_bit_mask);
            uart->rxbuffer.buffer++;
        }else{
            *(uint16_t*)uart->rxbuffer.buffer = (usart_data_receive(uart->periph) & uart->data_bit_mask);
             uart->rxbuffer.buffer += data_length;
        }
        
        /* change the receive pointer */
        uart->rxbuffer.pos++;
        temp_val = uart->rxbuffer.pos;
    }
    
    /* change the Rx state to free */
    uart->rx_state = UART_STATE_FREE;

    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by interrupt method, after the transfer is completed, 
                the user function is called
                the function is non-blocking
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_uart_transmit_interrupt(hal_uart_dev_struct *uart, void *p_buffer, \
                                     uint32_t length, hal_uart_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state){
        HAL_DEBUGE("uart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize transmit parameters */
    uart->tx_state = UART_STATE_BUSY;
    uart->error_state = HAL_USART_ERROR_NONE;
    uart->txbuffer.buffer = (uint8_t *)p_buffer; 
    uart->txbuffer.length = length;
    uart->txbuffer.pos = 0U;
    uart->tx_callback = (void *)p_user_func;
    /* configure the transmit ready and complete callback as the function implemented */
    uart->uart_irq.transmit_ready_handle = _uart_transmit_interrupt;
    uart->uart_irq.transmit_complete_handle = _uart_transmit_complete_interrupt;
    
    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_TC);

    /* enable the TBE interrupt */
    usart_interrupt_enable(uart->periph, USART_INT_TBE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by interrupt method, after the transfer is completed, 
                the user function is called
                the function is non-blocking
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_user_func: user callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_uart_receive_interrupt(hal_uart_dev_struct *uart, void *p_buffer, \
                                    uint32_t length, hal_uart_user_cb p_user_func)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state){
        HAL_DEBUGE("uart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize receive parameters */
    uart->rx_state = UART_STATE_BUSY;
    uart->error_state = HAL_USART_ERROR_NONE;
    uart->rxbuffer.buffer = (uint8_t *)p_buffer; 
    uart->rxbuffer.length = length;
    uart->rxbuffer.pos = 0U;
    uart->rx_callback = (void *)p_user_func;
    uart->uart_irq.receive_complete_handle = _uart_receive_interrupt;
    uart->data_bit_mask = _uart_data_bit_mask_get(uart);
    
    /* enable PERR, ERR, RBNE interrupt */
    usart_interrupt_enable(uart->periph, USART_INT_PERR);
    usart_interrupt_enable(uart->periph, USART_INT_ERR);
    usart_interrupt_enable(uart->periph, USART_INT_RBNE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      transmit amounts of data by DMA method, after the transfer is completed or error occurs, 
                the user function is called
                the function is non-blocking
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be transmitted
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_uart_transmit_dma(hal_uart_dev_struct *uart, void *p_buffer, \
                              uint32_t length, hal_uart_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameter */
    if(NULL == uart->p_dma_tx){
        HAL_DEBUGE("parameter [uart->p_dma_tx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state){
        HAL_DEBUGE("uart tx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    
    /* initialize transmit parameters */
    uart->tx_state = UART_STATE_BUSY;
    uart->error_state = HAL_USART_ERROR_NONE;
    uart->txbuffer.buffer = (uint8_t *)p_buffer; 
    uart->txbuffer.length = length;
    uart->txbuffer.pos = 0U;
    if(NULL != p_func){
        uart->tx_callback = (void *)p_func->complete_func;
        uart->uart_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    }else{
        uart->tx_callback = NULL;
        uart->uart_irq.error_handle = NULL;
    }
    uart->uart_irq.transmit_complete_handle = _uart_transmit_complete_interrupt;
    
    /* configure DMA interrupt callback function */
    dma_irq.full_finish_handle = _uart_transmit_dma;
    dma_irq.error_handle = _uart_dma_error;
    if(NULL != uart->p_dma_tx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = uart->p_dma_tx->dma_irq.half_finish_handle;
    }
    
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(uart->p_dma_tx, (uint32_t)uart->txbuffer.buffer, \
                           uart->periph + 0x28U, length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(uart->p_dma_tx->channel, DMA_FLAG_G);
    
    /* clear USART TC interrupt flag */
    usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_TC);

    /* DMA enable for transmission */
    usart_dma_transmit_config(uart->periph, USART_DENT_ENABLE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      receive amounts of data by DMA method, after the transfer is completed or error occurs, 
                the user function is called
                the function is non-blocking
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  length: number of data to be received
    \param[in]  p_func: pointer to callback function
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_uart_receive_dma(hal_uart_dev_struct *uart, void *p_buffer, \
                              uint32_t length, hal_uart_user_callback_struct *p_func)
{
    hal_dma_irq_struct dma_irq;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == p_buffer) || (0U == length)){
        HAL_DEBUGE("parameter [uart] or [p_buffer] or [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameter */
    if(NULL == uart->p_dma_rx){
        HAL_DEBUGE("parameter [uart->p_dma_rx] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state){
        HAL_DEBUGE("uart rx has already been used, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
    
    /* initialize DMA interrupt callback function structure with default value */
    hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
    
    /* initialize receive parameters */
    uart->rx_state = UART_STATE_BUSY;
    uart->error_state = HAL_USART_ERROR_NONE;
    uart->rxbuffer.buffer = (uint8_t *)p_buffer; 
    uart->rxbuffer.length = length;
    uart->rxbuffer.pos = 0U;
    if(NULL != p_func){
        uart->rx_callback = (void *)p_func->complete_func;
        uart->uart_irq.error_handle = (hal_irq_handle_cb)p_func->error_func;
    }else{
        uart->rx_callback = NULL;
        uart->uart_irq.error_handle = NULL;
    }
    uart->data_bit_mask = _uart_data_bit_mask_get(uart);
    
    /* enable the usart parity error and error interrupt: (frame error, noise error, overrun error) */
    usart_interrupt_enable(uart->periph, USART_INT_PERR);
    usart_interrupt_enable(uart->periph, USART_INT_ERR);
    
    /* configure DMA interrupt interrupt callback function */
    dma_irq.full_finish_handle = _uart_receive_dma;
    dma_irq.error_handle = _uart_dma_error;
    if(NULL != uart->p_dma_rx->dma_irq.half_finish_handle){
        dma_irq.half_finish_handle = uart->p_dma_rx->dma_irq.half_finish_handle;
    }
    
    /* start DMA interrupt mode transfer */
    hal_dma_start_interrupt(uart->p_dma_rx, uart->periph + 0x24U,\
                            (uint32_t)uart->rxbuffer.buffer, length, &dma_irq);
    /* clear DMA global interrupt flag */
    dma_flag_clear(uart->p_dma_tx->channel, DMA_FLAG_G);
    /* DMA enable for reception */
    usart_dma_receive_config(uart->periph, USART_DENR_ENABLE);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      pause UART DMA transfer during transmission process
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_dma_pause(hal_uart_dev_struct *uart)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state){
        /* disable DMA transimt */
        usart_dma_transmit_config(uart->periph, USART_DENT_DISABLE);
    }
    
    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state){
        /* disable the PERR and ERR interrupt */
        usart_interrupt_disable(uart->periph, USART_INT_PERR);
        usart_interrupt_disable(uart->periph, USART_INT_ERR);
        
        /* disable DMA receive */
        usart_dma_receive_config(uart->periph, USART_DENR_DISABLE);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      resume UART DMA transfer during transmission process
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_dma_resume(hal_uart_dev_struct *uart)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the tx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->tx_state){
        /* enable DMA transimt */
        usart_dma_transmit_config(uart->periph, USART_DENT_ENABLE);
    }
    
    /* check the rx_state wheher is busy or not */
    if(UART_STATE_BUSY == uart->rx_state){
        /* enable the PERR and ERR interrupt */
        usart_interrupt_enable(uart->periph, USART_INT_PERR);
        usart_interrupt_enable(uart->periph, USART_INT_ERR);
        
        /* enable DMA receive */
        usart_dma_receive_config(uart->periph, USART_DENR_ENABLE);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      stop UART transmit transfer
                the function is blocking
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_transmit_stop(hal_uart_dev_struct *uart)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable the TBE and TC interrupt */
    usart_interrupt_disable(uart->periph, USART_INT_TBE);
    usart_interrupt_disable(uart->periph, USART_INT_TC);
    
    /* disable DMA transimt and stop DMA */
    usart_dma_transmit_config(uart->periph, USART_DENT_DISABLE);
    hal_dma_stop(uart->p_dma_tx);
    
    /* reset the position and state */
    uart->txbuffer.pos = 0;
    uart->tx_state = UART_STATE_FREE;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      stop UART receive transfer
                the function is blocking
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_receive_stop(hal_uart_dev_struct *uart)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable the RBNE, PERR and ERR interrupt */
    usart_interrupt_disable(uart->periph, USART_INT_RBNE);
    usart_interrupt_disable(uart->periph, USART_INT_PERR);
    usart_interrupt_disable(uart->periph, USART_INT_ERR);
    
    /* disable DMA receive and stop DMA */
    usart_dma_receive_config(uart->periph, USART_DENR_DISABLE);
    hal_dma_stop(uart->p_dma_rx);
    
    /* reset the position and state */
    uart->rxbuffer.pos = 0;
    uart->rx_state = UART_STATE_FREE;
    
    /* clear interrupt error flags */
    usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_PERR);
    usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_FERR);
    usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_NERR);
    usart_interrupt_flag_clear(uart->periph, USART_INT_FLAG_ERR_ORERR);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable the UART works in which mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  mode: work mode
      \arg        UART_WORK_MODE_SINGLE_WIRE: single wire(half-duplex) communication mode
      \arg        UART_WORK_MODE_MULTIPROCESSCOR: multiprocessor communication mode
      \arg        UART_WORK_MODE_LIN: LIN mode
      \arg        UART_WORK_MODE_RS485: RS485 mode
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_work_mode_enable(hal_uart_dev_struct *uart, hal_uart_work_mode_enum mode)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == mode)){
        HAL_DEBUGE("parameter [uart] or [mode] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;

    switch(mode){
        case UART_WORK_MODE_SINGLE_WIRE:
            /* enable the single wire (half-duplex) mode */
            usart_halfduplex_enable(uart->periph);
            break;
        case UART_WORK_MODE_MULTIPROCESSCOR:
            /* enable the multi-processor mode */
            usart_mute_mode_enable(uart->periph);
            break;
        case UART_WORK_MODE_LIN:
            /* enable the LIN mode */
            usart_lin_mode_enable(uart->periph);
            break;
        case UART_WORK_MODE_RS485:
            /* enable the RS485 mode */
            usart_rs485_driver_enable(uart->periph);
            break;
        default:
            HAL_DEBUGI("[mode] value is out of range");
            break;
    }
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }

    return HAL_ERR_NONE;
}

/*!
    \brief      disable the UART work mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  mode: work mode
      \arg        UART_WORK_MODE_SINGLE_WIRE: single wire(half-duplex) communication mode
      \arg        UART_WORK_MODE_MULTIPROCESSCOR: multiprocessor communication mode
      \arg        UART_WORK_MODE_LIN: LIN mode
      \arg        UART_WORK_MODE_RS485: RS485 mode
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_work_mode_disable(hal_uart_dev_struct *uart, hal_uart_work_mode_enum mode)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == mode)){
        HAL_DEBUGE("parameter [uart] or [mode] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    switch(mode){
        case UART_WORK_MODE_SINGLE_WIRE:
            /* disable the single wire (half-duplex) mode */
            usart_halfduplex_disable(uart->periph);
            break;
        case UART_WORK_MODE_MULTIPROCESSCOR:
            /* disable the multi-processor mode */
            usart_mute_mode_disable(uart->periph);
            break;
        case UART_WORK_MODE_LIN:
            /* disable the LIN mode */
            usart_lin_mode_disable(uart->periph);
            break;
        case UART_WORK_MODE_RS485:
            /* disable the RS485 mode */
            usart_rs485_driver_disable(uart->periph);
            break;
        default:
            HAL_DEBUGI("[mode] value is out of range");
            break;
    }
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      enable the UART command 
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  cmd: command
      \arg        HAL_UART_CMD_ENTER_MUTE: mute mode command 
      \arg        HAL_UART_CMD_SEND_BREAK: send break command 
      \arg        HAL_UART_CMD_AUTOBAUD_DETECTION: auto baudrate detection command
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_command_enable(hal_uart_dev_struct *uart, hal_uart_command_enum cmd)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || (NULL == cmd)){
        HAL_DEBUGE("parameter [uart] or [cmd] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    usart_command_enable(uart->periph, (uint32_t)cmd);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable transmit in single wire(half-duplex) mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_singlewire_transmit_enable(hal_uart_dev_struct *uart)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    /* disable Tx and Rx */
    usart_transmit_config(uart->periph, USART_TRANSMIT_DISABLE);
    usart_receive_config(uart->periph, USART_RECEIVE_DISABLE);
    
    /* enable Tx */
    usart_transmit_config(uart->periph, USART_TRANSMIT_ENABLE);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      disable transmit in single wire(half-duplex) mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_singlewire_transmit_disable(hal_uart_dev_struct *uart)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    /* disable Tx */
    usart_transmit_config(uart->periph, USART_TRANSMIT_DISABLE);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable receive in single wire(half-duplex) mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_singlewire_receive_enable(hal_uart_dev_struct *uart)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    /* disable Tx and Rx */
    usart_transmit_config(uart->periph, USART_TRANSMIT_DISABLE);
    usart_receive_config(uart->periph, USART_RECEIVE_DISABLE);
    
    /* enable Rx */
    usart_receive_config(uart->periph, USART_RECEIVE_ENABLE);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      disable receive in single wire(half-duplex) mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_singlewire_receive_disable(hal_uart_dev_struct *uart)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    /* disable Rx */
    usart_receive_config(uart->periph, USART_RECEIVE_DISABLE);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable UART to wakeup the mcu from deep-sleep mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_wakeup_enable(hal_uart_dev_struct *uart)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    /* enable wakeup function */
    usart_wakeup_enable(uart->periph);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      disable UART to wakeup the mcu from deep-sleep mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_wakeup_disable(hal_uart_dev_struct *uart)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    /* disable wakeup function */
    usart_wakeup_disable(uart->periph);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      configure UART wakeup mode from deep-sleep mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  wum: wakeup mode
      \arg        UART_WAKEUP_BY_ADDRESS: WUF active on address match
      \arg        UART_WAKEUP_BY_START_BIT: WUF active on start bit
      \arg        UART_WAKEUP_BY_RBNE_SET: WUF active on RBNE
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_wakeup_mode_config(hal_uart_dev_struct *uart, uint32_t wum)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || ((UART_WAKEUP_BY_ADDRESS != wum) && (UART_WAKEUP_BY_START_BIT != wum) \
            && (UART_WAKEUP_BY_RBNE_SET != wum))){
        HAL_DEBUGE("parameter [uart] or [wum] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    usart_wakeup_mode_config(uart->periph, wum);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      configure the UART address detection mode
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  add_mode: address mode
      \arg        UART_MULTIPROCESSOR_ADDRESS_4BIT: 4-bit address detection
      \arg        UART_MULTIPROCESSOR_ADDRESS_FULLBIT: full-bit address detection
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_multiprocessor_address_mode_config(hal_uart_dev_struct *uart, uint32_t add_mode)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((NULL == uart) || ((UART_MULTIPROCESSOR_ADDRESS_4BIT != add_mode) && \
            (UART_MULTIPROCESSOR_ADDRESS_FULLBIT != add_mode))){
        HAL_DEBUGE("parameter [uart] or [add_mode] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    usart_address_detection_mode_config(uart->periph, add_mode);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      set the address of the UART terminal
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  addr: address
      \arg        0x00 - 0xFF
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS details refer to gd32e23x_hal.h
*/
int32_t hal_uart_multiprocessor_address_set(hal_uart_dev_struct *uart, uint8_t addr)
{
    uint8_t uen_flag = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* store the UEN flag */
    uen_flag = USART_CTL0(uart->periph) & USART_CTL0_UEN;
    
    usart_address_config(uart->periph, addr);
    
    /* restore the UEN flag */
    if(RESET != uen_flag){
        usart_enable(uart->periph);
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      configure the UART baudrate and stop bits, the other parameters are configured as default values
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which USART is configured
    \param[in]  baud: baudrate
    \param[in]  stopbits: USART_STB_1BIT, USART_STB_0_5BIT, USART_STB_2BIT, USART_STB_1_5BIT
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h
*/
int32_t hal_uart_simple_config(hal_uart_dev_struct *uart, uint32_t periph, uint32_t baud, uint32_t stopbits)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the parameters */
    if(((USART0 != periph) && (USART1 != periph)) || (0U == baud)){
        HAL_DEBUGE("parameter [periph] or [baud] value is invalid"); 
        return HAL_ERR_VAL;
    }
    
    /* check the parameter */
    if((USART_STB_1BIT != stopbits) && (USART_STB_0_5BIT != stopbits) && (USART_STB_2BIT != stopbits) && \
        (USART_STB_1_5BIT != stopbits)){
        HAL_DEBUGE("parameter [stopbit] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check the tx_state or rx_state wheher is busy or not */
    if((UART_STATE_BUSY == uart->tx_state) || (UART_STATE_BUSY == uart->rx_state)){
        HAL_DEBUGE("uart is in tx or rx state, please wait until run_state change to free ");
        return HAL_ERR_BUSY;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    uart->periph = periph;
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
    \brief      configure the UART baudrate, parity and stop bits
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  data_bits
      \arg        8: 8 data bits
      \arg        9: 9 data bits
    \param[in]  parity
      \arg        0: parity none
      \arg        1: parity odd
      \arg        2: parity even
    \param[in]  stop_bits
      \arg        1: 1 stop bit
      \arg        2: 2 stop bits
    \param[out] none
    \retval     
*/
void hal_uart_format(hal_uart_dev_struct *uart, int data_bits, int parity, int stop_bits)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if(NULL == uart){
        HAL_DEBUGE("parameter [uart] value is invalid");
        return;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable USART */
    usart_disable(uart->periph);
    
    /* configure USART parity */
    switch(parity){
        case 1:
            usart_parity_config(uart->periph, USART_PM_ODD);
            break;
        case 2:
            usart_parity_config(uart->periph, USART_PM_EVEN);
            break;
        case 0:
        default:
            usart_parity_config(uart->periph, USART_PM_NONE);
            break;
    }

    /* configure USART word length */
    if(data_bits == 9){
        usart_word_length_set(uart->periph, USART_WL_9BIT);
    }else{
        usart_word_length_set(uart->periph, USART_WL_8BIT);
    }

    /* configure USART stop bits */
    if(stop_bits == 2){
        usart_stop_bit_set(uart->periph, USART_STB_2BIT);
    }else{
        usart_stop_bit_set(uart->periph, USART_STB_1BIT);
    }

    /* enable USART */
    usart_enable(uart->periph);
}

/*!
    \brief      get the mask of date bit
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     the mask of date bit(0x7F, 0xFF, 0x1FF)
*/
static uint16_t _uart_data_bit_mask_get(hal_uart_dev_struct *uart)
{
    uint16_t reval;
    
    if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_WL)){
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_PCEN)){
            reval = 0xFFU;
        }else{
            reval = 0x1FFU;
        }
    }else{
        /* check whether the PCEN is enabled */
        if(RESET != (USART_CTL0(uart->periph) & USART_CTL0_PCEN)){
            reval = 0x7FU;
        }else{
            reval = 0xFFU;
        }
    }
    
    return reval;
}

/*!
    \brief      get UART error flag
    \param[in]  uart: UART device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     SET or RESET
*/
static FlagStatus _uart_error_flag_get(hal_uart_dev_struct *uart)
{
    if(0U == (USART_STAT(uart->periph) & (uint32_t)(USART_STAT_PERR | USART_STAT_FERR | \
                                                    USART_STAT_ORERR | USART_STAT_NERR))){
        return RESET;
    }else{
        return SET;
    }
}

/*!
    \brief      handle the transmit complete interrupt
    \param[in]  uart: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _uart_transmit_complete_interrupt(void *uart)
{
    hal_uart_dev_struct *p_uart = uart;
    hal_uart_user_cb p_func = (hal_uart_user_cb)p_uart->tx_callback;
    
    /* disable the transmit complete interrupt */
    usart_interrupt_disable(p_uart->periph, USART_INT_TC);
    /* reset transmit_complete_handle and tx_state */
    p_uart->uart_irq.transmit_complete_handle = NULL;
    p_uart->tx_state = UART_STATE_FREE;
    
    if(NULL != p_func){
        /* if there is a user transmit complete callback */
        p_func(p_uart);
    }
}

/*!
    \brief      handle the transmit interrupt
    \param[in]  uart: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _uart_transmit_interrupt(void *uart)
{
    uint32_t temp_val;
    hal_uart_dev_struct *p_uart = uart;
    
    temp_val = p_uart->txbuffer.pos;
    if(temp_val < p_uart->txbuffer.length){
        if((RESET != (USART_CTL0(p_uart->periph) & USART_CTL0_WL)) && \
           (RESET == (USART_CTL0(p_uart->periph) & USART_CTL0_PCEN))){
            /* 9-bit data, none parity */
            usart_data_transmit(p_uart->periph, (*(uint16_t*)p_uart->txbuffer.buffer & (uint16_t)0x1FFU));
            p_uart->txbuffer.buffer += 2U;
        }else{
            /* 9-bit data, with parity or 8-bit data */
            usart_data_transmit(p_uart->periph, (*p_uart->txbuffer.buffer & (uint8_t)0xFFU));
            p_uart->txbuffer.buffer++;
        }
        p_uart->txbuffer.pos++;
    }else{
        /* disable the TBE interrupt, enable the TC interrupt and reset the transmit_ready_handle */
        usart_interrupt_disable(p_uart->periph, USART_INT_TBE);
        usart_interrupt_enable(p_uart->periph, USART_INT_TC);
        p_uart->uart_irq.transmit_ready_handle = NULL;
    }
}

/*!
    \brief      handle the receive interrupt
    \param[in]  uart: pointer to a UART device information structure
    \param[out] none
    \retval     none
*/
static void _uart_receive_interrupt(void *uart)
{
    uint16_t recv_data;
    hal_uart_dev_struct *p_uart = uart;
    uint32_t temp_val;
    
    recv_data = (usart_data_receive(p_uart->periph) & p_uart->data_bit_mask);
    if(0x1FFU == p_uart->data_bit_mask){
        /* store the received data */
        *(uint16_t *)p_uart->rxbuffer.buffer = recv_data;
        p_uart->rxbuffer.buffer += 2U;
    }else{
        /* store the received data */
        *p_uart->rxbuffer.buffer = (uint8_t)recv_data;
        p_uart->rxbuffer.buffer++;
    }
    p_uart->rxbuffer.pos++;
    
    temp_val = p_uart->rxbuffer.pos;
    if(temp_val == p_uart->rxbuffer.length){
        hal_uart_user_cb p_func = (hal_uart_user_cb)p_uart->rx_callback;
        /* disable PERR, ERR, RBNE interrupt */
        usart_interrupt_disable(p_uart->periph, USART_INT_PERR);
        usart_interrupt_disable(p_uart->periph, USART_INT_ERR);
        usart_interrupt_disable(p_uart->periph, USART_INT_RBNE);
        /* reset receive_complete_handle and rx_state */
        p_uart->uart_irq.receive_complete_handle = NULL;
        p_uart->rx_state = UART_STATE_FREE;
        
        if(NULL != p_func){
            /* if there is a user receive complete callback */
            p_func(p_uart);
        }
    }
}

/*!
    \brief      handle the UART DMA transmit process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _uart_transmit_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_uart_dev_struct *p_uart;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_uart = (hal_uart_dev_struct*)p_dma->p_periph;
    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_uart->txbuffer.pos = p_uart->txbuffer.length;
        usart_dma_transmit_config(p_uart->periph, USART_DENT_DISABLE);
        /* enable TC interrupt */
        usart_interrupt_enable(p_uart->periph, USART_INT_TC);
    }
}

/*!
    \brief      handle the UART DMA receive process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _uart_receive_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_uart_dev_struct *p_uart;
    hal_uart_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_uart = (hal_uart_dev_struct*)p_dma->p_periph;
    p_func = (hal_uart_user_cb)p_uart->rx_callback;
    /* DMA normal mode */
    if(RESET == (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        p_uart->rxbuffer.pos = p_uart->rxbuffer.length;
        /* disable DMA receive, PERR and ERR interrupt */
        usart_dma_receive_config(p_uart->periph, USART_DENR_DISABLE);
        usart_interrupt_disable(p_uart->periph, USART_INT_PERR);
        usart_interrupt_disable(p_uart->periph, USART_INT_ERR);
        /* reset rx_state */
        p_uart->rx_state = UART_STATE_FREE;
    }
    
    if(NULL != p_func){
        /* if there is a user receive complete callback */
        p_func(p_uart);
    }
}

/*!
    \brief      handle the UART DMA error process
    \param[in]  dma: pointer to a DMA device information structure
    \param[out] none
    \retval     none
*/
static void _uart_dma_error(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_uart_dev_struct *p_uart;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_uart = (hal_uart_dev_struct*)p_dma->p_periph;

    if(UART_STATE_BUSY == p_uart->tx_state){
        /* transmit state is busy */
        p_uart->error_state |= HAL_USART_ERROR_DMATX;
        p_uart->last_error = HAL_USART_ERROR_DMATX;
        p_uart->txbuffer.pos = p_uart->txbuffer.length;
        /* disable DMA transmit and reset tx_state */
        usart_dma_transmit_config(p_uart->periph, USART_DENT_DISABLE);
        p_uart->tx_state = UART_STATE_FREE;
    }else if(UART_STATE_BUSY == p_uart->rx_state){
        /* receive state is busy */
        p_uart->error_state |= HAL_USART_ERROR_DMARX;
        p_uart->last_error = HAL_USART_ERROR_DMARX;
        p_uart->rxbuffer.pos = p_uart->rxbuffer.length;
        /* disable DMA receive, PERR, ERR interrupt */
        usart_dma_receive_config(p_uart->periph, USART_DENR_DISABLE);
        usart_interrupt_disable(p_uart->periph, USART_INT_PERR);
        usart_interrupt_disable(p_uart->periph, USART_INT_ERR);
        /* reset rx_state */
        p_uart->rx_state = UART_STATE_FREE;
    }else{
        HAL_DEBUGE("uart processor fatal error: dma error exception due to run state");
    }

    if(p_uart->uart_irq.error_handle != NULL){
        /* if there is a user error callback */
        p_uart->uart_irq.error_handle(p_uart);
        p_uart->error_state = HAL_USART_ERROR_NONE;
    }
}
