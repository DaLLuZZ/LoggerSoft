/*!
    \file    gd32e23x_hal_i2s.h
    \brief   definitions for the I2S
    
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

#ifndef GD32E23X_HAL_I2S_H
#define GD32E23X_HAL_I2S_H
#include "gd32e23x_hal.h"

#define HAL_I2S_ERROR_NONE            0x00000000U  /*!< no error */
#define HAL_I2S_ERROR_TIMEOUT         0x00000001U  /*!< timeout error */
#define HAL_I2S_ERROR_OVERRUN         0x00000002U  /*!< overrun error */
#define HAL_I2S_ERROR_UNDERRUN        0x00000004U  /*!< underrun error */

/* I2S parameter struct definitions */
typedef struct
{
    uint32_t mode;                                 /*!< I2S operation mode */
    uint32_t standard;                             /*!< I2S standard */
    uint32_t frameformat;                          /*!< I2S frame format */
    uint32_t mckout;                               /*!< I2S master clock output */
    uint32_t audiosample;                          /*!< I2S audio sample rate */
    uint32_t ckpl;                                 /*!< I2S idle state clock polarity */
}hal_i2s_init_struct;

/* I2S structure type enum */
typedef enum {
    HAL_I2S_INIT_STRUCT,                           /*!< I2S initialization structure */
    HAL_I2S_DEV_STRUCT,                            /*!< I2S device information structure */
} hal_i2s_struct_type_enum;

/* I2S run state */
typedef enum
{
    HAL_I2S_STATE_RESET      = 0x00U,              /*!< I2S not yet initialized or disabled */
    HAL_I2S_STATE_READY      = 0x01U,              /*!< I2S initialized and ready for use */
    HAL_I2S_STATE_BUSY       = 0x02U,              /*!< I2S internal process is ongoing  */
    HAL_I2S_STATE_BUSY_TX    = 0x03U,              /*!< data transmission process is ongoing */
    HAL_I2S_STATE_BUSY_RX    = 0x04U,              /*!< data reception process is ongoing */
    HAL_I2S_STATE_TIMEOUT    = 0x05U,              /*!< I2S timeout state */
    HAL_I2S_STATE_ERROR      = 0x06U               /*!< I2S error state */
}hal_i2s_run_state_enum;

/* I2S receive or transmit buffer struct definitions */
typedef struct {
    __IO uint16_t *buffer;                         /*!< pointer to I2S transfer buffer*/
    __IO uint16_t length;                          /*!< I2S transfer length */
    __IO uint16_t pos;                             /*!< I2S transfer position */
} i2s_buffer_struct;

/* I2S device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb receive_handler;             /*!< I2S receive complete callback function */
    hal_irq_handle_cb transmit_handler;            /*!< I2S transmit complete callback function */
    hal_irq_handle_cb error_handle;                /*!< I2S error complete callback function */
} hal_i2s_irq_struct;

/* I2S device information structure */
typedef struct
{
    uint32_t                       periph;         /*!< I2S peripheral */
    hal_i2s_irq_struct             i2s_irq;        /*!< I2S device interrupt callback function pointer */
    hal_dma_dev_struct             *p_dma_rx;      /*!< DMA receive pointer */
    hal_dma_dev_struct             *p_dma_tx;      /*!< DMA transmit pointer */
    i2s_buffer_struct              txbuffer;       /*!< transmit buffer */
    i2s_buffer_struct              rxbuffer;       /*!< receive buffer */
    void                           *rx_callback;   /*!< receive callback function pointer */
    void                           *tx_callback;   /*!< transmit callback function pointer */
    void                           *error_callback;/*!< error callback function pointer */
    __IO hal_i2s_run_state_enum    state;          /*!< I2S communication state */
    __IO uint32_t                  error_code;     /*!< I2S error code*/
}hal_i2s_dev_struct; 

/* I2S device user callback function pointer */
typedef void (*hal_i2s_user_cb)(hal_i2s_dev_struct *i2s);

/* I2S callback structure */
typedef struct{
    hal_i2s_user_cb complete_func;                 /*!< I2S user complete callback function */
    hal_i2s_user_cb error_func;                    /*!< I2S user error callback function */
}hal_i2s_user_callback_struct;

/* function declarations */
/* deinitialize I2S */
void hal_i2s_deinit (hal_i2s_dev_struct *i2s);
/* initialize I2S */
int32_t hal_i2s_init(hal_i2s_dev_struct *i2s, uint32_t periph, hal_i2s_init_struct *p_init);
/* initialize I2S structure */
void hal_i2s_struct_init(hal_i2s_struct_type_enum hal_struct_type, void *p_struct);

/* start I2S module function */
void hal_i2s_start(hal_i2s_dev_struct *i2s);
/* stop I2S module function */
void hal_i2s_stop(hal_i2s_dev_struct *i2s);

/* I2S interrupt handler content function,which is merely used in i2s_handler */
void hal_i2s_irq(hal_i2s_dev_struct *i2s);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_i2s_irq_handle_set(hal_i2s_dev_struct *i2s, hal_i2s_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_i2s_irq_handle_all_reset(hal_i2s_dev_struct *i2s);

/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_i2s_transmit_poll(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_i2s_receive_poll(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, uint32_t timeout_ms);

/* transmit amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_i2s_transmit_interrupt(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, hal_i2s_user_callback_struct *p_user_func);
/* receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_i2s_receive_interrupt(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, hal_i2s_user_callback_struct *p_user_func);

/* transmit amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_i2s_transmit_dma(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, hal_i2s_user_callback_struct *p_user_func);
/* receive amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_i2s_receive_dma(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, hal_i2s_user_callback_struct *p_user_func);

/* I2S DMA pause function */
void hal_i2s_dma_pause(hal_i2s_dev_struct *i2s);
/* I2S DMA resume function */
void hal_i2s_dma_resume(hal_i2s_dev_struct *i2s);
/* I2S DMA stop function */
void hal_i2s_dma_stop(hal_i2s_dev_struct *i2s);

#endif /* GD32E23X_HAL_I2S_H */
