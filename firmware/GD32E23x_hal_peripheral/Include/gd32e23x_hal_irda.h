/*!
    \file    gd32e23x_hal_irda.h
    \brief   definitions for the IRDA
    
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

#ifndef GD32E23X_HAL_IRDA_H
#define GD32E23X_HAL_IRDA_H
#include "gd32e23x_hal.h"

/* IRAD transfer buffer structure */
typedef struct {
    __IO uint8_t *buffer;                        /*!< pointer to transfer buffer */
    __IO uint32_t length;                        /*!< transfer length */
    __IO uint32_t pos;                           /*!< transfer position */
} irda_buffer_struct;

/* IrDA device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb receive_complete_handle;   /*!< receive complete callback function */
    hal_irq_handle_cb transmit_ready_handle;     /*!< transmit ready callback function */
    hal_irq_handle_cb transmit_complete_handle;  /*!< transmit complete callback function */
    hal_irq_handle_cb error_handle;              /*!< error callback function */
} hal_irda_irq_struct;

/* IrDA run state enum */
typedef enum {
    IRDA_STATE_FREE,                             /*!< ready for use */
    IRDA_STATE_BUSY,                             /*!< busy state */
} hal_irda_run_state_enum;

/* IrDA device information structure */
typedef struct {
    uint32_t                      periph;        /*!< usart port */
    hal_irda_irq_struct           irda_irq;      /*!< device interrupt callback function pointer */
    hal_dma_dev_struct            *p_dma_rx;     /*!< DMA receive pointer */
    hal_dma_dev_struct            *p_dma_tx;     /*!< DMA transmit pointer */
    irda_buffer_struct            txbuffer;      /*!< transmit buffer */
    irda_buffer_struct            rxbuffer;      /*!< receive buffer */
    uint16_t                      data_bit_mask; /*!< mask bit of data */
    __IO uint16_t                 last_error;    /*!< the last error code */
    __IO uint16_t                 error_state;   /*!< error state */
    __IO hal_irda_run_state_enum  tx_state;      /*!< transmit state */
    __IO hal_irda_run_state_enum  rx_state;      /*!< receive state */
    void                          *rx_callback;  /*!< receive callback function pointer */
    void                          *tx_callback;  /*!< transmit callback function pointer */
    void                          *priv;         /*!< private pointer */
} hal_irda_dev_struct;

typedef void (*hal_irda_user_cb)(hal_irda_dev_struct *irda);

/* IrDA initialization structure */
typedef struct {
    uint32_t baudrate;                  /*!< communication baudrate */
    uint32_t parity;                    /*!< parity mode */
    uint32_t word_length;               /*!< number of data bits in a frame */
    uint32_t direction;                 /*!< communication transfer direction */
    uint32_t mode;                      /*!< power mode */
    uint8_t prescaler;                  /*!< prescaler */
} hal_irda_init_struct;

/* IrDA user callback struct */
typedef struct{
    hal_irda_user_cb complete_func;     /*!< transfer complete callback function */
    hal_irda_user_cb error_func;        /*!< error callback function */
}hal_irda_user_callback_struct;

/* IrDA struct initialization type enum */
typedef enum {
    HAL_IRDA_INIT_STRUCT,               /*!< initialization structure */
    HAL_IRDA_DEV_STRUCT,                /*!< device information structure */
    HAL_IRDA_USER_CALLBCAK_STRUCT       /*!< user callback struct */
} hal_irda_struct_type_enum;

#define IRDA_PARITY_NONE               USART_PM_NONE     /*!< no parity */
#define IRDA_PARITY_EVEN               USART_PM_EVEN     /*!< even parity */
#define IRDA_PARITY_ODD                USART_PM_ODD      /*!< odd parity */

#define IRDA_WORD_LENGTH_8BIT          USART_WL_8BIT     /*!< 8 bits word length */
#define IRDA_WORD_LENGTH_9BIT          USART_WL_9BIT     /*!< 9 bits word length */

#define IRDA_DIRECTION_RX_TX           (USART_TRANSMIT_ENABLE | USART_RECEIVE_ENABLE)  /*!< RX and TX mode */
#define IRDA_DIRECTION_RX_ONLY         (USART_TRANSMIT_DISABLE | USART_RECEIVE_ENABLE) /*!< RX only mode */
#define IRDA_DIRECTION_TX_ONLY         (USART_TRANSMIT_ENABLE | USART_RECEIVE_DISABLE) /*!< TX only mode */

#define IRDA_NORMAL_MODE               USART_IRLP_NORMAL /* normal mode */
#define IRDA_LOW_POWER_MODE            USART_IRLP_LOW    /* low-power mode */

/* USART error code */
#define HAL_USART_ERROR_NONE           0U                /*!< no error */
#define HAL_USART_ERROR_PERR           BIT(0)            /*!< parity error */
#define HAL_USART_ERROR_NERR           BIT(1)            /*!< noise error */
#define HAL_USART_ERROR_FERR           BIT(2)            /*!< frame error */
#define HAL_USART_ERROR_ORERR          BIT(3)            /*!< overrun error */
#define HAL_USART_ERROR_DMATX          BIT(4)            /*!< DMA Tx error */
#define HAL_USART_ERROR_DMARX          BIT(5)            /*!< DMA Rx error */

/* function declarations */
/* initialization functions */
/* initialize the IrDA struct with the default values, note that this function must be
called after the struct is created */
void hal_irda_struct_init(hal_irda_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize the IrDA */
void hal_irda_deinit(hal_irda_dev_struct *irda);
/* initialize the IrDA with specified values */
int32_t hal_irda_init(hal_irda_dev_struct *irda, uint32_t periph, \
                      hal_irda_init_struct *p_init);
/* start IrDA module */
void hal_irda_start(hal_irda_dev_struct *irda);
/* stop IrDA module */
void hal_irda_stop(hal_irda_dev_struct *irda);

/* IrDA interrput handle functions */
/* handle the IrDA interrupts */
void hal_irda_irq(hal_irda_dev_struct *irda);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_irda_irq_handle_set(hal_irda_dev_struct *irda, hal_irda_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_irda_irq_handle_all_reset(hal_irda_dev_struct *irda);

/* transmit or receive functions */
/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_irda_transmit_poll(hal_irda_dev_struct *irda, void *p_buffer, \
                                uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_irda_receive_poll(hal_irda_dev_struct *irda, void *p_buffer, \
                               uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data by interrupt method, after the transfer is completed, 
the user function is called */
/* the function is non-blocking */
int32_t hal_irda_transmit_interrupt(hal_irda_dev_struct *irda, void *p_buffer, \
                                     uint32_t length, hal_irda_user_cb p_user_func);
/* receive amounts of data by interrupt method, after the transfer is completed, 
the user function is called */
/* the function is non-blocking */
int32_t hal_irda_receive_interrupt(hal_irda_dev_struct *irda, void *p_buffer, \
                                    uint32_t length, hal_irda_user_cb p_user_func);
/* transmit amounts of data by DMA method, after the transfer is completed or error occurs, 
the user function is called */
/* the function is non-blocking */
int32_t hal_irda_transmit_dma(hal_irda_dev_struct *irda, void *p_buffer, \
                              uint32_t length, hal_irda_user_callback_struct *p_func);
/* receive amounts of data by DMA method, after the transfer is completed or error occurs, 
the user function is called */
/* the function is non-blocking */
int32_t hal_irda_receive_dma(hal_irda_dev_struct *irda, void *p_buffer, \
                              uint32_t length, hal_irda_user_callback_struct *p_func);

/* transfer control functions */
/* pause IrDA DMA transfer during transmission process */
int32_t hal_irda_dma_pause(hal_irda_dev_struct *irda);
/* resume IrDA DMA transfer during transmission process */
int32_t hal_irda_dma_resume(hal_irda_dev_struct *irda);
/* stop IrDA transmit transfer */
/* the function is blocking */
int32_t hal_irda_transmit_stop(hal_irda_dev_struct *irda);
/* stop IrDA receive transfer */
/* the function is blocking */
int32_t hal_irda_receive_stop(hal_irda_dev_struct *irda);

/* configure the IrDA baudrate and prescaler, the other parameters are configured as default values */
int32_t hal_irda_simple_config(hal_irda_dev_struct *irda, uint32_t periph, \
                              uint32_t baud, uint8_t psc);

#endif /* GD32E23X_HAL_IRDA_H */
