/*!
    \file    gd32e23x_hal_usrt.h
    \brief   definitions for the USRT
    
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

#ifndef GD32E23X_HAL_USRT_H
#define GD32E23X_HAL_USRT_H
#include "gd32e23x_hal.h"

/* usrt transfer buffer structure */
typedef struct {
    __IO uint8_t *buffer;           /*!< pointer to transfer buffer */
    __IO uint32_t length;           /*!< transfer length */
    __IO uint32_t pos;              /*!< transfer position */
} usrt_buffer_struct;

/* usrt device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb receive_complete_handle;   /*!< receive complete callback function */  
    hal_irq_handle_cb transmit_ready_handle;     /*!< transmit ready callback function */
    hal_irq_handle_cb transmit_complete_handle;  /*!< transmit complete callback function */
    hal_irq_handle_cb error_handle;              /*!< error callback function */
} hal_usrt_irq_struct;

/* usrt run state enum */
typedef enum {
    USRT_STATE_FREE,                /*!< ready for use */
    USRT_STATE_BUSY,                /*!< busy state */
    USRT_STATE_BUSY_TX_RX,          /*!< busy for Tx Rx state */
} hal_usrt_run_state_enum;

/* usrt device information structure */
typedef struct {
    uint32_t                      periph;        /*!< usart port */
    hal_usrt_irq_struct           usrt_irq;      /*!< device interrupt callback function pointer */
    hal_dma_dev_struct            *p_dma_rx;     /*!< DMA receive pointer */
    hal_dma_dev_struct            *p_dma_tx;     /*!< DMA transmit pointer */
    usrt_buffer_struct            txbuffer;      /*!< transmit buffer */
    usrt_buffer_struct            rxbuffer;      /*!< receive buffer */
    uint16_t                      data_bit_mask; /*!< mask bit of data */
    __IO uint16_t                 last_error;    /*!< the last error code */
    __IO uint16_t                 error_state;   /*!< error state */
    __IO hal_usrt_run_state_enum  tx_state;      /*!< transmit state */
    __IO hal_usrt_run_state_enum  rx_state;      /*!< receive state */
    void                          *rx_callback;  /*!< receive callback function pointer */
    void                          *tx_callback;  /*!< transmit callback function pointer */
    void                          *priv;         /*!< private pointer */
} hal_usrt_dev_struct;

typedef void (*hal_usrt_user_cb)(hal_usrt_dev_struct *usrt);

typedef struct {
    uint32_t baudrate;              /*!< communication baudrate */
    uint32_t parity;                /*!< parity mode */
    uint32_t word_length;           /*!< number of data bits in a frame */
    uint32_t stop_bit;              /*!< number of stop bits */
    uint32_t direction;             /*!< communication transfer direction */
    uint32_t clock_polarity;        /*!< clock polarity */
    uint32_t clock_phase;           /*!< clock phase */
    uint32_t clock_length_lastbit;  /*!< clock length */
} hal_usrt_init_struct;

/* usrt user callback struct */
typedef struct{
    hal_usrt_user_cb complete_func;      /*!< transfer complete callback function */
    hal_usrt_user_cb error_func;         /*!< error callback function */
}hal_usrt_user_callback_struct;

typedef enum {
    HAL_USRT_INIT_STRUCT,           /*!< initialization structure */
    HAL_USRT_DEV_STRUCT,            /*!< device information structure */
    HAL_USRT_USER_CALLBCAK_STRUCT   /*!< user callback struct */
} hal_usrt_struct_type_enum;

/* init struct parameter */
#define USRT_PARITY_NONE             USART_PM_NONE    /*!< no parity */
#define USRT_PARITY_EVEN             USART_PM_EVEN    /*!< even parity */
#define USRT_PARITY_ODD              USART_PM_ODD     /*!< odd parity */

#define USRT_WORD_LENGTH_8BIT        USART_WL_8BIT    /*!< 8 bits word length */
#define USRT_WORD_LENGTH_9BIT        USART_WL_9BIT    /*!< 9 bits word length */

#define USRT_STOP_BIT_1              USART_STB_1BIT   /*!< 1 bit stop bit */
#define USRT_STOP_BIT_2              USART_STB_2BIT   /*!< 2 bits stop bit */
#define USRT_STOP_BIT_1_5            USART_STB_1_5BIT /*!< 1.5 bits stop bit */

#define USRT_DIRECTION_RX_TX         (USART_TRANSMIT_ENABLE | USART_RECEIVE_ENABLE)   /*!< RX and TX mode */
#define USRT_DIRECTION_RX_ONLY       (USART_TRANSMIT_DISABLE | USART_RECEIVE_ENABLE)  /*!< RX only mode */
#define USRT_DIRECTION_TX_ONLY       (USART_TRANSMIT_ENABLE | USART_RECEIVE_DISABLE)  /*!< TX only mode */

#define USRT_CLOCK_POLARITY_LOW      USART_CPL_LOW    /*!< steady low value on CK pin */
#define USRT_CLOCK_POLARITY_HIGH     USART_CPL_HIGH   /*!< steady high value on CK pin */

#define USRT_CLOCK_PHASE_1CK         USART_CPH_1CK    /*!< the first clock transition is the first data capture edge */
#define USRT_CLOCK_PHASE_2CK         USART_CPH_2CK    /*!< the second clock transition is the first data capture edge */

#define USRT_LAST_BIT_NOT_OUTPUT     USART_CLEN_NONE  /*!< the clock pulse of the last data bit (MSB) is not output to the CK pin */
#define USRT_LAST_BIT_OUTPUT         USART_CLEN_EN    /*!< the clock pulse of the last data bit (MSB) is output to the CK pin */

#define USRT_POLARITY_LOW_PHASE_1CK  (USRT_CLOCK_POLARITY_LOW | USRT_CLOCK_PHASE_1CK)  /*!< polarity low and phase on 1st clock */
#define USRT_POLARITY_LOW_PHASE_2CK  (USRT_CLOCK_POLARITY_LOW | USRT_CLOCK_PHASE_2CK)  /*!< polarity low and phase on 2nd clock */
#define USRT_POLARITY_HIGH_PHASE_1CK (USRT_CLOCK_POLARITY_HIGH | USRT_CLOCK_PHASE_1CK) /*!< polarity high and phase on 1st clock */
#define USRT_POLARITY_HIGH_PHASE_2CK (USRT_CLOCK_POLARITY_HIGH | USRT_CLOCK_PHASE_2CK) /*!< polarity high and phase on 2nd clock */

/* USART error code */
#define HAL_USART_ERROR_NONE         0U               /*!< no error */
#define HAL_USART_ERROR_PERR         BIT(0)           /*!< parity error */
#define HAL_USART_ERROR_NERR         BIT(1)           /*!< noise error */
#define HAL_USART_ERROR_FERR         BIT(2)           /*!< frame error */
#define HAL_USART_ERROR_ORERR        BIT(3)           /*!< overrun error */
#define HAL_USART_ERROR_DMATX        BIT(4)           /*!< DMA Tx error */
#define HAL_USART_ERROR_DMARX        BIT(5)           /*!< DMA Rx error */

/* function declarations */
/* initialization functions */
/* initialize the USRT struct with the default values, note that this function must be
called after the struct is created */
void hal_usrt_struct_init(hal_usrt_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize the USRT */
void hal_usrt_deinit(hal_usrt_dev_struct *usrt);
/* initialize the USRT with specified values */
int32_t hal_usrt_init(hal_usrt_dev_struct *usrt, uint32_t periph, \
                      hal_usrt_init_struct *p_init);
/* start USRT module */
void hal_usrt_start(hal_usrt_dev_struct *usrt);
/* stop USRT module */
void hal_usrt_stop(hal_usrt_dev_struct *usrt);

/* USRT interrput handle functions */
/* handle the USRT interrupts */
void hal_usrt_irq(hal_usrt_dev_struct *usrt);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_usrt_irq_handle_set(hal_usrt_dev_struct *usrt, hal_usrt_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_usrt_irq_handle_all_reset(hal_usrt_dev_struct *usrt);

/* transmit or receive functions */
/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_usrt_transmit_poll(hal_usrt_dev_struct *usrt, void *p_buffer, \
                                uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_usrt_receive_poll(hal_usrt_dev_struct *usrt, void *p_buffer, \
                               uint32_t length, uint32_t timeout_ms);
/* transmit & receive amounts of data, poll transfer process and completed status */
/* the function is blocking */
int32_t hal_usrt_transmit_receive_poll(hal_usrt_dev_struct *usrt, void *p_tx_buffer, void *p_rx_buffer, \
                                uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data by interrupt method, after the transfer is completed, 
the user function is called */
/* the function is non-blocking */
int32_t hal_usrt_transmit_interrupt(hal_usrt_dev_struct *usrt, void *p_buffer, \
                                     uint32_t length, hal_usrt_user_cb p_user_func);
/* receive amounts of data by interrupt method, after the transfer is completed, 
the user function is called */
/* the function is non-blocking */
int32_t hal_usrt_receive_interrupt(hal_usrt_dev_struct *usrt, void *p_buffer, \
                                    uint32_t length, hal_usrt_user_cb p_user_func);
/* transmit & receive amounts of data by interrupt method, after the transfer is completed, 
the user function is called */
/* the function is non-blocking */
int32_t hal_usrt_transmit_receive_interrupt(hal_usrt_dev_struct *usrt, void *p_tx_buffer, void *p_rx_buffer, \
                                    uint32_t length, hal_usrt_user_cb p_user_func);
/* transmit amounts of data by DMA method, after the transfer is completed or error occurs, 
the user function is called */
/* the function is non-blocking */
int32_t hal_usrt_transmit_dma(hal_usrt_dev_struct *usrt, void *p_buffer, \
                              uint32_t length, hal_usrt_user_callback_struct *p_func);
/* receive amounts of data by DMA method, after the transfer is completed or error occurs, 
the user function is called */
/* the function is non-blocking */
int32_t hal_usrt_receive_dma(hal_usrt_dev_struct *usrt, void *p_buffer, \
                              uint32_t length, hal_usrt_user_callback_struct *p_func);
/* transmit & receive amounts of data by DMA method, after the transfer is completed or error occurs, 
the user function is called */
/* the function is non-blocking */
int32_t hal_usrt_transmit_receive_dma(hal_usrt_dev_struct *usrt, void *p_tx_buffer, void *p_rx_buffer, \
                                    uint32_t length, hal_usrt_user_callback_struct *p_func);

/* transfer control functions */
/* pause USRT DMA transfer during transmission process */
int32_t hal_usrt_dma_pause(hal_usrt_dev_struct *usrt);
/* resume USRT DMA transfer during transmission process */
int32_t hal_usrt_dma_resume(hal_usrt_dev_struct *usrt);
/* stop USRT transmit and receive transfer */
/* the function is blocking */
int32_t hal_usrt_transfer_stop(hal_usrt_dev_struct *usrt);

/* configure the USRT baudrate and stop bits, the other parameters are configured as default values */
int32_t hal_usrt_simple_config(hal_usrt_dev_struct *usrt, uint32_t periph, \
                              uint32_t baud, uint32_t stopbits); 
/* configure the USRT clock polarity, phase and last bit */
int32_t hal_usrt_simple_clock_config(hal_usrt_dev_struct *usrt, uint32_t periph, \
                              uint32_t clock_polarity_phase, uint32_t clock_lastbit);

#endif
