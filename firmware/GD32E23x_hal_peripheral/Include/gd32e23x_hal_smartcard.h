/*!
    \file    gd32e23x_hal_smartcard.h
    \brief   definitions for the SMARTCARD
    
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

#ifndef GD32E23X_HAL_SMARTCARD_H
#define GD32E23X_HAL_SMARTCARD_H

#include "gd32e23x_hal.h"

/* smartcard transfer buffer structure */
typedef struct {
    __IO uint8_t *buffer;                                /*!< pointer to smartcard transfer buffer */
    __IO uint32_t length;                                /*!< smartcard transfer length */
    __IO uint32_t pos;                                   /*!< smartcard transfer position */
} smartcard_buffer_struct;

/* smartcard device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb receive_complete_handle;           /*!< smartcard receive complete callback function */
    hal_irq_handle_cb transmit_ready_handle;             /*!< smartcard transmit ready callback function */
    hal_irq_handle_cb transmit_complete_handle;          /*!< smartcard transmit complete callback function */
    hal_irq_handle_cb error_handle;                      /*!< smartcard transfer error callback function */
} hal_smartcard_irq_struct;

/* smartcard run state enum */
typedef enum {
    SMARTCARD_STATE_FREE,                                /*!< smartcard ready for use */
    SMARTCARD_STATE_BUSY,                                /*!< smartcard in busy state */
} hal_smartcard_run_state_enum;

/* smartcard device information structure */
typedef struct {
    uint32_t                             periph;         /*!< usart port */
    hal_smartcard_irq_struct             smartcard_irq;  /*!< smartcard device interrupt callback function pointer */
    hal_dma_dev_struct                   *p_dma_rx;      /*!< DMA receive pointer */
    hal_dma_dev_struct                   *p_dma_tx;      /*!< DMA transmit pointer */
    smartcard_buffer_struct              txbuffer;       /*!< transmit buffer */
    smartcard_buffer_struct              rxbuffer;       /*!< receive buffer */
    __IO uint16_t                        last_error;     /*!< the last error code */
    __IO uint16_t                        error_state;    /*!< smartcard error state */
    __IO hal_smartcard_run_state_enum    tx_state;       /*!< transmit state */
    __IO hal_smartcard_run_state_enum    rx_state;       /*!< receive state */
    void                                 *rx_callback;   /*!< receive callback function pointer */
    void                                 *tx_callback;   /*!< transmit callback function pointer */
    void                                 *priv;          /*!< private pointer */
} hal_smartcard_dev_struct;

/* smartcard initialization structure */
typedef struct {
    uint32_t baudrate;                                   /*!< smartcard communication baud rate */
    uint32_t parity;                                     /*!< parity mode */
    uint32_t word_length;                                /*!< number of data bits in a frame */
    uint32_t stop_bit;                                   /*!< number of stop bits */
    uint32_t direction;                                  /*!< communication transfer direction */
    uint32_t clock_polarity;                             /*!< the state of serial colck */
    uint32_t clock_phase;                                /*!< the clock transition on which the bit capture is made */
    uint32_t clock_length_lastbit;                       /*!< whether the clock pulse corresponding to the last transmitted data bit */
    uint32_t prescaler;                                  /*!< smartcard prescaler */
    uint32_t guard_time;                                 /*!< smartcard guard time */
    uint32_t nack_state;                                 /*!< NACK transmission is enabled or disabled */
    uint32_t timeout_enable;                             /*!< the receiver timeout is enabled */
    uint32_t timeout_value;                              /*!< the receiver timeout value */
    uint32_t sample_method;                              /*!< select sample method */
    uint32_t block_length;                               /*!< the smartcard block length in T=1 reception mode */
    uint32_t auto_retry_count;                           /*!< the smartcard auto-retry count */
} hal_smartcard_init_struct;

/* smartcard initialization extend structure */
typedef struct {
    ControlStatus first_bit_msb;                         /*!< MSB is sent first on communication line */
    ControlStatus tx_rx_swap;                            /*!< Tx and Rx pins are swapped */
    ControlStatus rx_level_invert;                       /*!< the Rx pin active level is inverted */
    ControlStatus tx_level_invert;                       /*!< the Tx pin active level is inverted */
    ControlStatus data_bit_invert;                       /*!< data is inverted */
    ControlStatus overrun_disable;                       /*!< the reception overrun detection is disabled */
    ControlStatus rx_error_dma_stop;                     /*!< the DMA is disabled in case of reception error */
} hal_smartcard_init_ex_struct;

/* smartcard structure type enum */
typedef enum {
    HAL_SMARTCARD_INIT_STRUCT,                           /*!< smartcard initialization structure */
    HAL_SMARTCARD_INIT_EX_STRUCT,                        /*!< smartcard initialization extend structure */
    HAL_SMARTCARD_DEV_STRUCT,                            /*!< smartcard device information structure */
    HAL_SMARTCARD_USER_CALLBCAK_STRUCT,                  /*!< smartcard user callback structure */
    HAL_SMARTCARD_IRQ_INIT_STRUCT,                       /*!< smartcard interrupt callback initialization structure */
} hal_smartcard_struct_type_enum;

/* smartcard dma type enum */
typedef enum {
    SMARTCARD_RX_DMA_TYPE,                               /*!< smartcard DMA receive type */
    SMARTCARD_TX_DMA_TYPE,                               /*!< smartcard DMA transmit type */
} hal_smartcard_dma_type_enum;

/* smartcard device user callback function pointer */
typedef void (*hal_smartcard_user_cb)(hal_smartcard_dev_struct *smartcard);

/* smartcard callback structure */
typedef struct{
    hal_smartcard_user_cb complete_func;                 /*!< user-defined transfer complete callback function */
    hal_smartcard_user_cb error_func;                    /*!< user-defined transfer error callback function */
} hal_smartcard_user_callback_struct;

/* smartcard parity */
#define SMARTCARD_PARITY_NONE                  USART_PM_NONE                                      /*!< no parity */
#define SMARTCARD_PARITY_EVEN                  USART_PM_EVEN                                      /*!< even parity */
#define SMARTCARD_PARITY_ODD                   USART_PM_ODD                                       /*!< odd parity */

/* word length */
#define SMARTCARD_WORD_LENGTH_8BIT             USART_WL_8BIT                                      /*!< 8 bits word length */
#define SMARTCARD_WORD_LENGTH_9BIT             USART_WL_9BIT                                      /*!< 9 bits word length */

/* number of stop bits */
#define SMARTCARD_STOP_BIT_0_5                 USART_STB_0_5BIT                                   /*!< 0.5 bit */
#define SMARTCARD_STOP_BIT_1_5                 USART_STB_1_5BIT                                   /*!< 1.5 bits */

/* transfer direction */
#define SMARTCARD_DIRECTION_RX_TX              (USART_TRANSMIT_ENABLE | USART_RECEIVE_ENABLE)     /*!< RX and TX mode */
#define SMARTCARD_DIRECTION_RX_ONLY            (USART_TRANSMIT_DISABLE | USART_RECEIVE_ENABLE)    /*!< RX mode */
#define SMARTCARD_DIRECTION_TX_ONLY            (USART_TRANSMIT_ENABLE | USART_RECEIVE_DISABLE)    /*!< TX mode */

/* clock polarity */
#define SMARTCARD_CLOCK_POLARITY_LOW           USART_CPL_LOW                                      /*!< low polarity */
#define SMARTCARD_CLOCK_POLARITY_HIGH          USART_CPL_HIGH                                     /*!< high polarity */

/* clock phase */
#define SMARTCARD_CLOCK_PHASE_1CK              USART_CPH_1CK                                      /*!< frame phase on first clock transition */
#define SMARTCARD_CLOCK_PHASE_2CK              USART_CPH_2CK                                      /*!< frame phase on second clock transition */

/* last bit */
#define SMARTCARD_LAST_BIT_NOT_OUTPUT          USART_CLEN_NONE                                    /*!< frame last data bit clock pulse not output to SCLK pin */
#define SMARTCARD_LAST_BIT_OUTPUT              USART_CLEN_EN                                      /*!< frame last data bit clock pulse output to SCLK pin */

/* sample method */
#define SMARTCARD_ONE_SAMPLE_BIT               USART_OSB_1BIT                                     /*!< frame one-bit sample method */
#define SMARTCARD_THREE_SAMPLE_BIT             USART_OSB_3BIT                                     /*!< frame three-bit sample method */

/* NACK enable */
#define SMARTCARD_NACK_ENABLE                  USART_CTL2_NKEN                                    /*!< NACK transmission enabled */
#define SMARTCARD_NACK_DISABLE                 0x0U                                               /*!< NACK transmission disabled  */

/* Timeout enable */
#define SMARTCARD_TIMEOUT_DISABLE              0x0U                                               /*!< receiver timeout disabled */
#define SMARTCARD_TIMEOUT_ENABLE               USART_CTL1_RTEN                                    /*!< receiver timeout enabled */

/* SMARTCARD error code */
#define HAL_SMARTCARD_ERROR_NONE               0U                                                 /*!< no error */
#define HAL_SMARTCARD_ERROR_PERR               BIT(0)                                             /*!< parity error */
#define HAL_SMARTCARD_ERROR_NERR               BIT(1)                                             /*!< noise error */
#define HAL_SMARTCARD_ERROR_FERR               BIT(2)                                             /*!< frame error */
#define HAL_SMARTCARD_ERROR_ORERR              BIT(3)                                             /*!< overrun error */
#define HAL_SMARTCARD_ERROR_DMATX              BIT(4)                                             /*!< overrun error */
#define HAL_SMARTCARD_ERROR_DMARX              BIT(5)                                             /*!< overrun error */
#define HAL_SMARTCARD_ERROR_RTF                BIT(11)                                            /*!< Receiver TimeOut error */

/* function declarations */
/* initialize the smartcard structure with the default values */
void hal_smartcard_struct_init(hal_smartcard_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize smartcard */
void hal_smartcard_deinit(hal_smartcard_dev_struct *smartcard);
/* initialize the smartcard with specified values */
int32_t hal_smartcard_init(hal_smartcard_dev_struct *smartcard, uint32_t periph, \
                           hal_smartcard_init_struct *p_init);
/* initialize the smartcard with specified values when using extended functions */
int32_t hal_smartcard_init_ex(hal_smartcard_dev_struct *smartcard, uint32_t periph, \
                              hal_smartcard_init_ex_struct *p_init);
/* handle the smartcard interrupts */
void hal_smartcard_irq(hal_smartcard_dev_struct *smartcard);
/* start smartcard module function */
void hal_smartcard_start(hal_smartcard_dev_struct *smartcard);
/* stop smartcard module function */
void hal_smartcard_stop(hal_smartcard_dev_struct *smartcard);

/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_smartcard_irq_handle_set(hal_smartcard_dev_struct *smartcard, hal_smartcard_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_smartcard_irq_handle_all_reset(hal_smartcard_dev_struct *smartcard);

/* transmit or receive functions */
/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_smartcard_transmit_poll(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                    uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_smartcard_receive_poll(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                   uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data by interrupt method, after the transfer is completed, the user function is called */
/* the function is non-blocking */
int32_t hal_smartcard_transmit_interrupt(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                         uint32_t length, hal_smartcard_user_cb p_user_func);
/* receive amounts of data by interrupt method, after the transfer is completed, the user function is called */
/* the function is non-blocking */
int32_t hal_smartcard_receive_interrupt(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                        uint32_t length, hal_smartcard_user_cb p_user_func);
/* transmit amounts of data by DMA method, after the transfer is completed or error occurs, the user function is called */
/* the function is non-blocking */
int32_t hal_smartcard_transmit_dma(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                   uint32_t length, hal_smartcard_user_callback_struct *p_user_func);
/* receive amounts of data by DMA method, after the transfer is completed or error occurs, the user function is called */
/* the function is non-blocking */
int32_t hal_smartcard_receive_dma(hal_smartcard_dev_struct *smartcard, void *p_buffer, \
                                  uint32_t length, hal_smartcard_user_callback_struct *p_user_func);

/* transfer control functions */
/* stop smartcard transmit transfer */
/* the function is blocking */
int32_t hal_smartcard_transmit_stop(hal_smartcard_dev_struct *smartcard);
/* stop smartcard receive transfer */
/* the function is blocking */
int32_t hal_smartcard_receive_stop(hal_smartcard_dev_struct *smartcard);

/* dynamic update the smartcard block length */
void hal_smartcard_block_length_set(hal_smartcard_dev_struct *smartcard, uint32_t block_length);
/* dynamic update the smartcard receiver timeout value */
void hal_smartcard_receiver_timeout_set(hal_smartcard_dev_struct *smartcard, uint32_t rtimeout);
/* enable the smartcard receiver timeout */
void hal_smartcard_receiver_timeout_enable(hal_smartcard_dev_struct *smartcard);
/* disable the smartcard receiver timeout */
void hal_smartcard_receiver_timeout_disable(hal_smartcard_dev_struct *smartcard);

#endif /* GD32E23X_HAL_SMARTCARD_H */
