/*!
    \file    gd32e23x_hal_uart.h
    \brief   definitions for the UART
    
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


#ifndef GD32E23X_HAL_UART_H
#define GD32E23X_HAL_UART_H
#include "gd32e23x_hal.h"

/* uart work mode enum */
typedef enum {
    UART_WORK_MODE_ASYN = 0,        /*!< asynchronous communication mode */
    UART_WORK_MODE_SINGLE_WIRE,     /*!< single wire(half-duplex) communication mode */
    UART_WORK_MODE_MULTIPROCESSCOR, /*!< multiprocessor communication mode */
    UART_WORK_MODE_LIN,             /*!< LIN mode */
    UART_WORK_MODE_RS485,           /*!< RS485 mode */
}hal_uart_work_mode_enum;

/* uart transfer buffer structure */
typedef struct {
    __IO uint8_t *buffer;           /*!< pointer to transfer buffer */
    __IO uint32_t length;           /*!< transfer length */
    __IO uint32_t pos;              /*!< transfer position */
} uart_buffer_struct;

/* uart device interrupt callback function pointer structure */
typedef struct {
    __IO hal_irq_handle_cb receive_complete_handle;   /*!< receive complete callback function */  
    __IO hal_irq_handle_cb transmit_ready_handle;     /*!< transmit ready callback function */
    __IO hal_irq_handle_cb transmit_complete_handle;  /*!< transmit complete callback function */
    __IO hal_irq_handle_cb error_handle;              /*!< error callback function */
    __IO hal_irq_handle_cb wakeup_handle;             /*!< wakeup callback function */
    __IO hal_irq_handle_cb idle_line_detected_handle; /*!< idle line detected callback function */
    __IO hal_irq_handle_cb address_match_handle;      /*!< address match callback function */
    __IO hal_irq_handle_cb lin_break_detected_handle; /*!< LIN break detected callback function */
    __IO hal_irq_handle_cb cts_change_handle;         /*!< CTS change callback function */
} hal_uart_irq_struct;

/* uart run state enum */
typedef enum {
    UART_STATE_FREE,                             /*!< ready for use */
    UART_STATE_BUSY,                             /*!< busy state */
} hal_uart_run_state_enum;

/* uart device information structure */
typedef struct {
    uint32_t                      periph;        /*!< usart port */
    hal_uart_irq_struct           uart_irq;      /*!< device interrupt callback function pointer */
    hal_dma_dev_struct            *p_dma_rx;     /*!< DMA receive pointer */
    hal_dma_dev_struct            *p_dma_tx;     /*!< DMA transmit pointer */
    uart_buffer_struct            txbuffer;      /*!< transmit buffer */
    uart_buffer_struct            rxbuffer;      /*!< receive buffer */
    hal_uart_work_mode_enum       work_mode;     /*!< work mode */
    uint16_t                      data_bit_mask; /*!< mask bit of data */
    __IO uint16_t                 last_error;    /*!< the last error code */
    __IO uint16_t                 error_state;   /*!< error state */
    __IO hal_uart_run_state_enum  tx_state;      /*!< transmit state */
    __IO hal_uart_run_state_enum  rx_state;      /*!< receive state */
    void                          *rx_callback;  /*!< receive callback function pointer */
    void                          *tx_callback;  /*!< transmit callback function pointer */
    void                          *priv;         /*!< private pointer */
} hal_uart_dev_struct;

typedef void (*hal_uart_user_cb)(hal_uart_dev_struct *uart);

/* uart LIN mode struct */
typedef struct {
    uint32_t break_frame_length;   /*!< LIN break frame length */
} _uart_lin_mode_struct;

/* uart RS485 mode struct */
typedef struct {
    uint32_t de_polarity;          /*!< driver enable polarity */
    uint32_t de_assertion_time;    /*!< driver enable assertion time */
    uint32_t de_deassertion_time;  /*!< driver enable deassertion time */
} _uart_rs485_mode_struct;

/* uart multi-processor mode struct */
typedef struct {
    uint32_t wakeup_mode;          /*!< wakeup mode */
    uint8_t address;               /*!< wakeup address */
    uint32_t addr_length;          /*!< address length */
} _uart_multiprocessor_mode_struct;

/* baudrate detection mode struct */
typedef struct {
    ControlStatus use;             /*!< use control */
    uint32_t detection_mode;       /*!< auto baudrate detection mode */
} _autobaud_detection_struct;

/* wakeup from deep-sleep mode struct */
typedef struct {
    ControlStatus use;             /*!< use control */
    uint32_t wakeup_by;            /*!< wakeup by which event from deep-sleep mode */
} _wakeup_from_deepsleep_struct;

/* uart initialization structure */
typedef struct {
    hal_uart_work_mode_enum work_mode;  /*!< work mode */
    uint32_t baudrate;                  /*!< communication baudrate */
    uint32_t parity;                    /*!< parity mode */
    uint32_t word_length;               /*!< number of data bits in a frame */
    uint32_t stop_bit;                  /*!< number of stop bits */
    uint32_t over_sample;               /*!< oversample mode */
    uint32_t sample_method;             /*!< one sample bit method */
    uint32_t direction;                 /*!< communication transfer direction */
    uint32_t hardware_flow;             /*!< hardware flow control */
    
    _uart_lin_mode_struct lin_mode;     /*!< LIN mode */
    _uart_rs485_mode_struct rs485_mode; /*!< RS485 mode */
    _uart_multiprocessor_mode_struct multiprocessor_mode; /*!< multi-processor mode */
} hal_uart_init_struct;

/* uart initialization extend structure */
typedef struct {
    ControlStatus first_bit_msb;         /*!< MSB is sent first on communication line */
    ControlStatus tx_rx_swap;            /*!< Tx and Rx pins are swapped */
    ControlStatus rx_level_invert;       /*!< the Rx pin active level is inverted */
    ControlStatus tx_level_invert;       /*!< the Tx pin active level is inverted */
    ControlStatus data_bit_invert;       /*!< data is inverted */
    ControlStatus overrun_disable;       /*!< the reception overrun detection is disabled */
    ControlStatus rx_error_dma_stop;     /*!< the DMA is disabled in case of reception error */
    
    _autobaud_detection_struct autobaud; /*!< auto baudrate detection */
    _wakeup_from_deepsleep_struct wakeup;/*!< wakeup mode */
} hal_uart_init_ex_struct;

/* uart user callback struct */
typedef struct{
    hal_uart_user_cb complete_func;      /*!< transfer complete callback function */
    hal_uart_user_cb error_func;         /*!< error callback function */
}hal_uart_user_callback_struct;

/* uart struct initialization type enum */
typedef enum {
    HAL_UART_INIT_STRUCT,                /*!< initialization structure */
    HAL_UART_INIT_EX_STRUCT,             /*!< initialization extend structure */
    HAL_UART_DEV_STRUCT,                 /*!< device information structure */
    HAL_UART_USER_CALLBCAK_STRUCT,       /*!< user callback struct */
    HAL_UART_IRQ_INIT_STRUCT             /*!< interrupt callback initialization structure */
} hal_uart_struct_type_enum;

/* uart command type enum */
typedef enum{
    HAL_UART_CMD_ENTER_MUTE         = USART_CMD_MMCMD,  /*!< mute mode command */
    HAL_UART_CMD_SEND_BREAK         = USART_CMD_SBKCMD, /*!< send break command */
    HAL_UART_CMD_AUTOBAUD_DETECTION = USART_CMD_ABDCMD  /*!< auto baudrate detection command */
}hal_uart_command_enum;

/* initialization struct parameters */
#define UART_PARITY_NONE            USART_PM_NONE     /*!< no parity */
#define UART_PARITY_EVEN            USART_PM_EVEN     /*!< even parity */
#define UART_PARITY_ODD             USART_PM_ODD      /*!< odd parity */

#define UART_WORD_LENGTH_8BIT       USART_WL_8BIT     /*!< 8 bits word length */
#define UART_WORD_LENGTH_9BIT       USART_WL_9BIT     /*!< 9 bits word length */

#define UART_OVER_SAMPLE_8          USART_OVSMOD_8    /*!< oversampling by 8 */
#define UART_OVER_SAMPLE_16         USART_OVSMOD_16   /*!< oversampling by 16 */

#define UART_THREE_SAMPLE_BIT       USART_OSB_3BIT    /*!< three sample bit method */
#define UART_ONE_SAMPLE_BIT         USART_OSB_1BIT    /*!< one sample bit method */

#define UART_STOP_BIT_1             USART_STB_1BIT    /*!< 1 bit stop bit */
#define UART_STOP_BIT_0_5           USART_STB_0_5BIT  /*!< 0.5 bit stop bit */
#define UART_STOP_BIT_2             USART_STB_2BIT    /*!< 2 bits stop bit */
#define UART_STOP_BIT_1_5           USART_STB_1_5BIT  /*!< 1.5 bits stop bit */

#define UART_DIRECTION_RX_TX        (USART_TRANSMIT_ENABLE | USART_RECEIVE_ENABLE)  /*!< RX and TX mode */
#define UART_DIRECTION_RX_ONLY      (USART_TRANSMIT_DISABLE | USART_RECEIVE_ENABLE) /*!< RX only mode */
#define UART_DIRECTION_TX_ONLY      (USART_TRANSMIT_ENABLE | USART_RECEIVE_DISABLE) /*!< TX only mode */

#define UART_MULTIPROCESSOR_WAKEUP_IDLE        USART_WM_IDLE      /*!< wakeup from deep-sleep by idle line */
#define UART_MULTIPROCESSOR_WAKEUP_ADDRESS     USART_WM_ADDR      /*!< wakeup from deep-sleep by address mark */

#define UART_MULTIPROCESSOR_ADDRESS_4BIT       USART_ADDM_4BIT    /*!< 4-bit address detection */
#define UART_MULTIPROCESSOR_ADDRESS_FULLBIT    USART_ADDM_FULLBIT /*!< full-bit address detection */

#define UART_LIN_BREAK_DETECTION_10BIT         USART_LBLEN_10B    /*!< 10 bits break detection */
#define UART_LIN_BREAK_DETECTION_11BIT         USART_LBLEN_11B    /*!< 11 bits break detection */

#define UART_RS485_DE_POLARITY_HIGH            USART_DEP_HIGH     /*!< DE signal is active high */
#define UART_RS485_DE_POLARITY_LOW             USART_DEP_LOW      /*!< DE signal is active low */

#define UART_HARDWARE_FLOW_NONE                (USART_RTS_DISABLE | USART_CTS_DISABLE) /*!< hardware flow none */
#define UART_HARDWARE_FLOW_RTS_ONLY            (USART_RTS_ENABLE | USART_CTS_DISABLE)  /*!< RTS only */
#define UART_HARDWARE_FLOW_CTS_ONLY            (USART_RTS_DISABLE | USART_CTS_ENABLE)  /*!< CTS only */
#define UART_HARDWARE_FLOW_RTS_CTS             (USART_RTS_ENABLE | USART_CTS_ENABLE)   /*!< RTS and CTS */

/* init ex struct parameter */
#define UART_WAKEUP_BY_ADDRESS                 USART_WUM_ADDR     /*!< WUF active on address match */
#define UART_WAKEUP_BY_START_BIT               USART_WUM_STARTB   /*!< WUF active on start bit */
#define UART_WAKEUP_BY_RBNE_SET                USART_WUM_RBNE     /*!< WUF active on RBNE */

#define UART_AUTOBAUD_BY_BIT_SEQUENCE_1XXXXXXX USART_ABDM_FTOR    /*!< falling edge to rising edge measurement */
#define UART_AUTOBAUD_BY_BIT_SEQUENCE_10XXXXXX USART_ABDM_FTOF    /*!< falling edge to falling edge measurement */

/* USART error code */
#define HAL_USART_ERROR_NONE                    0U                /*!< no error */
#define HAL_USART_ERROR_PERR                    BIT(0)            /*!< parity error */
#define HAL_USART_ERROR_NERR                    BIT(1)            /*!< noise error */
#define HAL_USART_ERROR_FERR                    BIT(2)            /*!< frame error */
#define HAL_USART_ERROR_ORERR                   BIT(3)            /*!< overrun error */
#define HAL_USART_ERROR_DMATX                   BIT(4)            /*!< DMA Tx error */
#define HAL_USART_ERROR_DMARX                   BIT(5)            /*!< DMA Rx error */

/* function declarations */
/* initialization functions */
/* initialize the UART struct with the default values, note that this function must be
called after the struct is created */
void hal_uart_struct_init(hal_uart_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize the UART */
void hal_uart_deinit(hal_uart_dev_struct *uart);
/* initialize the UART with specified values */
int32_t hal_uart_init(hal_uart_dev_struct *uart, uint32_t periph, \
                      hal_uart_init_struct *p_init);
/* initialize the UART with specified values when using extended functions */
int32_t hal_uart_init_ex(hal_uart_dev_struct *uart, uint32_t periph, \
                      hal_uart_init_ex_struct *p_init);
/* start UART module */
void hal_uart_start(hal_uart_dev_struct *uart);
/* stop UART module */
void hal_uart_stop(hal_uart_dev_struct *uart);

/* UART interrput handle functions */
/* handle the UART interrupts */
void hal_uart_irq(hal_uart_dev_struct *uart);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_uart_irq_handle_set(hal_uart_dev_struct *uart, hal_uart_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_uart_irq_handle_all_reset(hal_uart_dev_struct *uart);

/* transmit or receive functions */
/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_uart_transmit_poll(hal_uart_dev_struct *uart, void *p_buffer, \
                                uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_uart_receive_poll(hal_uart_dev_struct *uart, void *p_buffer, \
                               uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data by interrupt method, after the transfer is completed, 
the user function is called */
/* the function is non-blocking */
int32_t hal_uart_transmit_interrupt(hal_uart_dev_struct *uart, void *p_buffer, \
                                     uint32_t length, hal_uart_user_cb p_user_func);
/* receive amounts of data by interrupt method, after the transfer is completed, 
the user function is called */
/* the function is non-blocking */
int32_t hal_uart_receive_interrupt(hal_uart_dev_struct *uart, void *p_buffer, \
                                    uint32_t length, hal_uart_user_cb p_user_func);
/* transmit amounts of data by DMA method, after the transfer is completed or error occurs, 
the user function is called */
/* the function is non-blocking */
int32_t hal_uart_transmit_dma(hal_uart_dev_struct *uart, void *p_buffer, \
                              uint32_t length, hal_uart_user_callback_struct *p_func);
/* receive amounts of data by DMA method, after the transfer is completed or error occurs, 
the user function is called */
/* the function is non-blocking */
int32_t hal_uart_receive_dma(hal_uart_dev_struct *uart, void *p_buffer, \
                              uint32_t length, hal_uart_user_callback_struct *p_func);

/* transfer control functions */
/* pause UART DMA transfer during transmission process */
int32_t hal_uart_dma_pause(hal_uart_dev_struct *uart);
/* resume UART DMA transfer during transmission process */
int32_t hal_uart_dma_resume(hal_uart_dev_struct *uart);
/* stop UART transmit transfer */
/* the function is blocking */
int32_t hal_uart_transmit_stop(hal_uart_dev_struct *uart);
/* stop UART receive transfer */
/* the function is blocking */
int32_t hal_uart_receive_stop(hal_uart_dev_struct *uart);

/* UART configuration functions */
/* enable the UART works in which mode */
int32_t hal_uart_work_mode_enable(hal_uart_dev_struct *uart, hal_uart_work_mode_enum mode);
/* disable the UART work mode */
int32_t hal_uart_work_mode_disable(hal_uart_dev_struct *uart, hal_uart_work_mode_enum mode);
/* enable the UART command */
int32_t hal_uart_command_enable(hal_uart_dev_struct *uart, hal_uart_command_enum cmd);

/* configuration functions in single wire(half-duplex) mode */
/* enable transmit in single wire(half-duplex) mode */
int32_t hal_uart_singlewire_transmit_enable(hal_uart_dev_struct *uart);
/* disable transmit in single wire(half-duplex) mode */
int32_t hal_uart_singlewire_transmit_disable(hal_uart_dev_struct *uart);
/* enable receive in single wire(half-duplex) mode */
int32_t hal_uart_singlewire_receive_enable(hal_uart_dev_struct *uart);
/* disable receive in single wire(half-duplex) mode */
int32_t hal_uart_singlewire_receive_disable(hal_uart_dev_struct *uart);

/* configuration functions in wakeup mode */
/* enable UART to wakeup the mcu from deep-sleep mode */
int32_t hal_uart_wakeup_enable(hal_uart_dev_struct *uart);
/* disable UART to wakeup the mcu from deep-sleep mode */
int32_t hal_uart_wakeup_disable(hal_uart_dev_struct *uart);
/* configure UART wakeup mode from deep-sleep mode */
int32_t hal_uart_wakeup_mode_config(hal_uart_dev_struct *uart, uint32_t wum);

/* configuration functions in multi-processor mode */
/* configure the UART address detection mode in multi-processor mode */
int32_t hal_uart_multiprocessor_address_mode_config(hal_uart_dev_struct *uart, uint32_t add_mod);
/* set the address of the UART terminal in multi-processor mode */
int32_t hal_uart_multiprocessor_address_set(hal_uart_dev_struct *uart, uint8_t addr);

/* configure the UART baudrate and stop bits, the other parameters are configured as default values */
int32_t hal_uart_simple_config(hal_uart_dev_struct *uart, uint32_t periph, \
                              uint32_t baud, uint32_t stopbits);
/* configure the UART baudrate, parity and stop bits */
void hal_uart_format(hal_uart_dev_struct *uart, int data_bits, int parity, int stop_bits);

#endif /* GD32E23X_HAL_UART_H */
