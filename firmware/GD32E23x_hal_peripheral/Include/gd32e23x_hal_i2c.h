/*!
    \file    gd32e23x_hal_i2c.h
    \brief   definitions for the I2C

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

#ifndef GD32E23X_HAL_I2C_H
#define GD32E23X_HAL_I2C_H
#include "gd32e23x_hal.h"

typedef enum {
    HAL_I2C_INIT_STRUCT,        /*!< I2C initialize structure */
    HAL_I2C_DEV_STRUCT,         /*!< I2C device information structure */
    HAL_I2C_IRQ_STRUCT,         /*!< I2C device interrupt callback function pointer structure */
} hal_i2c_struct_type_enum;

typedef struct {
    uint32_t duty_cycle;        /*!< duty cycle in fast mode or fast mode plus */
    uint32_t clock_speed;       /*!< I2C clock speed */
    uint32_t address_format;    /*!< I2C addformat, 7bits or 10bits */
    uint32_t own_address1;      /*!< I2C own address */
    uint32_t dual_address;      /*!< dual-address mode switch */
    uint32_t own_address2;      /*!< I2C own address2 in dual-address mode */
    uint32_t general_call;      /*!< whether or not to response to a general call */
    uint32_t no_stretch;        /*!< whether to stretch SCL low when data is not ready in slave mode */
}hal_i2c_init_struct;

#define I2C_BUSY_TIMEOUT                    ((uint32_t)10000)       /* 10s timeout */
#define I2C_TIMEOUT                         ((uint32_t)100000)      /* 100s timeout */
#define I2C_ADDR_TIMEOUT                    ((uint32_t)10000)       /* 10s timeout */

/* I2C error state */
#define HAL_I2C_ERROR_NONE                  0U                      /*!< no error */
#define HAL_I2C_ERROR_BERR                  BIT(0)                  /*!< bus error */
#define HAL_I2C_ERROR_LOSTARB               BIT(1)                  /*!< arbitration lost in master mode */
#define HAL_I2C_ERROR_AERR                  BIT(2)                  /*!< acknowledge error */
#define HAL_I2C_ERROR_OUERR                 BIT(3)                  /*!< over-run error */
#define HAL_I2C_ERROR_PECERR                BIT(4)                  /*!< PEC error */
#define HAL_I2C_ERROR_DMATX                 BIT(7)                  /*!< DMA TX error */
#define HAL_I2C_ERROR_DMARX                 BIT(8)                  /*!< DMA RX error */

/* I2C previous state */
#define HAL_I2C_PREVIOUS_STATE_NONE         0U                      /*!< default value */
#define HAL_I2C_PREVIOUS_STATE_TX           BIT(0)                  /*!< the last communication is TX */
#define HAL_I2C_PREVIOUS_STATE_RX           BIT(1)                  /*!< the last communication is RX */

/* I2C dual-address mode switch */
#define I2C_DUADEN_DISABLE                  ((uint32_t)0x00000000U) /*!< dual-address mode disabled */
#define I2C_DUADEN_ENABLE                   ((uint32_t)0x00000001U) /*!< dual-address mode enabled */

/* I2C frame flag in serial transfer */
#define I2C_NO_OPTION_TRANSFER              ((uint16_t)0x0001U)     /*!< there is only one frame in serial transfer */
#define I2C_FIRST_TRANSFER                  ((uint16_t)0x0002U)     /*!< first frame in serial transfer */
#define I2C_LAST_TRANSFER                   ((uint16_t)0x0003U)     /*!< last frame in serial transfer */
#define I2C_NEXT_TRANSFER                   ((uint16_t)0x0004U)     /*!< next frame in serial transfer, there are at least there frames in serial transfer */

#define I2C_MEMORY_ADDRESS_8BIT             ((uint16_t)0x0000U)     /*!< memory address is 8 bits */
#define I2C_MEMORY_ADDRESS_16BIT            ((uint32_t)0x0001U)     /*!< memory address is 16 bits */

typedef struct {
    __IO uint8_t *buffer;                               /*!< pointer to transfer buffer */
    __IO uint32_t length;                               /*!< transfer length */
    __IO uint32_t pos;                                  /*!< transfer position */
} i2c_buffer_struct;

typedef struct {
    hal_irq_handle_cb event_handle;                     /*!< event callback function */
    hal_irq_handle_cb error_handle;                     /*!< error callback function */
    hal_irq_handle_cb rxframe_rise_handle;              /*!< rxframe rise callback function */
    hal_irq_handle_cb rxframe_fall_handle;              /*!< rxframe fall callback function */
    hal_irq_handle_cb txframe_rise_handle;              /*!< txframe rise callback function */
    hal_irq_handle_cb txframe_fall_handle;              /*!< txframe fall callback function */
} hal_i2c_irq_struct;

typedef enum {
    I2C_STATE_READY,                                    /*!< I2C is ready for use */
    I2C_STATE_BUSY,                                     /*!< I2C transfer process is ongoing */
    I2C_STATE_MEMORY_BUSY_TX,                           /*!< I2C write memory process is ongoing */
    I2C_STATE_MEMORY_BUSY_RX,                           /*!< I2C read memory process is ongoing */
    I2C_STATE_LISTEN,                                   /*!< I2C addressing listen is ongoing in slave mode */
    I2C_STATE_BUSY_LISTEN                               /*!< I2C addressing listen and data transfer is ongoing */
} hal_i2c_run_state_enum;

typedef struct{
    uint32_t                        device_address;     /*!< device address */
    uint16_t                        memory_address;     /*!< memory address */
    uint16_t                        address_size;       /*!< memory address size */
    FlagStatus                      address_complete;   /*!< addressing complete flag, initialize to RESET */
    uint8_t                         address_count;      /*!< address count, initialize to 0 */
    FlagStatus                      second_addressing;  /*!< initialize to RESET */
} hal_i2c_slave_address_struct;

typedef struct {
    uint32_t                        periph;             /*!< I2C port */
    hal_i2c_irq_struct              i2c_irq;            /*!< device interrupt callback function pointer */
    hal_dma_dev_struct              *p_dma_rx;          /*!< DMA receive pointer */
    hal_dma_dev_struct              *p_dma_tx;          /*!< DMA transmit pointer */
    i2c_buffer_struct               txbuffer;           /*!< transmit buffer */
    i2c_buffer_struct               rxbuffer;           /*!< receive buffer */
    hal_i2c_slave_address_struct    slave_address;      /*!< slave address */
    __IO uint16_t                   transfer_option;    /*!< transfer option */
    __IO uint16_t                   last_error;         /*!< the last error code */
    __IO uint32_t                   error_state;        /*!< error state */
    __IO hal_i2c_run_state_enum     tx_state;           /*!< transmit state */
    __IO hal_i2c_run_state_enum     rx_state;           /*!< receive state */
    __IO uint32_t                   previous_state;     /*!< previous state */
    void                            *rx_callback;       /*!< receive callback function pointer */
    void                            *tx_callback;       /*!< transmit callback function pointer */
} hal_i2c_dev_struct;

typedef void (*hal_i2c_user_cb)(hal_i2c_dev_struct *i2c);

/* function declarations */
/* initialize the I2C structure with the default values */
void hal_i2c_struct_init(hal_i2c_struct_type_enum struct_type, void *p_struct);
/* deinitialize I2C */
void hal_i2c_deinit(hal_i2c_dev_struct *i2c);
/* initialize I2C */
int32_t hal_i2c_init(hal_i2c_dev_struct *i2c, uint32_t periph, \
                     hal_i2c_init_struct *p_init);
/* I2C error interrupt handler */
void hal_i2c_error_irq(hal_i2c_dev_struct *i2c);
/* I2C evevt interrupt handler */
void hal_i2c_event_irq(hal_i2c_dev_struct *i2c);
/* start I2C module function */
void hal_i2c_start(hal_i2c_dev_struct *i2c);
/* stop I2C module function */
void hal_i2c_stop(hal_i2c_dev_struct *i2c);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_i2c_irq_handle_set(hal_i2c_dev_struct *i2c, hal_i2c_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_i2c_irq_handle_all_reset(hal_i2c_dev_struct *i2c);
/* transmit amounts of data in master mode, poll transmit process and completed status*/
/* the function is blocking */
int32_t hal_i2c_master_transmit_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                uint32_t length, uint32_t timeout_ms);
/* receive amounts of data in master mode, poll receive process and completed status */
/* the function is blocking */
int32_t hal_i2c_master_receive_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data in slave mode, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_i2c_slave_transmit_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, uint32_t timeout_ms);
/* receive amounts of data in slave mode, poll receive process and completed status */
/* the function is blocking */
int32_t hal_i2c_slave_receive_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, uint32_t timeout_ms);
/* write amounts of data to memory, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_i2c_memory_write_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, uint32_t timeout_ms);
/* read amounts of data from memory, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_i2c_memory_read_poll(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, uint32_t timeout_ms);
/* transmit amounts of data in master mode by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_master_transmit_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* receive amounts of data in master mode by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_master_receive_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                        uint32_t length, hal_i2c_user_cb p_user_func);
/* transmit amounts of data in slave mode by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_slave_transmit_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* receive amounts of data in slave mode by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_slave_receive_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* write amounts of data to memory by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_memory_write_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func);
/* read amounts of data from memory by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_memory_read_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func);
/* transmit amounts of data in master mode by dma method */
/* the function is non-blocking */
int32_t hal_i2c_master_transmit_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func);
/* receive amounts of data in master mode by dma method */
/* the function is non-blocking */
int32_t hal_i2c_master_receive_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func);
/* transmit amounts of data in slave mode by dma method */
/* the function is non-blocking */
int32_t hal_i2c_slave_transmit_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func);
/* receive amounts of data in slave mode by dma method */
/* the function is non-blocking */
int32_t hal_i2c_slave_receive_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                               uint32_t length, hal_i2c_user_cb p_user_func);
/* write amounts of data to memory by dma method */
/* the function is non-blocking */
int32_t hal_i2c_memory_write_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func);
/* read amounts of data from memory by dma method */
/* the function is non-blocking */
int32_t hal_i2c_memory_read_dma(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                              uint32_t length, hal_i2c_user_cb p_user_func);
/* check whether the device is ready for access */
int32_t hal_i2c_device_ready_check(hal_i2c_dev_struct *i2c, uint32_t timeout_ms);
/* serial transmit amounts of data in master mode by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_master_serial_transmit_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* serial receive amounts of data in master mode by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_master_serial_receive_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer, \
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* serial transmit amounts of data in slave mode by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_slave_serial_transmit_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* serial receive amounts of data in slave mode by interrupt method */
/* the function is non-blocking */
int32_t hal_i2c_slave_serial_receive_interrupt(hal_i2c_dev_struct *i2c, uint8_t *p_buffer,\
                                    uint32_t length, hal_i2c_user_cb p_user_func);
/* enable address listen in slave mode by interrupt method */
int32_t hal_i2c_address_listen_interrupt_enable(hal_i2c_dev_struct *i2c, hal_i2c_user_cb p_user_func);
/* disable address listen in slave mode by interrupt method */
int32_t hal_i2c_address_listen_interrupt_disable(hal_i2c_dev_struct *i2c);

#endif /* GD32E23X_HAL_I2C_H */

