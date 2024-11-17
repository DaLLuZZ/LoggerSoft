/*!
    \file    gd32e23x_hal_smbus.h
    \brief   definitions for the SMBUS

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

#ifndef GD32E23X_HAL_SMBUS_H
#define GD32E23X_HAL_SMBUS_H
#include "gd32e23x_hal.h"

typedef enum {
    SMBUS_STATE_READY,                                          /*!< smbus ready state */
    SMBUS_STATE_FREE,                                           /*!< smbus free state */
    SMBUS_STATE_BUSY                                            /*!< smbus busy state */
} hal_smbus_run_state_enum;

typedef enum {
    HAL_SMBUS_INIT_STRUCT,                                      /*!< SMBUS initialize structure */
    HAL_SMBUS_DEV_STRUCT,                                       /*!< SMBUS device information structure */
    HAL_SMBUS_IRQ_STRUCT                                        /*!< SMBUS device interrupt callback function pointer structure */
} hal_smbus_struct_type_enum;

typedef struct {  
    hal_irq_handle_cb event_handle;                             /*!< event handle */
    hal_irq_handle_cb error_handle;                             /*!< error handle */
} hal_smbus_irq_struct;

typedef struct {
    __IO uint8_t *buffer;                                       /*!< buffer address */
    __IO uint32_t length;                                       /*!< buffer length */
    __IO uint32_t pos;                                          /*!< buffer position */
} smbus_buffer_struct;

typedef struct {
    uint32_t own_address1;                                      /*!< SMBUS own address */
    uint32_t own_address2;                                      /*!< I2C own address2 in dual-address mode */
    uint32_t dual_address;                                      /*!< dual-address mode switch */
    uint32_t clock_speed;                                       /*!< SMBUS clock speed */
    uint32_t address_format;                                    /*!< SMBUS addformat, 7bits or 10bits */
    uint32_t smbus_type;                                        /*!< SMBus type */
    uint32_t smbus_pec;                                         /*!< SMBus PEC calculation enable */
    uint32_t smbus_pec_transfer;                                /*!< SMBus PEC transfer */
    uint32_t smbus_arp;                                         /*!< SMBus ARP protocol enable */
    uint32_t general_call;                                      /*!< whether or not to response to a general call */
    uint32_t no_stretch;                                        /*!< whether to stretch SCL low when data is not ready in slave mode */
} hal_smbus_init_struct;

typedef struct {
    uint32_t                         periph;                    /* smbus port */
    uint16_t                         slave_address;             /* slave address */
    hal_smbus_init_struct            smbus_init;                /* smbus initialize structure */
    hal_smbus_irq_struct             smbus_irq;                 /* smbus interrupt request structure */
    smbus_buffer_struct              txbuffer;                  /* smbus transmit buffer structure */
    smbus_buffer_struct              rxbuffer;                  /* smbus receive buffer structure */
    __IO uint16_t                    last_error;                /* smbus last error */
    __IO uint32_t                    error_state;               /* smbus error state */
    __IO hal_smbus_run_state_enum    tx_state;                  /* smbus transmit state enum */
    __IO hal_smbus_run_state_enum    rx_state;                  /* smbus receive state enum */
    void                             *rx_callback;              /* smbus receive callback fuction */
    void                             *tx_callback;              /* smbus transmit callback fuction */
    void                             *priv;                     /* priv data */
} hal_smbus_dev_struct;

/* SMBUS callback fuction */
typedef void (*hal_smbus_user_cb)(hal_smbus_dev_struct *smbus);


/* address mode for the SMBUS slave */
#define SMBUS_ADDFORMAT_7BITS          I2C_ADDFORMAT_7BITS      /*!< address:7 bits */
#define SMBUS_ADDFORMAT_10BITS         I2C_ADDFORMAT_10BITS     /*!< address:10 bits */

/* I2C dual-address mode switch */
#define SMBUS_DUADEN_DISABLE           I2C_DUADEN_DISABLE       /*!< dual-address mode disabled */
#define SMBUS_DUADEN_ENABLE            I2C_DUADEN_ENABLE        /*!< dual-address mode enabled */

/* whether or not to response to a general call */
#define SMBUS_GCEN_ENABLE              I2C_GCEN_ENABLE          /*!< slave will response to a general call */
#define SMBUS_GCEN_DISABLE             I2C_GCEN_DISABLE         /*!< slave will not response to a general call */

/* whether or not to stretch SCL low */
#define SMBUS_SCLSTRETCH_ENABLE        I2C_SCLSTRETCH_ENABLE    /*!< SCL stretching is enabled */
#define SMBUS_SCLSTRETCH_DISABLE       I2C_SCLSTRETCH_DISABLE   /*!< SCL stretching is disabled */

/* SMBUS transfer direction */
#define SMBUS_RECEIVER                 I2C_RECEIVER             /*!< receiver */
#define SMBUS_TRANSMITTER              I2C_TRANSMITTER          /*!< transmitter */

/* SMBus mode switch and SMBus type selection */
#define SMBUS_DEVICE                   I2C_SMBUS_DEVICE         /*!< SMBus mode device type */
#define SMBUS_HOST                     I2C_SMBUS_HOST           /*!< SMBus mode host type */

/* SMBus mode switch and SMBus type selection */
#define SMBUS_MODE_ENABLE              I2C_SMBUSMODE_ENABLE     /*!< I2C mode */

/* whether or not to send an ACK */
#define SMBUS_ACK_DISABLE              I2C_ACK_DISABLE          /*!< ACK will be not sent */
#define SMBUS_ACK_ENABLE               I2C_ACK_ENABLE           /*!< ACK will be sent */

/* SMBUS POAP position*/
#define SMBUS_ACKPOS_NEXT              I2C_ACKPOS_NEXT          /*!< ACKEN bit decides whether or not to send ACK for the next byte */
#define SMBUS_ACKPOS_CURRENT           I2C_ACKPOS_CURRENT       /*!< ACKEN bit decides whether or not to send ACK or not for the current byte */

/* software reset SMBUS */
#define SMBUS_SRESET_SET               I2C_SRESET_SET           /*!< I2C is under reset */
#define SMBUS_SRESET_RESET             I2C_SRESET_RESET         /*!< I2C is not under reset */

/* software reset SMBUS */
#define SMBUS_PEC_ENABLE               I2C_PEC_ENABLE           /*!< I2C is under reset */
#define SMBUS_PEC_DISABLE              I2C_PEC_DISABLE                       /*!< I2C is not under reset */

/* PEC transfer */
#define SMBUS_PECTRANS_ENABLE          I2C_PECTRANS_ENABLE      /*!< transfer PEC */
#define SMBUS_PECTRANS_DISABLE         I2C_PECTRANS_DISABLE     /*!< not transfer PEC value */

/* ARP protocol in SMBus switch */
#define SMBUS_ARP_ENABLE               I2C_ARP_ENABLE           /*!< ARP enable */
#define SMBUS_ARP_DISABLE              I2C_ARP_DISABLE          /*!< ARP disable */

/* issue or not alert through SMBA pin */
#define SMBUS_SALTSEND_ENABLE          I2C_SALTSEND_ENABLE      /*!< issue alert through SMBA pin */
#define SMBUS_SALTSEND_DISABLE         I2C_SALTSEND_DISABLE     /*!< not issue alert through SMBA */

/* SMBUS error code */
#define HAL_SMBUS_ERROR_NONE           0U                       /*!< no error */
#define HAL_SMBUS_ERROR_BERR           BIT(0)                   /*!< bus error */
#define HAL_SMBUS_ERROR_LOSTARB        BIT(1)                   /*!< arbitration lost in master mode */
#define HAL_SMBUS_ERROR_AERR           BIT(2)                   /*!< acknowledge error */
#define HAL_SMBUS_ERROR_OUERR          BIT(3)                   /*!< over-run error */
#define HAL_SMBUS_ERROR_SMBTO          BIT(4)                   /*!< OUERR error */
#define HAL_SMBUS_ERROR_SMBALT         BIT(5)                   /*!< OUERR error */
#define HAL_SMBUS_ERROR_PECERR         BIT(6)                   /*!< PEC error */
#define HAL_SMBUS_ERROR_DMATX          BIT(7)                   /*!< DMA TX error */
#define HAL_SMBUS_ERROR_DMARX          BIT(8)                   /*!< DMA RX error */
//#define HAL_SMBUS_ERROR_TIMEOUT        BIT(9)                 /*!< Timeout Error */


/* function declarations */
/* initialize the SMBUS structure with the default values */
void hal_smbus_struct_init(hal_smbus_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize SMBUS interface */
void hal_smbus_deinit(hal_smbus_dev_struct *smbus);
/* initialize SMBUS registers */
int32_t hal_smbus_init(hal_smbus_dev_struct *smbus, uint32_t periph, \
                      hal_smbus_init_struct *p_init);
/* SMBUS evevt interrupt handler */
void hal_smbus_event_irq(hal_smbus_dev_struct *smbus);
/* SMBUS error interrupt handler */
void hal_smbus_error_irq(hal_smbus_dev_struct *smbus);
/* start SMBUS transfer */
void hal_smbus_start(hal_smbus_dev_struct *smbus);
/* stop SMBUS transfer */
void hal_smbus_stop(hal_smbus_dev_struct *smbus);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_smbus_irq_handle_set(hal_smbus_dev_struct *smbus, hal_smbus_irq_struct *p_irq);
/* reset all user-defined interrupt callback function */
void hal_smbus_irq_handle_all_reset(hal_smbus_dev_struct *smbus);

/* SMBUS master transmit in interrupt mode */
int32_t hal_smbus_master_transmit_interrupt(hal_smbus_dev_struct *smbus, uint8_t *p_buffer, \
                                     uint32_t length, hal_smbus_user_cb p_user_func);
/* SMBUS master receive with interrupt mode */
int32_t hal_smbus_master_receive_interrupt(hal_smbus_dev_struct *smbus, uint8_t *p_buffer, \
                                    uint32_t length, hal_smbus_user_cb p_user_func);
/* SMBUS slave transmit in interrupt mode */
int32_t hal_smbus_slave_transmit_interrupt(hal_smbus_dev_struct *smbus, uint8_t *p_buffer, \
                                     uint32_t length, hal_smbus_user_cb p_user_func);
/* SMBUS slave receive with interrupt mode */
int32_t hal_smbus_slave_receive_interrupt(hal_smbus_dev_struct *smbus, uint8_t *p_buffer, \
                                    uint32_t length, hal_smbus_user_cb p_user_func);
                                    
/* Enable the SMBUS alert mode with Interrupt */
int32_t hal_smbus_enable_alert_interrupt(hal_smbus_dev_struct *smbus);
/* Disable the SMBUS alert mode with Interrupt */
int32_t hal_smbus_disable_alert_interrupt(hal_smbus_dev_struct *smbus);

#endif /* GD32E23X_HAL_SMBUS_H */
