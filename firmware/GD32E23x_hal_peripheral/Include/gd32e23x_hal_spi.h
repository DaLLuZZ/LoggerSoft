/*!
    \file    gd32e23x_hal_spi.h
    \brief   definitions for the SPI
    
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

#ifndef GD32E23X_HAL_SPI_H
#define GD32E23X_HAL_SPI_H
#include "gd32e23x_hal.h"

/* SPI error code */
#define HAL_SPI_ERROR_NONE              0U       /*!< no error */
#define HAL_SPI_ERROR_MODF              BIT(0)   /*!< mode fault error */
#define HAL_SPI_ERROR_CRC               BIT(1)   /*!< CRC error */
#define HAL_SPI_ERROR_OVR               BIT(2)   /*!< overrun error */
#define HAL_SPI_ERROR_FRE               BIT(3)   /*!< frame error */
#define HAL_SPI_ERROR_DMA               BIT(4)   /*!< DMA transfer error */
#define HAL_SPI_ERROR_ABORT             BIT(6)   /*!< error during SPI abort procedure */
#define HAL_SPI_ERROR_TIMEOUT           BIT(7)   /*!< SPI timeout error */


/* get SPI init value */
#define __HAL_SPI_GET_DEVICE_MODE(periph)      ((SPI_CTL0(periph)) & ((SPI_CTL0_MSTMOD) | (SPI_CTL0_SWNSS)))
#define __HAL_SPI_GET_SPI0_FRAME_SIZE(periph)  ((SPI_CTL0(periph)) & (SPI_CTL0_FF16))
#define __HAL_SPI_GET_SPI1_FRAME_SIZE(periph)  ((SPI_CTL1(periph)) & (SPI_CTL1_DZ))
#define __HAL_SPI_GET_SPI1_CRC_LENGTH(periph)  ((SPI_CTL0(periph)) & (SPI_CTL0_CRCL))
#define __HAL_SPI_GET_CRC_USED(periph)         ((SPI_CTL0(periph)) & (SPI_CTL0_CRCEN))
#define __HAL_SPI_GET_TRANS_MODE(periph)       ((SPI_CTL0(periph)) & ((SPI_CTL0_BDEN | SPI_CTL0_BDOEN | SPI_CTL0_RO)))

/* TI mode selection */
#define SPI_TIMODE_DISABLE              (0x00000000U)   /*!< SPI TI mode disable */
#define SPI_TIMODE_ENABLE               SPI_CTL1_TMOD   /*!< SPI TI mode enable */

/* NSSP mode selection */
#define SPI_NSSP_ENABLE                 SPI_CTL1_NSSP   /*!< SPI NSSP mode disable */
#define SPI_NSSP_DISABLE                (0x00000000U)   /*!< SPI NSSP mode enable */

/* CRC function selection */
#define SPI_CRC_DISABLE                 (0x00000000U)   /*!< SPI CRC disable */
#define SPI_CRC_ENABLE                  SPI_CTL0_CRCEN  /*!< SPI CRC enable */

/* clear SPI error flag */                                               
#define SPI_ERROR_FLAG_CONF             (0x00000000U)   /*!< SPI configuration error */
#define SPI_ERROR_FLAG_RXORERR          (0x00000001U)   /*!< SPI reception overrun error */
#define SPI_ERROR_FLAG_FERR             (0x00000002U)   /*!< SPI format error */
#define SPI_ERROR_FLAG_CRCERR           (0x00000003U)   /*!< SPI CRC error */

/* SPI receive or transmit buffer struct definitions */
typedef struct {
    __IO uint8_t *buffer;                               /*!< pointer to SPI transfer buffer*/
    __IO uint32_t length;                               /*!< SPI transfer length */
    __IO uint32_t pos;                                  /*!< SPI transfer position */
} spi_buffer_struct;

/* SPI device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb receive_handler;                  /*!< SPI receive complete callback function */
    hal_irq_handle_cb transmit_handler;                 /*!< SPI transmit complete callback function */
    hal_irq_handle_cb transmit_receive_handler;         /*!< SPI transmit and receive complete callback function */
    hal_irq_handle_cb error_handler;                    /*!< SPI error complete callback function */
} hal_spi_irq_struct;

/* SPI structure type enum */
typedef enum {
    HAL_SPI_INIT_STRUCT,                                /*!< SPI initialization structure */
    HAL_SPI_DEV_STRUCT,                                 /*!< SPI device information structure */
} hal_spi_struct_type_enum;          

/* SPI run state */
typedef enum{
  HAL_SPI_STATE_READY      = 0x01U,                     /*!< peripheral Initialized and ready for use */
  HAL_SPI_STATE_BUSY       = 0x02U,                     /*!< an internal process is ongoing */
  HAL_SPI_STATE_BUSY_TX    = 0x03U,                     /*!< data transmission process is ongoing */
  HAL_SPI_STATE_BUSY_RX    = 0x04U,                     /*!< data reception process is ongoing */
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,                     /*!< data Transmission and Reception process is ongoing */
  HAL_SPI_STATE_ERROR      = 0x06U,                     /*!< SPI error state */
  HAL_SPI_STATE_ABORT      = 0x07U                      /*!< SPI abort is ongoing */
}hal_spi_run_state_enum;

/* SPI parameter struct definitions */
typedef struct {
    uint32_t device_mode;                               /*!< SPI master or slave */
    uint32_t trans_mode;                                /*!< SPI transtype */
    uint32_t frame_size;                                /*!< SPI frame size */
    uint32_t nss;                                       /*!< SPI NSS control by handware or software */
    uint32_t endian;                                    /*!< SPI big endian or little endian */
    uint32_t clock_polarity_phase;                      /*!< SPI clock phase and polarity */
    uint32_t prescale;                                  /*!< SPI prescaler value */
    uint32_t crc_calculation;                           /*!< SPI CRC function selection */
    uint32_t crc_poly;                                  /*!< SPI CRC polynomial value */
    uint32_t crc_length;                                /*!< SPI CRC length */
    uint32_t nssp_mode;                                 /*!< SPI NSSP mode selection */
    uint32_t ti_mode;                                   /*!< SPI TI mode selection*/
}hal_spi_init_struct;

/* SPI device information structure */
typedef struct{
    uint32_t                        periph;             /*!< SPI peripheral */
    hal_spi_irq_struct              spi_irq;            /*!< SPI device interrupt callback function pointer */
    hal_dma_dev_struct              *p_dma_rx;          /*!< DMA receive pointer */
    hal_dma_dev_struct              *p_dma_tx;          /*!< DMA transmit pointer */
    spi_buffer_struct               txbuffer;           /*!< transmit buffer */
    spi_buffer_struct               rxbuffer;           /*!< receive buffer */
    uint32_t                        crc_size;           /*!< CRC size */ 
    void                            *rx_callback;       /*!< receive callback function pointer */
    void                            *tx_callback;       /*!< transmit callback function pointer */
    void                            *tx_rx_callback;    /*!< transmit and receive callback function pointer */
    void                            *error_callback;    /*!< error callback function pointer */
    __IO hal_spi_run_state_enum     state;              /*!< SPI communication state */
    __IO uint32_t                   error_code;         /*!< SPI error code*/
} hal_spi_dev_struct;

/* SPI device user callback function pointer */
typedef void (*hal_spi_user_cb)(hal_spi_dev_struct *spi);

/* SPI callback structure */
typedef struct{
    hal_spi_user_cb complete_func;                      /*!< SPI user complete callback function */
    hal_spi_user_cb error_func;                         /*!< SPI user error callback function */
}hal_spi_user_callback_struct;

/* function declarations */
/* deinitialize SPI */
void hal_spi_deinit(hal_spi_dev_struct *spi);
/* initialize SPI structure */
void hal_spi_struct_init(hal_spi_struct_type_enum hal_struct_type,void *p_struct);
/* initialize SPI */
int32_t hal_spi_init(hal_spi_dev_struct *spi, uint32_t periph,hal_spi_init_struct *p_init);

/* transmit amounts of data, poll transmit process and completed status */
/* the function is blocking */
int32_t hal_spi_transmit_poll(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, \
                                uint32_t length, uint32_t timeout_ms);
/* receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_spi_receive_poll(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, \
                               uint32_t length, uint32_t timeout_ms);
/* transmit and receive amounts of data, poll receive process and completed status */
/* the function is blocking */
int32_t hal_spi_transmit_receive_poll(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, \
                               uint8_t *p_rxbuffer, uint32_t length, uint32_t timeout_ms);

/* transmit amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_spi_transmit_interrupt(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, \
                                   uint32_t length, hal_spi_user_callback_struct *p_user_func);
/* receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_spi_receive_interrupt(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, \
                                  uint32_t length, hal_spi_user_callback_struct *p_user_func);
/* transmit and receive amounts of data by interrupt method */
/* the function is non-blocking */
int32_t hal_spi_transmit_receive_interrupt(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, \
                                           uint8_t *p_rxbuffer,uint32_t length,hal_spi_user_callback_struct *p_user_func);

/* transmit amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_spi_transmit_dma(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, \
                             uint32_t length, hal_spi_user_callback_struct *p_user_func);
/* receive amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_spi_receive_dma(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, \
                            uint32_t length, hal_spi_user_callback_struct *p_user_func);
/* transmit and receive amounts of data by dma method */
/* the function is non-blocking */
int32_t hal_spi_transmit_receive_dma(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, \
                                     uint8_t *p_rxbuffer,uint32_t length, hal_spi_user_callback_struct *p_user_func);

/* SPI interrupt handler content function,which is merely used in spi_handler */
void hal_spi_irq(hal_spi_dev_struct *spi);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_spi_irq_handle_set(hal_spi_dev_struct *spi, hal_spi_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_spi_irq_handle_all_reset(hal_spi_dev_struct *spi);

/* SPI start function */
void hal_spi_start(hal_spi_dev_struct *spi);
/* SPI stop function */
void hal_spi_stop(hal_spi_dev_struct *spi);

/* SPI abort function */
int32_t hal_spi_abort(hal_spi_dev_struct *spi);

/* SPI DMA pause function */
void hal_spi_dma_pause(hal_spi_dev_struct *spi);
/* SPI DMA resume function */
void hal_spi_dma_resume(hal_spi_dev_struct *spi);
/* SPI DMA stop function */
void hal_spi_dma_stop(hal_spi_dev_struct *spi);

#endif /* GD32E23X_HAL_SPI_H */
