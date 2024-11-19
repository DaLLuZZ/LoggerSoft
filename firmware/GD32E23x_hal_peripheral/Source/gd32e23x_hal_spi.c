/*!
    \file    gd32e23x_hal_spi.c
    \brief   SPI driver
    
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

#define SPI_INIT_MASK                   ((uint32_t)0x00000040U)  /*!< SPI parameter initialization mask */
#define SPI_FIFO_INIT_MASK1             ((uint32_t)0x00000840U)  /*!< SPI1 parameter initialization mask1 */
#define SPI_FIFO_INIT_MASK2             ((uint32_t)0x00000000U)  /*!< SPI1 parameter initialization mask2*/

#define SPI_TIMEOUT_VALUE  200

/* SPI private function */
static void _spi_transmit_interrupt(void *spi);
static void _spi_receive_interrupt(void *spi);
static void _spi_2lines_receive_interrupt(void *spi);
static void _spi_2lines_transmit_interrupt(void *spi);

static void _spi_transmit_compelete_dma(void *dma);
static void _spi_receive_compelete_dma(void *dma);
static void _spi_transmit_receive_compelete_dma(void *dma);
static void _spi_dma_error(void *dma);

static void _spi_stop_receive_interrupt(void *spi);
static void _spi_stop_transmit_interrupt(void *hspi);
static void _spi_close_receive_interrupt(hal_spi_dev_struct *spi);
static void _spi_close_transmit_interrupt(hal_spi_dev_struct *spi);
static void _spi_close_transmit_receive_interrupt(hal_spi_dev_struct *spi);

static int32_t _spi_end_transmit_receive(hal_spi_dev_struct *spi, uint32_t timeout_ms);
static int32_t _spi_end_receive(hal_spi_dev_struct *spi, uint32_t timeout_ms);

static void _spi_clear_error_flag(uint32_t periph ,uint32_t error_flag);

/*!
    \brief      initialize the SPI structure with the default values
    \param[in]  struct_type: refer to hal_spi_struct_type_enum
    \param[in]  p_struct: point to SPI structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_spi_struct_init(hal_spi_struct_type_enum struct_type,void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(struct_type){
    case HAL_SPI_INIT_STRUCT:
         /* initialize SPI initialization structure with the default values */
        ((hal_spi_init_struct*)p_struct)->device_mode = SPI_MASTER ;
        ((hal_spi_init_struct*)p_struct)->trans_mode  = SPI_TRANSMODE_FULLDUPLEX;
        ((hal_spi_init_struct*)p_struct)->frame_size  = SPI_FRAMESIZE_8BIT;
        ((hal_spi_init_struct*)p_struct)->nss         = SPI_NSS_SOFT;
        ((hal_spi_init_struct*)p_struct)->clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
        ((hal_spi_init_struct*)p_struct)->crc_calculation  = SPI_CRC_DISABLE;
        ((hal_spi_init_struct*)p_struct)->crc_length  = SPI_CRC_8BIT;
        ((hal_spi_init_struct*)p_struct)->crc_poly    = 0x07U;
        ((hal_spi_init_struct*)p_struct)->endian      = SPI_ENDIAN_MSB;
        ((hal_spi_init_struct*)p_struct)->ti_mode     = SPI_TIMODE_DISABLE;
        ((hal_spi_init_struct*)p_struct)->nssp_mode   = SPI_NSSP_DISABLE;
        ((hal_spi_init_struct*)p_struct)->prescale    = SPI_PSC_16;
        break;
    
    case HAL_SPI_DEV_STRUCT:
        /* initialize SPI device information structure with the default values */
        ((hal_spi_dev_struct*)p_struct)->periph = 0;
        ((hal_spi_dev_struct*)p_struct)->spi_irq.error_handler = NULL;
        ((hal_spi_dev_struct*)p_struct)->spi_irq.receive_handler = NULL;
        ((hal_spi_dev_struct*)p_struct)->spi_irq.transmit_handler = NULL;
        ((hal_spi_dev_struct*)p_struct)->p_dma_rx        = NULL;
        ((hal_spi_dev_struct*)p_struct)->p_dma_tx        = NULL;
        ((hal_spi_dev_struct*)p_struct)->txbuffer.buffer = NULL;
        ((hal_spi_dev_struct*)p_struct)->txbuffer.length = 0;
        ((hal_spi_dev_struct*)p_struct)->txbuffer.pos    = 0;
        ((hal_spi_dev_struct*)p_struct)->rxbuffer.buffer = NULL;
        ((hal_spi_dev_struct*)p_struct)->rxbuffer.length = 0;
        ((hal_spi_dev_struct*)p_struct)->rxbuffer.pos    = 0;
        ((hal_spi_dev_struct*)p_struct)->rx_callback     = NULL;
        ((hal_spi_dev_struct*)p_struct)->tx_callback     = NULL;
        ((hal_spi_dev_struct*)p_struct)->tx_rx_callback  = NULL;
        ((hal_spi_dev_struct*)p_struct)->error_callback  = NULL;
        ((hal_spi_dev_struct*)p_struct)->state           = HAL_SPI_STATE_READY;
        ((hal_spi_dev_struct*)p_struct)->error_code      = HAL_SPI_ERROR_NONE;
        break;

    default:
        HAL_DEBUGW("parameter [struct_type] value is undefine");
        break;
    }
    
}

/*!
    \brief      deinitialize SPI
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_deinit(hal_spi_dev_struct *spi)
{
    if((SPI0 == spi->periph) || (SPI1 == spi->periph)){
        spi->state = HAL_SPI_STATE_BUSY;
        spi_i2s_deinit(spi->periph);
        hal_spi_struct_init(HAL_SPI_DEV_STRUCT, spi);
        spi->state = HAL_SPI_STATE_READY;
    }else{
        HAL_DEBUGE("parameter [i2c->periph] value is invalid");
    }  
}

/*!
    \brief      initialize SPI registers
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  periph: SPI peripheral address
    \param[in]  p_init: the initialization data needed to initialize SPI
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_spi_init(hal_spi_dev_struct *spi, uint32_t periph, hal_spi_init_struct *p_init)
{
    uint32_t reg = 0U;
    uint32_t reg1 =0U;
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi) || (NULL == p_init)){
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
   
    spi->periph = periph;

    spi->state = HAL_SPI_STATE_BUSY;
    spi_disable(periph);
        
    /* SPI0 init */
    if(SPI0 == periph){
        reg = SPI_CTL0(periph);
        reg &= SPI_INIT_MASK;
        reg1 = SPI_CTL1(periph);
        reg1 &= SPI_FIFO_INIT_MASK2;
        /* frame_size of SPI0 is different to SPI1 */
        reg |= ((p_init->frame_size)&0x00000800);
        reg |= (p_init->device_mode | p_init->crc_calculation | p_init->trans_mode | p_init->nss | p_init->endian | p_init->clock_polarity_phase|p_init->prescale);
        /* write to SPI_CTL0 register */
        SPI_CTL0(periph) = (uint32_t)reg;
        reg1 |= p_init->ti_mode | p_init->nssp_mode;
       
        if((SPI_MASTER ==  p_init->device_mode) && (SPI_NSS_HARD == p_init->nss)){
            reg1 |= SPI_CTL1_NSSDRV;
        }
        /* write to SPI_CTL1 register */
        SPI_CTL1(periph) = (uint32_t)reg1;
        if(SPI_CRC_ENABLE == p_init->crc_calculation){
            spi_crc_polynomial_set(periph,p_init->crc_poly);
        }
    }
    
    /* SPI1 init */
    if(SPI1 == periph){
        reg = SPI_CTL0(periph);
        reg &= SPI_FIFO_INIT_MASK1;
        reg1 = SPI_CTL1(periph);
        reg1 &= SPI_FIFO_INIT_MASK2;
        reg |= (p_init->device_mode | p_init->crc_calculation | p_init->trans_mode | p_init->nss | p_init->endian | p_init->clock_polarity_phase|p_init->prescale);
        /* write to SPI_CTL0 register */
        SPI_CTL0(periph) = (uint32_t)reg;
        reg1 |= (p_init->ti_mode | p_init->nssp_mode | p_init->frame_size);
        if((SPI_MASTER ==  p_init->device_mode) && (SPI_NSS_HARD == p_init->nss)){
            reg1 |= SPI_CTL1_NSSDRV;
        }
        /* write to SPI_CTL1 register */
        SPI_CTL1(periph) = (uint32_t)reg1;
        
        if(p_init->frame_size == SPI_FRAMESIZE_16BIT){
            spi_crc_length_set(periph,SPI_CRC_16BIT);
        }
        if(p_init->frame_size == SPI_FRAMESIZE_8BIT){
            spi_crc_length_set(periph,SPI_CRC_8BIT);
        }
        if(SPI_CRC_ENABLE == p_init->crc_calculation){
            spi_crc_polynomial_set(periph,p_init->crc_poly);
        }
        /* CRC calculation is valid only for 16bit and 8 bit */
        if ((p_init->frame_size != SPI_FRAMESIZE_16BIT) && (p_init->frame_size != SPI_FRAMESIZE_8BIT)){
            /* CRC disable */
            spi_crc_off(spi->periph);
        }
    }

    /* select SPI mode */
    SPI_I2SCTL(periph) &= (uint32_t)(~SPI_I2SCTL_I2SSEL);
    
    spi->state = HAL_SPI_STATE_READY;
    return HAL_ERR_NONE;

}

/*!
    \brief      SPI transmit in polling mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, HAL_ERR_BUSY,HAL_ERR_TIMEOUT details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_transmit_poll(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint32_t length, uint32_t timeout_ms)
{
    uint32_t tick_start;

    if (spi->state != HAL_SPI_STATE_READY){
        return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if (NULL == spi){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* set the transaction information */
    spi->state = HAL_SPI_STATE_BUSY_TX;
    spi->error_code = HAL_ERR_NONE;
    spi->txbuffer.buffer  = (uint8_t *)p_txbuffer;
    spi->txbuffer.length  = length;
    spi->txbuffer.pos     = length;

    /* init field not used to zero */
    spi->rxbuffer.buffer  = (uint8_t *)NULL;
    spi->rxbuffer.length  = 0U;
    spi->rxbuffer.pos     = 0U;
    spi->tx_callback      = NULL;
    spi->rx_callback      = NULL;

    /* 1 line transmit */
    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)){
        SPI_CTL0(spi->periph) |= SPI_CTL0_BDOEN;
    }
    
    if (__HAL_SPI_GET_CRC_USED(spi->periph) == SPI_CRC_ENABLE){
        /* reset CRC */
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
    }
    spi_enable(spi->periph);
    
    if(((__HAL_SPI_GET_SPI1_FRAME_SIZE(spi->periph) > SPI_FRAMESIZE_8BIT)&&(SPI1 == spi->periph))||
        ((__HAL_SPI_GET_SPI0_FRAME_SIZE(spi->periph) == 0x00000800)&&(SPI0 == spi->periph))){
        SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
        if((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) || (0x01U == spi->txbuffer.pos)){
            
            SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
            p_txbuffer += sizeof(uint16_t);
            spi->txbuffer.pos--;
        }
    
        /* transmit 16 bit data*/
        while(spi->txbuffer.pos > 0U){
            /* wait TBE set */
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_TBE)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
            /* transmit data */
            SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
            p_txbuffer += sizeof(uint16_t);
            spi->txbuffer.pos--;
         }

    }else{/* transmit 8 bit data */
        if(SPI1 == spi->periph){
            
            if ((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) || (0x01U == spi->txbuffer.pos)){
                if (spi->txbuffer.pos > 1U){
                    SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                    /* packing mode */
                    SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
                    p_txbuffer += sizeof(uint16_t);
                    spi->txbuffer.pos -= 2U;
                }else{
                    SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
                    SPI_DATA(spi->periph) = (*p_txbuffer++);
                    spi->txbuffer.pos--;
                }
            }
            while (spi->txbuffer.pos > 0U){
                SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                /* wait until TBE is set*/
                tick_start = hal_basetick_count_get();
                while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_TBE)){
                    if(HAL_TIMEOUT_FOREVER != timeout_ms){
                            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                                return HAL_ERR_TIMEOUT;
                        }
                    }
                }

                if (spi->txbuffer.pos > 1U){
                    SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                    /* packing mode */
                    SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
                    p_txbuffer += sizeof(uint16_t);
                    spi->txbuffer.pos -= 2U;
                }else{
                    SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
                    SPI_DATA(spi->periph) = (*p_txbuffer++);
                    spi->txbuffer.pos--;
                }
            }
        }
        if(SPI0 == spi->periph){
            if ((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) || (0x01U == spi->txbuffer.pos)){
                SPI_DATA(spi->periph) = (*p_txbuffer);
                p_txbuffer += sizeof(uint8_t);
                spi->txbuffer.pos--;
            }
            while(spi->txbuffer.pos > 0U){
                /* wait until TBE is set */
                tick_start = hal_basetick_count_get();
                while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_TBE)){
                    if(HAL_TIMEOUT_FOREVER != timeout_ms){
                        if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                                return HAL_ERR_TIMEOUT;
                        }
                    }
                }

                *(__IO uint8_t *)&SPI_DATA(spi->periph) = (*p_txbuffer++);
                spi->txbuffer.pos--;        
            }            
        }
    }
    
    if(__HAL_SPI_GET_CRC_USED(spi->periph) == SPI_CRC_ENABLE){
        /* next data is crc */
        spi_crc_next(spi->periph);
    }

    /* check the end of the transaction */
    if(HAL_ERR_NONE != _spi_end_transmit_receive(spi,SPI_TIMEOUT_VALUE)){
        return HAL_ERR_TIMEOUT ;
    }
    spi->state = HAL_SPI_STATE_READY;
    return HAL_ERR_NONE;
}


/*!
    \brief      SPI receive in polling mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rxbuffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_BUSY details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_receive_poll(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer,uint32_t length, uint32_t timeout_ms)
{
    __IO uint16_t tmp_crc = 0U;
    uint32_t tick_start = 0;
    if ((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) && (SPI_TRANSMODE_FULLDUPLEX == __HAL_SPI_GET_TRANS_MODE(spi->periph))){
        //spi->state = HAL_SPI_STATE_BUSY_RX;
        /* call transmit-receive function to send dummy data generate clock */
        return hal_spi_transmit_receive_poll(spi, p_rxbuffer, p_rxbuffer, length, timeout_ms);
    }    

    if (HAL_SPI_STATE_READY != spi->state){
        return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if (NULL == spi){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    spi->state            = HAL_SPI_STATE_BUSY_RX;
    spi->error_code       = HAL_ERR_NONE;
    /* init field not used to zero */
    spi->txbuffer.buffer  = (uint8_t *)NULL;
    spi->txbuffer.length  = 0;
    spi->txbuffer.pos     = 0;
    spi->tx_callback      = NULL;
    spi->rx_callback      = NULL;
    spi->tx_rx_callback   = NULL;
    spi->error_callback   = NULL;
    /* set the receiver information */
    spi->rxbuffer.buffer  = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length  = length;
    spi->rxbuffer.pos     = length;

    /* 1 line receive */
    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)){
        SPI_CTL0(spi->periph) &= ~SPI_CTL0_BDOEN;
    }
    
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        /* reset CRC */
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
        /* set crc next bit before the latest data */
        spi->rxbuffer.pos--;
    }
    /* half-word access SPI data register */
    if(SPI1 == spi->periph){
        SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
    }

    spi_enable(spi->periph);

    /* receive 8 bit data */
    if(((__HAL_SPI_GET_SPI1_FRAME_SIZE(spi->periph) <= SPI_FRAMESIZE_8BIT)&&(SPI1 == spi->periph))||((__HAL_SPI_GET_SPI0_FRAME_SIZE(spi->periph) == 0x00000000)&&(SPI0 == spi->periph))){
        SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
        while (spi->rxbuffer.pos > 0U){
            /* wait uintl RBNE is set */
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_RBNE)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
            (*(uint8_t *)p_rxbuffer++) = SPI_DATA(spi->periph);
            spi->rxbuffer.pos--;

        }
    }else{/* receive 16 bit data */
        SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
        while (spi->rxbuffer.pos > 0U){
            /* wait uintl RBNE is set */
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_RBNE)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }

            /* receive data */
            (*(uint16_t *)p_rxbuffer)= SPI_DATA(spi->periph);
            p_rxbuffer += sizeof(uint16_t);
            spi->rxbuffer.pos--;    
        }
    }

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
      
            spi_crc_next(spi->periph);
            /* wait until RBNE is set */
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_RBNE)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }

            /* receive last data */
            if(SPI0 == spi->periph){
                if (__HAL_SPI_GET_SPI0_FRAME_SIZE(spi->periph) > SPI_FRAMESIZE_8BIT){
                    *((uint16_t *)p_rxbuffer) = SPI_DATA(spi->periph);
                }else{
                    (*(uint8_t *)p_rxbuffer) = *(__IO uint8_t *)&SPI_DATA(spi->periph);
                }
            }
            if(SPI1 == spi->periph){
                if (__HAL_SPI_GET_SPI1_FRAME_SIZE(spi->periph) > SPI_FRAMESIZE_8BIT){
                    SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                    *((uint16_t *)p_rxbuffer) = SPI_DATA(spi->periph);
                }else{
                     SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
                    (*(uint8_t *)p_rxbuffer) = SPI_DATA(spi->periph);
                }
            }
            /* wait until RBNE is set */
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_RBNE)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }

            tmp_crc = SPI_DATA(spi->periph);
        if(SET == spi_i2s_flag_get(spi->periph,SPI_FLAG_CRCERR)){
            _spi_clear_error_flag(spi->periph,SPI_ERROR_FLAG_CRCERR);
            spi->state = HAL_SPI_STATE_READY;
            return HAL_ERR_HARDWARE;
        }
    }
    /* check the end of the transaction */
    if(HAL_ERR_NONE != _spi_end_transmit_receive(spi,SPI_TIMEOUT_VALUE)){
        spi->state = HAL_SPI_STATE_READY;
        return HAL_ERR_TIMEOUT;
    }

    spi->state = HAL_SPI_STATE_READY;
    return HAL_ERR_NONE;
    
}

/*!
    \brief      SPI transmit and receive in polling mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer 
    \param[in]  p_rxbuffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_transmit_receive_poll(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint8_t *p_rxbuffer, uint32_t length, uint32_t timeout_ms)
{
    /* alternate Rx and Tx during transfer */
    uint32_t tx_flag = 1U;
    __IO uint16_t tmp_crc = 0U;
    uint32_t tick_start = 0;
    if (HAL_SPI_STATE_READY != spi->state){
        return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if (NULL == spi){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* set transmit information */
    spi->state            = HAL_SPI_STATE_BUSY_TX_RX;
    spi->error_code        = HAL_ERR_NONE;
    spi->txbuffer.buffer  = (uint8_t *)p_txbuffer;
    spi->txbuffer.length  = length;
    spi->txbuffer.pos     = length;

    /* set receive information*/
    spi->rxbuffer.buffer  = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length  = length;
    spi->rxbuffer.pos     = length;
    
    spi->tx_callback      = NULL;
    spi->rx_callback      = NULL;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        /* reset CRC */
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
    }

    if(SPI1 == spi->periph){
        SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
    }
    
    spi_enable(spi->periph);

    /* transmit and receive 16 bit data */
    if(((__HAL_SPI_GET_SPI1_FRAME_SIZE(spi->periph) > SPI_FRAMESIZE_8BIT)&&(SPI1 == spi->periph))||((__HAL_SPI_GET_SPI0_FRAME_SIZE(spi->periph) == 0x00000800)&&(SPI0 == spi->periph))){
        if ((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) || (0x01U == spi->txbuffer.pos)){
            SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
            p_txbuffer += sizeof(uint16_t);
            spi->txbuffer.pos--;
        } 
        
        while (spi->txbuffer.pos > 0U || spi->rxbuffer.pos > 0U){
            if (tx_flag && (spi->txbuffer.pos > 0U)){
                tick_start = hal_basetick_count_get();
                while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_TBE)){
                    if(HAL_TIMEOUT_FOREVER != timeout_ms){
                        if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                            return HAL_ERR_TIMEOUT;
                        }
                    }
                }

                SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
                p_txbuffer += sizeof(uint16_t);
                spi->txbuffer.pos--;
                tx_flag = 0U;

                /* enable CRC transmission */
                if ((0U == spi->txbuffer.pos) && (SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph))){
                    spi_crc_next(spi->periph);
                }
            }
            /* check RBNE flag */
            if ((spi->rxbuffer.pos > 0U)){
                tick_start = hal_basetick_count_get();
                while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_RBNE)){
                    if(HAL_TIMEOUT_FOREVER != timeout_ms){
                        if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                            return HAL_ERR_TIMEOUT;
                        }
                    }
                }
                *((uint16_t *)p_rxbuffer) = SPI_DATA(spi->periph);
                p_rxbuffer += sizeof(uint16_t);
                spi->rxbuffer.pos--;
                /* next data is a transmission */
                tx_flag = 1U;
            }
        }
     }else{ /* transmit and receive 8 bit data */
        if(SPI1 == spi->periph){
            if((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) || (0x01U == spi->txbuffer.pos)){
                if (spi->txbuffer.pos > 1U){
                    SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                    /* packing mode */
                    SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
                    p_txbuffer += sizeof(uint16_t);
                    spi->txbuffer.pos -= 2U;
                }else{
                    SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
                    SPI_DATA(spi->periph) = (*p_txbuffer++);
                    spi->txbuffer.pos--;
                    SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                }
            }
            while (spi->txbuffer.pos > 0U || spi->rxbuffer.pos > 0U){
                if (tx_flag && (spi->txbuffer.pos > 0U)){
                    /* wait until TBE is set */
                    tick_start = hal_basetick_count_get();
                    while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_TBE)){
                        if(HAL_TIMEOUT_FOREVER != timeout_ms){
                            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                                return HAL_ERR_TIMEOUT;
                            }
                        }
                    }

                    if(spi->txbuffer.pos > 1U){
                        SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                        /* packing mode */
                        SPI_DATA(spi->periph) = *((uint16_t *)p_txbuffer);
                        p_txbuffer += sizeof(uint16_t);
                        spi->txbuffer.pos -= 2U;

                    }else{
                        SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
                        SPI_DATA(spi->periph) = (*p_txbuffer++);
                        spi->txbuffer.pos--;
                    }
                    /* next data is a reception */
                    tx_flag = 0U; 

                }
                    if ((spi->txbuffer.pos == 0U) && (SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph))){
                        spi_crc_next(spi->periph);
                    }
                if (spi->rxbuffer.pos > 0U){
                    tick_start = hal_basetick_count_get();
                    /* wait until RBNE is set */
                    while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_RBNE)){
                        if(HAL_TIMEOUT_FOREVER != timeout_ms){
                            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                                return HAL_ERR_TIMEOUT;
                            }
                        }
                    }

                    if (spi->rxbuffer.pos > 1U){
                         SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                        /* packing mode */
                        *((uint16_t *)p_rxbuffer) =SPI_DATA(spi->periph) ;
                        p_rxbuffer += sizeof(uint16_t);
                        spi->rxbuffer.pos -= 2U;

                    }else{
                        SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
                        (*p_rxbuffer++) = SPI_DATA(spi->periph);
                        spi->rxbuffer.pos--;
                    }
                    tx_flag = 1U;
                }
            }
        }
        if(SPI0 == spi->periph){
            if ((SPI_SLAVE == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) || (0x01U == spi->txbuffer.pos)){
                SPI_DATA(spi->periph) = (*p_txbuffer);
                p_txbuffer += sizeof(uint8_t);
                spi->txbuffer.pos--;
             }
            while(spi->txbuffer.pos > 0U || spi->rxbuffer.pos > 0U){
                if(tx_flag && (spi->txbuffer.pos > 0U)){
                     tick_start = hal_basetick_count_get();
                     /* wait until TBE is set */
                     while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_TBE)){
                         if(HAL_TIMEOUT_FOREVER != timeout_ms){
                             if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                                  return HAL_ERR_TIMEOUT;
                             }
                         }
                     }

                    *(__IO uint8_t *)&SPI_DATA(spi->periph) = (*p_txbuffer++);
                    spi->txbuffer.pos--;
                    tx_flag = 0U;
                
                }
                if((spi->txbuffer.pos == 0U) && (SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph))){
                    spi_crc_next(spi->periph);
                }
                if(spi->rxbuffer.pos > 0U){
                    tick_start = hal_basetick_count_get();
                    while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_RBNE)){
                        if(HAL_TIMEOUT_FOREVER != timeout_ms){
                            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                                return HAL_ERR_TIMEOUT;
                            }
                        }
                    }

                    (*(uint8_t *)p_rxbuffer++) = SPI_DATA(spi->periph);
                    spi->rxbuffer.pos--;
                    tx_flag = 1U;
                }
            }
        }
    }

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        while(RESET == spi_i2s_flag_get(spi->periph, SPI_FLAG_RBNE)){
            tick_start = hal_basetick_count_get();
            if(HAL_TIMEOUT_FOREVER != timeout_ms){
                if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                    spi->state = HAL_SPI_STATE_READY;
                    return HAL_ERR_TIMEOUT;
                }
            }
        }
        tmp_crc = SPI_DATA(spi->periph);
        if(SET == spi_i2s_flag_get(spi->periph,SPI_FLAG_CRCERR)){
            _spi_clear_error_flag(spi->periph,SPI_ERROR_FLAG_CRCERR);
            spi->state = HAL_SPI_STATE_READY;
            return HAL_ERR_HARDWARE;
        }
    }

    /* check the end of the transaction */
    if(HAL_ERR_NONE != _spi_end_transmit_receive(spi,SPI_TIMEOUT_VALUE)){
        spi->state = HAL_SPI_STATE_READY;
        return HAL_ERR_TIMEOUT ;
    }
    spi->state = HAL_SPI_STATE_READY;
    return HAL_ERR_NONE;
}

/*!
    \brief      SPI transmit in interrupt mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_transmit_interrupt(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint32_t length, hal_spi_user_callback_struct *p_user_func)
{
    if (HAL_SPI_STATE_READY != spi->state){
        return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if (NULL == spi){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* set the transmit information */
    spi->state = HAL_SPI_STATE_BUSY_TX;
    spi->error_code = HAL_SPI_ERROR_NONE;
    spi->txbuffer.buffer = (uint8_t *)p_txbuffer;
    spi->txbuffer.length = length;
    spi->txbuffer.pos = length;

    /* init field not used to zero */
    spi->rxbuffer.buffer  = (uint8_t *)NULL;
    spi->rxbuffer.length  = 0U;
    spi->rxbuffer.pos = 0U;
    
    spi->rx_callback = NULL;
    spi->tx_rx_callback = NULL;
    spi->tx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;
    
    spi->spi_irq.transmit_handler = _spi_transmit_interrupt;

    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)){
        SPI_CTL0(spi->periph) |= SPI_CTL0_BDOEN;
    }
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
    }

    if(SPI1 == spi->periph){
        SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
    }
    /* enable TBE and ERR interrupt */
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_TBE);
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_ERR);

    spi_enable(spi->periph);

    return HAL_ERR_NONE;    
}

/*!
    \brief      SPI receive in interrupt mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rxbuffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_receive_interrupt(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, uint32_t length, hal_spi_user_callback_struct *p_user_func)
{
    
    if ((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) && (SPI_TRANSMODE_FULLDUPLEX == __HAL_SPI_GET_TRANS_MODE(spi->periph))){
        spi->state = HAL_SPI_STATE_BUSY_RX;
        /* call transmit-receive function to send dummy data generate clock */
        return hal_spi_transmit_receive_interrupt(spi, p_rxbuffer, p_rxbuffer, length, p_user_func);
    }    

    if (HAL_SPI_STATE_READY != spi->state){
        return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if (NULL == spi | 0U == length){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    spi->state = HAL_SPI_STATE_BUSY_RX;
    /* init field not used to zero */
    spi->txbuffer.buffer = (uint8_t *)NULL;
    spi->txbuffer.length = 0;
    spi->txbuffer.pos = 0;
    
    /* set the receiption information */
    spi->rxbuffer.buffer = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length = length;
    spi->rxbuffer.pos = length;
    
    spi->tx_callback = NULL;        
    spi->rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    spi->spi_irq.receive_handler = _spi_receive_interrupt;

    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)){
        SPI_CTL0(spi->periph) &= ~SPI_CTL0_BDOEN;
    }
    
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        spi->crc_size = 1U;
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
    }else{
        spi->crc_size = 0U;
    }

    if(SPI1 == spi->periph){
        SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
    }
    /* enable RBNE and ERR interrupt */
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_RBNE);
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_ERR);

    spi_enable(spi->periph);

    return HAL_ERR_NONE;
}

/*!
    \brief      SPI transmit and receive in interrupt mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer 
    \param[in]  p_rxbuffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, HAL_ERR_BUSY details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_transmit_receive_interrupt(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint8_t *p_rxbuffer,uint32_t length, hal_spi_user_callback_struct *p_user_func)
{

    if (spi->state != HAL_SPI_STATE_READY){
        return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if (NULL == spi | 0U == length){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    spi_disable(spi->periph);
    /* set transmit information */
    spi->state       = HAL_SPI_STATE_BUSY_TX_RX;
    spi->txbuffer.buffer  = (uint8_t *)p_txbuffer;
    spi->txbuffer.length  = length;
    spi->txbuffer.pos = length;

    /* set receiption information */
    spi->rxbuffer.buffer  = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length  = length;
    spi->rxbuffer.pos     = length;

    spi->tx_callback = NULL;
    spi->rx_callback = NULL;
    spi->tx_rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    spi->spi_irq.receive_handler = _spi_2lines_receive_interrupt;
    spi->spi_irq.transmit_handler = _spi_2lines_transmit_interrupt;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        spi->crc_size = 1U;
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
    }else{
        spi->crc_size = 0U;
    }

    if(SPI1 == spi->periph){
        SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
    }
    /* enable TBE, RBNE and ERR interrupt */
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_TBE);
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_RBNE);
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_ERR);

    spi_enable(spi->periph);

    return HAL_ERR_NONE;
    
}

/*!
    \brief      SPI transmit in DMA mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_BUSY, HAL_ERR_VAL details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_transmit_dma(hal_spi_dev_struct *spi, uint8_t *p_txbuffer, uint32_t length, hal_spi_user_callback_struct *p_user_func)
{
    uint32_t dma_memory_width_temp=0;
    hal_dma_irq_struct dma_irq;
    
    if (spi->state != HAL_SPI_STATE_READY){
        return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if ((NULL == spi) || (0U == length)){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* set the transaction information */
    spi->state = HAL_SPI_STATE_BUSY_TX;
    spi->txbuffer.buffer = (uint8_t *)p_txbuffer;
    spi->txbuffer.length = length;
    spi->txbuffer.pos = length;
    
    /* Init field not used to zero */
    spi->rxbuffer.buffer = (uint8_t *)NULL;
    spi->rxbuffer.length = 0U;
    spi->rxbuffer.pos = 0U;
    spi->rx_callback = NULL;
    spi->tx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;
       
    /* 1 lines transmit */
    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)){
        SPI_CTL0(spi->periph) |= SPI_CTL0_BDOEN;
    }
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        /* reset CRC */
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
    }
    spi->p_dma_tx->dma_irq.half_finish_handle = NULL;
    spi->p_dma_tx->dma_irq.full_finish_handle = _spi_transmit_compelete_dma;
    spi->p_dma_tx->dma_irq.error_handle = _spi_dma_error;

    
    if(SPI1 == spi->periph){
         SPI_CTL1(spi->periph) &= (uint32_t)(~SPI_CTL1_TXDMA_ODD);
        /* packing mode is enabled only if the DMA setting is HALFWORD */
        if (__HAL_SPI_GET_SPI1_FRAME_SIZE(spi->periph) > SPI_FRAMESIZE_8BIT){
            SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
        }else{
            SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
            dma_memory_width_temp = DMA_CHCTL(spi->p_dma_tx->channel)&(DMA_CHXCTL_MWIDTH);
            if (DMA_MEMORY_WIDTH_16BIT == dma_memory_width_temp){
                SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                if (0U == (spi->txbuffer.length & 0x1U)){
                    SPI_CTL1(spi->periph) &= (uint32_t)(~SPI_CTL1_TXDMA_ODD);
                    spi->txbuffer.pos = (spi->txbuffer.pos >> 1U);
                }else{
                    SPI_CTL1(spi->periph) |= (uint32_t)SPI_CTL1_TXDMA_ODD;
                    spi->txbuffer.pos = (spi->txbuffer.pos >> 1U) + 1U;
                }
            }
        }
    }
    dma_irq.half_finish_handle = NULL;
    dma_irq.full_finish_handle = _spi_transmit_compelete_dma;
    dma_irq.error_handle = _spi_dma_error;
    
    hal_dma_start_interrupt(spi->p_dma_tx,(uint32_t)spi->txbuffer.buffer, (uint32_t)&SPI_DATA(spi->periph), spi->txbuffer.pos, &dma_irq);
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_ERR);
    spi_enable(spi->periph);

    /* enable SPI DMA transmit */
    spi_dma_enable(spi->periph, SPI_DMA_TRANSMIT);

    return HAL_ERR_NONE;
}

/*!
    \brief      SPI receive in DMA mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_rxbuffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_receive_dma(hal_spi_dev_struct *spi, uint8_t *p_rxbuffer, uint32_t length, hal_spi_user_callback_struct *p_user_func)
{
    uint32_t dma_memory_width_temp=0;
    hal_dma_irq_struct dma_irq;

    if ((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) && (SPI_TRANSMODE_FULLDUPLEX == __HAL_SPI_GET_DEVICE_MODE(spi->periph))){
        spi->state = HAL_SPI_STATE_BUSY_RX;
        /* call transmit-receive function to send dummy data generate clock on CLK line */
        return hal_spi_transmit_receive_dma(spi, p_rxbuffer, p_rxbuffer, length, p_user_func);
    }    

    if (spi->state != HAL_SPI_STATE_READY){
         return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if (NULL == spi){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    spi->state = HAL_SPI_STATE_BUSY_RX;
    spi->error_code = HAL_ERR_NONE;
    spi->txbuffer.buffer  = (uint8_t *)NULL;
    spi->txbuffer.length  = 0;
    spi->txbuffer.pos = 0;

    /* set the receive information */
    spi->rxbuffer.buffer  = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length  = length;
    spi->rxbuffer.pos     = length;
    
    spi->rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;
    spi->tx_callback = NULL;
    
    /* 1 line receive */
    if((SPI_CTL0(spi->periph)) & (SPI_CTL0_BDEN)){
        SPI_CTL0(spi->periph) &= ~SPI_CTL0_BDOEN;
    }
    /* DMA receive complete callback */
    spi->p_dma_rx->dma_irq.half_finish_handle = NULL;
    spi->p_dma_rx->dma_irq.full_finish_handle = _spi_receive_compelete_dma;
    spi->p_dma_rx->dma_irq.error_handle = _spi_dma_error;
    
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        /* reset CRC */
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
    }

    if(SPI1 == spi->periph){
        /* packing mode is enabled only if the DMA setting is DMA_MEMORY_WIDTH_16BIT */
        dma_memory_width_temp = DMA_CHCTL(spi->p_dma_rx->channel)&(DMA_CHXCTL_MWIDTH);
        
        SPI_CTL1(spi->periph) &= (uint32_t)(~SPI_CTL1_TXDMA_ODD);
       
        if((__HAL_SPI_GET_SPI1_FRAME_SIZE(spi->periph) > SPI_FRAMESIZE_8BIT)){
        
            SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;

        }else{
            SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
            if(DMA_MEMORY_WIDTH_16BIT == dma_memory_width_temp){
                SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                if (0U == (spi->rxbuffer.length & 0x1U)){
                    SPI_CTL1(spi->periph) &= (uint32_t)(~SPI_CTL1_RXDMA_ODD);
                    spi->rxbuffer.pos = (spi->rxbuffer.pos >> 1U);
                }else{
                    SPI_CTL1(spi->periph) |= (uint32_t)SPI_CTL1_RXDMA_ODD;
                    spi->rxbuffer.pos = (spi->rxbuffer.pos >> 1U) + 1U;
                }
            }
        }
    }
    
    spi->tx_callback = NULL;
    spi->tx_rx_callback = NULL;
    spi->rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;
    
    /* DMA receive complete callback */
    dma_irq.half_finish_handle = NULL;
    dma_irq.full_finish_handle = _spi_receive_compelete_dma;
    dma_irq.error_handle = _spi_dma_error;
    
    hal_dma_start_interrupt(spi->p_dma_rx, (uint32_t)&SPI_DATA(spi->periph) ,(uint32_t)spi->rxbuffer.buffer, spi->rxbuffer.pos, &dma_irq);

    spi_enable(spi->periph);

    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_ERR);

    /* enable SPI DMA receive */
    spi_dma_enable(spi->periph, SPI_DMA_RECEIVE);

    return HAL_ERR_NONE;
}

/*!
    \brief      SPI transmit and receive in DMA mode
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_txbuffer: pointer to txbuffer 
    \param[in]  p_rxbuffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_spi_transmit_receive_dma(hal_spi_dev_struct *spi, uint8_t *p_txbuffer,uint8_t *p_rxbuffer,uint32_t length, hal_spi_user_callback_struct *p_user_func)
{

    uint32_t dma_memory_width_temp=0;
    hal_dma_irq_struct dma_irq;

    
    if (HAL_SPI_STATE_READY != spi->state){
        return HAL_ERR_BUSY;
    }
#if (1 == HAL_PARAMETER_CHECK)
    if ((NULL == spi) || (0U == length)){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* set the transaction information */
    spi->state = HAL_SPI_STATE_BUSY_TX_RX;
    spi->txbuffer.buffer  = (uint8_t *)p_txbuffer;
    spi->txbuffer.length  = length;
    spi->txbuffer.pos = length;
    
    /* set the reception information */
    spi->rxbuffer.buffer = (uint8_t *)p_rxbuffer;
    spi->rxbuffer.length = length;
    spi->rxbuffer.pos = length;
    
    spi->tx_rx_callback = (void *)p_user_func->complete_func;
    spi->error_callback = (void *)p_user_func->error_func;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        /* reset CRC */
        spi_crc_off(spi->periph);
        spi_crc_on(spi->periph);
    }
    
    spi->p_dma_tx->dma_irq.half_finish_handle = NULL;
    spi->p_dma_tx->dma_irq.full_finish_handle = NULL;
    spi->p_dma_tx->dma_irq.error_handle = NULL;
    
    spi->p_dma_rx->dma_irq.half_finish_handle = NULL;
    spi->p_dma_rx->dma_irq.full_finish_handle = _spi_transmit_receive_compelete_dma;
    spi->p_dma_rx->dma_irq.error_handle = _spi_dma_error;

    if(SPI1 == spi->periph){
        SPI_CTL1(spi->periph) &= (uint32_t)(~SPI_CTL1_TXDMA_ODD);
        SPI_CTL1(spi->periph) &= (uint32_t)(~SPI_CTL1_RXDMA_ODD);
        if (__HAL_SPI_GET_SPI1_FRAME_SIZE(spi->periph) > SPI_FRAMESIZE_8BIT){
            SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
        }else{
            SPI_CTL1(spi->periph) |= SPI_CTL1_BYTEN;
            dma_memory_width_temp = DMA_CHCTL(spi->p_dma_tx->channel)&(DMA_CHXCTL_MWIDTH);
            if (DMA_MEMORY_WIDTH_16BIT == dma_memory_width_temp){
                SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                if (0U == (spi->txbuffer.length & 0x1U)){
                    SPI_CTL1(spi->periph) &= (uint32_t)(~SPI_CTL1_TXDMA_ODD);
                    spi->txbuffer.pos = (spi->txbuffer.pos >> 1U);
                }else{
                    SPI_CTL1(spi->periph) |= (uint32_t)SPI_CTL1_TXDMA_ODD;
                    spi->txbuffer.pos = (spi->txbuffer.pos >> 1U) + 1U;
                }
            }
            dma_memory_width_temp = DMA_CHCTL(spi->p_dma_rx->channel)&(DMA_CHXCTL_MWIDTH);
            if(DMA_MEMORY_WIDTH_16BIT == dma_memory_width_temp){
                SPI_CTL1(spi->periph) &= ~SPI_CTL1_BYTEN;
                if(0U == (spi->rxbuffer.length & 0x1U)){
                    SPI_CTL1(spi->periph) &= (uint32_t)(~SPI_CTL1_RXDMA_ODD);
                    spi->rxbuffer.pos = (spi->rxbuffer.pos >> 1U);
                }else{
                    SPI_CTL1(spi->periph) |= (uint32_t)SPI_CTL1_RXDMA_ODD;
                    spi->rxbuffer.pos = (spi->rxbuffer.pos >> 1U) + 1U;
                }
            }
        }
    }

    dma_irq.half_finish_handle = NULL;
    if(HAL_SPI_STATE_BUSY_RX == spi->state){
        dma_irq.full_finish_handle = _spi_receive_compelete_dma;
    }else{
        dma_irq.full_finish_handle = _spi_transmit_receive_compelete_dma;
    }

    dma_irq.error_handle = _spi_dma_error;
    
    hal_dma_start_interrupt(spi->p_dma_rx, (uint32_t)&SPI_DATA(spi->periph),(uint32_t)spi->rxbuffer.buffer, spi->rxbuffer.pos, &dma_irq);
    /* enable SPI DMA receive */
    spi_dma_enable(spi->periph, SPI_DMA_RECEIVE);

    hal_dma_start_interrupt(spi->p_dma_tx,(uint32_t)spi->txbuffer.buffer, (uint32_t)&SPI_DATA(spi->periph), spi->txbuffer.pos, &dma_irq);

    spi_enable(spi->periph);
    spi_i2s_interrupt_enable(spi->periph,SPI_I2S_INT_ERR);
    /* enable SPI DMA transmit */
    spi_dma_enable(spi->periph, SPI_DMA_TRANSMIT);

    return HAL_ERR_NONE;    
}



/*!
    \brief      SPI interrupt handler content function
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_irq(hal_spi_dev_struct *spi)
{
    __IO uint32_t tmp = 0x00U;  
    
    if ((spi_i2s_interrupt_flag_get(spi->periph,SPI_I2S_INT_FLAG_TBE) != RESET)){
        
        spi->spi_irq.transmit_handler(spi);
        
    }
    if (((SPI_STAT(spi->periph) & SPI_STAT_RXORERR) == RESET)&&(spi_i2s_interrupt_flag_get(spi->periph,SPI_I2S_INT_FLAG_RBNE) != RESET)){
        
        spi->spi_irq.receive_handler(spi);
    }

    if ((((SPI_STAT(spi->periph) & (SPI_STAT_RXORERR | SPI_STAT_CONFERR | SPI_STAT_FERR))) != RESET) && ((SPI_CTL1(spi->periph) & SPI_CTL1_ERRIE) != RESET)){
        /* SPI overrun error interrupt occurred */
        if (spi_i2s_interrupt_flag_get(spi->periph,SPI_I2S_INT_FLAG_RXORERR) != RESET){
            _spi_clear_error_flag(spi->periph,SPI_ERROR_FLAG_RXORERR);
        }

        /* SPI mode error interrupt occurred */
        if (spi_i2s_interrupt_flag_get(spi->periph,SPI_INT_FLAG_CONFERR) != RESET){
            _spi_clear_error_flag(spi->periph,SPI_ERROR_FLAG_CONF);
        }

        /* SPI frame error interrupt occurred */
        if(spi_i2s_interrupt_flag_get(spi->periph,SPI_I2S_INT_FLAG_FERR) != RESET){
            _spi_clear_error_flag(spi->periph,SPI_ERROR_FLAG_FERR);
        }

        /* disable all interrupts */
        spi_i2s_interrupt_disable(spi->periph,SPI_I2S_INT_TBE);
        spi_i2s_interrupt_disable(spi->periph,SPI_I2S_INT_RBNE);
        spi_i2s_interrupt_disable(spi->periph,SPI_I2S_INT_ERR);
        spi->state = HAL_SPI_STATE_READY;

        /* disable the SPI DMA requests if enabled */
        hal_dma_stop(spi->p_dma_tx);
        hal_dma_stop(spi->p_dma_rx);
            
    }
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: the structure that contains callback handlers of SPI interrupt
    \param[out] none
    \retval     none
*/
void hal_spi_irq_handle_set(hal_spi_dev_struct *spi, hal_spi_irq_struct *p_irq)
{
    if(NULL != p_irq->error_handler ){
        spi->spi_irq.error_handler = p_irq->error_handler;
        spi_i2s_interrupt_enable(spi->periph, SPI_I2S_INT_ERR);
    }else{
        spi->spi_irq.error_handler = NULL;
        spi_i2s_interrupt_disable(spi->periph, SPI_I2S_INT_ERR);
    }

    if(NULL != p_irq->receive_handler ){
        spi->spi_irq.receive_handler = p_irq->receive_handler;
        spi_i2s_interrupt_enable(spi->periph, SPI_I2S_INT_RBNE);
    }else{
        spi->spi_irq.receive_handler = NULL;
        spi_i2s_interrupt_disable(spi->periph, SPI_I2S_INT_RBNE);
    }
        
    if(NULL != p_irq->transmit_handler ){
        spi->spi_irq.transmit_handler = p_irq->transmit_handler;
        spi_i2s_interrupt_enable(spi->periph, SPI_I2S_INT_TBE);
    }else{
        spi->spi_irq.transmit_handler = NULL;
        spi_i2s_interrupt_disable(spi->periph, SPI_I2S_INT_TBE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  spi: SPI device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_irq_handle_all_reset(hal_spi_dev_struct *spi)
{
    spi->spi_irq.error_handler=NULL;
    spi->spi_irq.transmit_handler = NULL;
    spi->spi_irq.receive_handler =NULL;
    spi->spi_irq.transmit_receive_handler =NULL;
}

/*!
    \brief      enable SPI peripheral
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/void hal_spi_start(hal_spi_dev_struct *spi)
{
    spi_enable(spi->periph);
}
/*!
    \brief      disable SPI peripheral
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/void hal_spi_stop(hal_spi_dev_struct *spi)
{
    spi_disable(spi->periph);
}

/*!
    \brief      stop SPI DMA
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_spi_abort(hal_spi_dev_struct *spi)
{
    uint32_t tick_start = 0;
    int32_t errorcode;
    errorcode = HAL_ERR_NONE;
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == spi)){
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    if(SPI_CTL1(spi->periph) & SPI_CTL1_TBEIE){
        spi->spi_irq.transmit_handler = _spi_stop_transmit_interrupt;
        tick_start = hal_basetick_count_get();
        while(spi->state != HAL_SPI_ERROR_NONE){
            if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                    spi->error_code = HAL_SPI_ERROR_ABORT;
                    break;
                }
            }
        }
    }

    if(SPI_CTL1(spi->periph) & SPI_CTL1_RBNEIE){
        spi->spi_irq.receive_handler = _spi_stop_receive_interrupt;
        tick_start = hal_basetick_count_get();
        while(spi->state != HAL_SPI_ERROR_NONE){
            if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                    spi->error_code = HAL_SPI_ERROR_ABORT;
                    break;
                }
            }
        }
    }

    spi_i2s_interrupt_disable(spi->periph,SPI_I2S_INT_ERR);
    if ((SPI_CTL1(spi->periph)&SPI_CTL1_DMATEN )|| (SPI_CTL1(spi->periph)&SPI_CTL1_DMAREN )){
        if (spi->p_dma_tx != NULL){
            hal_dma_stop(spi->p_dma_tx);
                                 
            spi->error_code = HAL_SPI_ERROR_NONE;
            spi_dma_disable(spi->periph,SPI_DMA_TRANSMIT);
            if(HAL_ERR_NONE != _spi_end_transmit_receive(spi,SPI_TIMEOUT_VALUE)){
                 spi->error_code = HAL_SPI_ERROR_ABORT;
            }
            spi_disable(spi->periph);

            if(SPI1 == spi->periph){
                tick_start = hal_basetick_count_get();
                while(RESET == spi_i2s_flag_get(spi->periph,SPI_RXLVL_EMPTY)){
                    if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                        if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                                spi->error_code = HAL_SPI_ERROR_ABORT;
                        }
                    }
                }
            }
        }
        if(spi->p_dma_rx != NULL){
            hal_dma_stop(spi->p_dma_rx);
            
            spi->error_code = HAL_SPI_ERROR_ABORT;
            spi_disable(spi->periph);

            tick_start = hal_basetick_count_get();
            while(RESET != spi_i2s_flag_get(spi->periph, SPI_FLAG_TRANS)){
                if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                    if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                        spi->error_code = HAL_SPI_ERROR_ABORT;
                    }
                }
            }

            if(SPI1 == spi->periph){
                tick_start = hal_basetick_count_get();
                while(RESET == spi_i2s_flag_get(spi->periph, SPI_RXLVL_EMPTY)){
                    if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                        if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                            spi->error_code = HAL_SPI_ERROR_ABORT;
                        }
                    }
                }
            }
            spi_dma_disable(spi->periph,SPI_DMA_RECEIVE);
        }
    }
    spi->rxbuffer.pos = 0U;
    spi->txbuffer.pos = 0U;

    if (HAL_SPI_ERROR_ABORT == spi->error_code){
        errorcode = HAL_ERR_NONE ;
    }else{
        spi->error_code = HAL_SPI_ERROR_NONE;
    }

    /* clear the rx overrun error flag */
    _spi_clear_error_flag(spi->periph,SPI_ERROR_FLAG_RXORERR);
    
    spi->state = HAL_SPI_STATE_READY;

    return errorcode;
}

/*!
    \brief      stop SPI DMA
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_dma_pause(hal_spi_dev_struct *spi)
{
    spi_dma_disable(spi->periph,SPI_DMA_TRANSMIT);
    spi_dma_disable(spi->periph,SPI_DMA_RECEIVE);
}

/*!
    \brief      stop SPI DMA
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_dma_resume(hal_spi_dev_struct *spi)
{
    spi_dma_enable(spi->periph,SPI_DMA_TRANSMIT);
    spi_dma_enable(spi->periph,SPI_DMA_RECEIVE);
}

/*!
    \brief      stop SPI DMA
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_spi_dma_stop(hal_spi_dev_struct *spi)
{
    if(NULL!= spi->p_dma_rx){
        hal_dma_stop(spi->p_dma_rx);
    }
    if (NULL != spi->p_dma_tx){
        hal_dma_stop(spi->p_dma_tx);
    }

    spi_dma_disable(spi->periph,SPI_DMA_TRANSMIT);
    spi_dma_disable(spi->periph,SPI_DMA_RECEIVE);
    spi->state = HAL_SPI_STATE_READY;
}

/*!
    \brief      SPI 8 bit transmit interrupt  handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_transmit_interrupt(void *spi)
{
    hal_spi_dev_struct *p_spi = spi;
    if ((__HAL_SPI_GET_SPI1_FRAME_SIZE(p_spi->periph) > SPI_FRAMESIZE_8BIT) || ((__HAL_SPI_GET_SPI0_FRAME_SIZE(p_spi->periph) == 0x00000800))){
        SPI_DATA(p_spi->periph)= *((uint16_t *)p_spi->txbuffer.buffer);
        p_spi->txbuffer.buffer += sizeof(uint16_t);
        p_spi->txbuffer.pos--;
    }else{
        if(SPI0 == p_spi->periph){
            spi_i2s_data_transmit(p_spi->periph, *p_spi->txbuffer.buffer++);
            p_spi->txbuffer.pos--;
        }
        if(SPI1 == p_spi->periph){
            SPI_CTL1(p_spi->periph) |= SPI_CTL1_BYTEN;
            SPI_DATA(p_spi->periph) = (*p_spi->txbuffer.buffer++);
            p_spi->txbuffer.pos--;
        }

    }
    
    if (0U == p_spi->txbuffer.pos){
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)){
            spi_crc_next(p_spi->periph);
        }
        _spi_close_transmit_interrupt(p_spi); 
    }
}

/*!
    \brief      SPI receive interrupt  handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_receive_interrupt(void *spi)
{
     hal_spi_dev_struct *p_spi = spi;

    if ((__HAL_SPI_GET_SPI1_FRAME_SIZE(p_spi->periph) > SPI_FRAMESIZE_8BIT) || ((__HAL_SPI_GET_SPI0_FRAME_SIZE(p_spi->periph) == 0x00000800))){
        *((uint16_t *)p_spi->rxbuffer.buffer) = SPI_DATA(p_spi->periph);
        p_spi->rxbuffer.buffer += sizeof(uint16_t);
        p_spi->rxbuffer.pos--;
    }else
    {
        if(SPI0 == p_spi->periph){
            *p_spi->rxbuffer.buffer++ = SPI_DATA(p_spi->periph);
             p_spi->rxbuffer.pos--;   
        }
        if(SPI1 == p_spi->periph){
            SPI_CTL1(p_spi->periph) |= SPI_CTL1_BYTEN;
            (*p_spi->rxbuffer.buffer++) = SPI_DATA(p_spi->periph);
            p_spi->rxbuffer.pos--;
        }
        
    } 
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)&&(1U == p_spi->rxbuffer.pos)){
        spi_crc_next(p_spi->periph);
    }

    if (0U == p_spi->rxbuffer.pos){
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)){
            __IO uint32_t tmp =0;
            tmp = SPI_DATA(p_spi->periph);
        }
        _spi_close_receive_interrupt(p_spi); 
    }
}

/*!
    \brief      SPI 8 bit 2 lines receive interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_2lines_receive_interrupt(void *spi)
{
    hal_spi_dev_struct *p_spi = spi;

    if ((__HAL_SPI_GET_SPI1_FRAME_SIZE(p_spi->periph) > SPI_FRAMESIZE_8BIT) || ((__HAL_SPI_GET_SPI0_FRAME_SIZE(p_spi->periph) == 0x00000800))){
        
        *((uint16_t *)p_spi->rxbuffer.buffer) = spi_i2s_data_receive(p_spi->periph);
        p_spi->rxbuffer.buffer += sizeof(uint16_t);
        p_spi->rxbuffer.pos--;
        
    }else{/* receive data in 8 bit mode */

        if(SPI0 == p_spi->periph){
            *p_spi->rxbuffer.buffer++ = *((__IO uint8_t *)&SPI_DATA(p_spi->periph));
             p_spi->rxbuffer.pos--;   
        }
        if(SPI1 == p_spi->periph){
            if (p_spi->rxbuffer.pos > 1U){
                SPI_CTL1(p_spi->periph) &= ~SPI_CTL1_BYTEN;
                /* packing mode */
                *((uint16_t *)p_spi->rxbuffer.buffer) = SPI_DATA(p_spi->periph);
                p_spi->rxbuffer.buffer += sizeof(uint16_t);
                p_spi->rxbuffer.pos -= 2U;
                if(1 == p_spi->rxbuffer.pos){
                    SPI_CTL1(p_spi->periph) |= SPI_CTL1_BYTEN;
                }
            }else{
                SPI_CTL1(p_spi->periph) |= SPI_CTL1_BYTEN;
                (*p_spi->rxbuffer.buffer++) = SPI_DATA(p_spi->periph);
                p_spi->rxbuffer.pos--;
                SPI_CTL1(p_spi->periph) &= ~SPI_CTL1_BYTEN;
            }
        }
    }
    
    if (0U == p_spi->rxbuffer.pos){
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)){
            __IO uint32_t tmp =0;
            tmp = SPI_DATA(p_spi->periph);
        }
        /* disable RBNE and ERR interrupt */
        spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_RBNE);
        spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_ERR);
        if (0U == p_spi->txbuffer.pos){
            _spi_close_transmit_receive_interrupt(p_spi); 
        }
    }

}

/*!
    \brief      SPI 8 bit 2 lines transmit interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_2lines_transmit_interrupt(void *spi)
{
    hal_spi_dev_struct *p_spi = spi;

    if ((__HAL_SPI_GET_SPI1_FRAME_SIZE(p_spi->periph) > SPI_FRAMESIZE_8BIT) || ((__HAL_SPI_GET_SPI0_FRAME_SIZE(p_spi->periph) == 0x00000800))){

            SPI_DATA(p_spi->periph) = *((uint16_t *)p_spi->txbuffer.buffer);
            p_spi->txbuffer.buffer += sizeof(uint16_t);
            p_spi->txbuffer.pos--;

    }else{/* transmit data in 8 bit mode */
        
        if(SPI0 == p_spi->periph){
            spi_i2s_data_transmit(p_spi->periph, *p_spi->txbuffer.buffer++);
            p_spi->txbuffer.pos--;
        }
        /* transmit data in packing mode */
        if(SPI1 == p_spi->periph){
            if(p_spi->txbuffer.pos > 1U){
                SPI_CTL1(p_spi->periph) &= ~SPI_CTL1_BYTEN;
                SPI_DATA(p_spi->periph) =  *((uint16_t *)p_spi->txbuffer.buffer);
                p_spi->txbuffer.buffer += sizeof(uint16_t);
                p_spi->txbuffer.pos -= 2U;
                if (1U == p_spi->rxbuffer.pos){
                    SPI_CTL1(p_spi->periph) |= SPI_CTL1_BYTEN;
                }
                
            }else{
                SPI_CTL1(p_spi->periph) |= SPI_CTL1_BYTEN;
                SPI_DATA(p_spi->periph) = (*p_spi->txbuffer.buffer++);
                p_spi->txbuffer.pos--;
            }
        }
        
    }
    
    /* check end of the reception */
    if (0U == p_spi->txbuffer.pos){
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)){
            spi_crc_next(p_spi->periph);
            spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_TBE);
            return;
        }
        /* disable RBNE and ERR interrupt */
        spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_TBE);

        if (0U == p_spi->rxbuffer.pos){
            _spi_close_transmit_receive_interrupt(p_spi); 
        }
    }
}

/*!
    \brief      SPI DMA transmit handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _spi_transmit_compelete_dma(void *dma)
{
    hal_spi_user_cb p_func = NULL;
    hal_spi_user_cb p_func_err = NULL;
    
    hal_dma_dev_struct *p_dma;
    hal_spi_dev_struct *p_spi;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_spi = (hal_spi_dev_struct*)p_dma->p_periph;
    p_func = (hal_spi_user_cb)p_spi->tx_callback;
    p_func_err= (hal_spi_user_cb)p_spi->error_callback;

    /* DMA normal Mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN )){
        /* disable ERR interrupt */
        spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_ERR);
        /* disable Tx DMA Request */
        spi_dma_disable(p_spi->periph,SPI_DMA_TRANSMIT);

        /* check the end of the transaction */
        if (HAL_ERR_NONE != _spi_end_transmit_receive(p_spi, SPI_TIMEOUT_VALUE)){
            p_spi->error_code = HAL_SPI_ERROR_TIMEOUT;
        }

        /* clear overrun flag in 2 Lines communication mode because received data is not read */
        if (SPI_TRANSMODE_FULLDUPLEX == __HAL_SPI_GET_DEVICE_MODE(p_spi->periph)){
            if(SET == spi_i2s_flag_get(p_spi->periph,SPI_FLAG_RXORERR)){
                _spi_clear_error_flag(p_spi->periph,SPI_ERROR_FLAG_RXORERR);
                p_spi->error_code = HAL_SPI_ERROR_CRC;
            }
        }

        p_spi->txbuffer.pos = 0U;
        p_spi->state = HAL_SPI_STATE_READY;

        if (p_spi->error_code != HAL_ERR_NONE){
            if(NULL != p_func_err){
                p_func_err(p_spi);
                return;
            }
        }
    }
    if(NULL != p_func){
        p_func(p_spi);
    }

}

/*!
    \brief      SPI DMA receive handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _spi_receive_compelete_dma(void *dma)
{
    __IO uint16_t tmpcrc = 0U;
    hal_spi_user_cb p_func = NULL;
    hal_spi_user_cb p_func_err = NULL;
    
    hal_dma_dev_struct *p_dma;
    hal_spi_dev_struct *p_spi;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_spi = (hal_spi_dev_struct*)p_dma->p_periph;
    p_func = (hal_spi_user_cb)p_spi->rx_callback;
    p_func_err= (hal_spi_user_cb)p_spi->error_callback;
    
    /* DMA Normal Mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN )){
        /* Disable ERR interrupt */
        spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_ERR);
        /* disable Rx/Tx DMA Request */
        spi_dma_disable(p_spi->periph,SPI_DMA_TRANSMIT);
        spi_dma_disable(p_spi->periph,SPI_DMA_RECEIVE);
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)){
            tmpcrc = SPI_DATA(p_spi->periph);
        }



        /* check the end of the transaction */
        if (_spi_end_receive(p_spi, 200) != HAL_ERR_NONE){
            p_spi->error_code = HAL_SPI_ERROR_TIMEOUT;
        }
        
        p_spi->rxbuffer.pos = 0U;
        p_spi->state = HAL_SPI_STATE_READY;

        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)){
            if(SET == spi_i2s_flag_get(p_spi->periph,SPI_FLAG_CRCERR)){
                _spi_clear_error_flag(p_spi->periph,SPI_ERROR_FLAG_CRCERR);
                p_spi->error_code = HAL_SPI_ERROR_CRC;
            }
        }
        
        if (p_spi->error_code != HAL_ERR_NONE){
            if(NULL != p_func_err){
                p_func_err(p_spi);
                return;
            }
        }
    }
    if(NULL != p_func){
        p_func(p_spi);
    }
}

/*!
    \brief      SPI DMA transmit and receive handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _spi_transmit_receive_compelete_dma(void *dma)
{
    uint32_t tick_start = 0;
    __IO uint16_t tmpcrc = 0U;
    hal_spi_user_cb p_func = NULL;
    hal_spi_user_cb p_func_err = NULL;
    
    hal_dma_dev_struct *p_dma;
    hal_spi_dev_struct *p_spi;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_spi = (hal_spi_dev_struct*)p_dma->p_periph;
    p_func = (hal_spi_user_cb)p_spi->tx_rx_callback;
    p_func_err= (hal_spi_user_cb)p_spi->error_callback;
    
    /* DMA normal mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN)){
        /* disable ERR interrupt */
        spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_ERR);
        
        if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(p_spi->periph)){

            if(SPI1 == p_spi->periph){
                if((__HAL_SPI_GET_SPI1_CRC_LENGTH(p_spi->periph) == SPI_CRC_8BIT) &&(__HAL_SPI_GET_SPI1_FRAME_SIZE(p_spi->periph) > SPI_FRAMESIZE_8BIT)){
                    tick_start = hal_basetick_count_get();
                    while(RESET == spi_i2s_flag_get(p_spi->periph, SPI_RXLVL_QUARTER_FULL)){
                        if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                            if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                                spi_crc_off(p_spi->periph);
                                spi_crc_on(p_spi->periph);
                                break;
                            }
                        }
                    }
                    tmpcrc = *(__IO uint8_t *)&SPI_DATA(p_spi->periph);
                }else{
                    tick_start = hal_basetick_count_get();
                    while(RESET != spi_i2s_flag_get(p_spi->periph, SPI_RXLVL_HAlF_FULL)){
                        if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                            if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                                spi_crc_off(p_spi->periph);
                                spi_crc_on(p_spi->periph);
                                break;
                            }
                        }
                    }
                    tmpcrc = SPI_DATA(p_spi->periph);
                }
            }
            if(SPI0 == p_spi->periph){
                tick_start = hal_basetick_count_get();
                while(RESET == spi_i2s_flag_get(p_spi->periph, SPI_FLAG_RBNE)){
                    if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                        if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                            spi_crc_off(p_spi->periph);
                            spi_crc_on(p_spi->periph);
                            break;
                        }
                    }
                }
                tmpcrc = SPI_DATA(p_spi->periph);
            }
        }
        
        /* check the end of the transaction */
        if (_spi_end_transmit_receive(p_spi, SPI_TIMEOUT_VALUE) != HAL_ERR_NONE){
            p_spi->error_code = HAL_SPI_ERROR_TIMEOUT;
        }

        /* disable Rx and Tx DMA */
        spi_dma_disable(p_spi->periph,SPI_DMA_TRANSMIT);
        spi_dma_disable(p_spi->periph,SPI_DMA_RECEIVE);

        p_spi->rxbuffer.pos = 0U;
        p_spi->txbuffer.pos = 0U;
        p_spi->state = HAL_SPI_STATE_READY;
        
        if (p_spi->error_code != HAL_ERR_NONE){
            if(NULL != p_func_err){
                p_func_err(p_spi);
                return;
            }
        }
    }
    if(NULL != p_func){
        p_func(p_spi);
    }
    
}

/*!
    \brief      SPI DMA error handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _spi_dma_error(void *dma)
{
    hal_spi_user_cb p_func_err = NULL;
    
    hal_dma_dev_struct *p_dma;
    hal_spi_dev_struct *p_spi;
        
    p_dma = (hal_dma_dev_struct*)dma;
    p_spi = (hal_spi_dev_struct*)p_dma->p_periph;
    p_func_err= (hal_spi_user_cb)p_spi->error_callback;

    spi_dma_disable(p_spi->periph,SPI_DMA_RECEIVE);
    spi_dma_disable(p_spi->periph,SPI_DMA_TRANSMIT);

    p_spi->error_code = HAL_SPI_ERROR_DMA;
    p_spi->state = HAL_SPI_STATE_READY;
    
    if(NULL != p_func_err){
        p_func_err(p_spi);
    }
}

/*!
    \brief      SPI stop receive  interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_stop_receive_interrupt(void *spi)
{
    uint32_t tick_start = 0;
    hal_spi_dev_struct *p_spi = spi;
    /* disable SPI Peripheral */
    spi_disable(p_spi->periph);

    /* disable TBEIE, RBNEIE and ERRIE interrupts */
    spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_TBE);
    spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_RBNE);
    spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_ERR);

    /* check RBNEIE is diabled */
    tick_start = hal_basetick_count_get();
    while((SPI_CTL1(p_spi->periph) & SPI_CTL1_RBNEIE)){
        if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
            if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                p_spi->error_code = HAL_SPI_ERROR_ABORT;
            }
        }
    }

    tick_start = hal_basetick_count_get();
    while(RESET != spi_i2s_flag_get(p_spi->periph,SPI_FLAG_TRANS)){
        if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
            if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                p_spi->error_code = HAL_SPI_ERROR_ABORT;
            }
        }
    }
    if(SPI1 == p_spi->periph){
        tick_start = hal_basetick_count_get();
        while(RESET == spi_i2s_flag_get(p_spi->periph,SPI_TXLVL_EMPTY)){
            if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                    p_spi->error_code = HAL_SPI_ERROR_ABORT;
                }
            }
        }
    }
    p_spi->state = HAL_SPI_STATE_ABORT;
}

/*!
    \brief      SPI stop transmit interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_stop_transmit_interrupt(void *spi)
{
    uint32_t tick_start = 0;
    hal_spi_dev_struct *p_spi = spi;

    /* disable TBEIE, ERRIE(mode fault event, overrun error, TI frame error) interrupts */
    spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_TBE);
    spi_i2s_interrupt_disable(p_spi->periph,SPI_I2S_INT_ERR);
    if(SPI1 == p_spi->periph){
        tick_start = hal_basetick_count_get();
        while((SPI_CTL1(p_spi->periph) & SPI_CTL1_RBNEIE)){
            if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                    p_spi->error_code = HAL_SPI_ERROR_ABORT;
                }
            }
        }
        
        if(HAL_ERR_NONE != _spi_end_transmit_receive(p_spi,SPI_TIMEOUT_VALUE)){
            p_spi->error_code = HAL_SPI_ERROR_ABORT;
        }

        tick_start = hal_basetick_count_get();
        while(RESET == spi_i2s_flag_get(p_spi->periph,SPI_RXLVL_EMPTY)){
            if(HAL_TIMEOUT_FOREVER != SPI_TIMEOUT_VALUE){
                if(SET == hal_basetick_timeout_check(tick_start, SPI_TIMEOUT_VALUE)){
                    p_spi->error_code = HAL_SPI_ERROR_ABORT;
                }
            }
        }

        p_spi->state = HAL_SPI_STATE_ABORT;
    } 
    /* disable SPI peripheral */
    spi_disable(p_spi->periph);
}

/*!
    \brief      SPI close transmit and  receive interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_close_transmit_receive_interrupt(hal_spi_dev_struct *spi)
{
    hal_spi_user_cb p_func = NULL;
    
    /* disable ERR interrupt */
    spi_i2s_interrupt_disable(spi->periph, SPI_I2S_INT_ERR);
    /* check the end of the transaction */
    if (_spi_end_transmit_receive(spi, SPI_TIMEOUT_VALUE) != HAL_ERR_NONE){
        spi->error_code = HAL_SPI_ERROR_TIMEOUT;
    }
    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        if(RESET != spi_i2s_flag_get(spi->periph,SPI_FLAG_CRCERR)){
            spi_crc_error_clear(spi->periph);
            p_func = (hal_spi_user_cb)spi->error_callback;
            if(NULL != p_func){
                p_func(spi);
            }
            return;
        }
    }
    if(HAL_ERR_NONE == spi->error_code){
        if (HAL_SPI_STATE_BUSY_RX == spi->state){
            spi->state = HAL_SPI_STATE_READY;
            p_func = (hal_spi_user_cb)spi->tx_callback;
            if(NULL != p_func){
                p_func(spi);
            }
        }else{
            spi->state = HAL_SPI_STATE_READY;
            p_func = (hal_spi_user_cb)spi->tx_rx_callback;
            if(NULL != p_func){
                p_func(spi);
            }
        }
    }else{
        spi->state = HAL_SPI_STATE_READY;
        p_func = (hal_spi_user_cb)spi->error_callback;
        if(NULL != p_func){
            p_func(spi);
        }
    }
 
}

/*!
    \brief      SPI close receive interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_close_receive_interrupt(hal_spi_dev_struct *spi)
{
    hal_spi_user_cb p_func = NULL;
    
    /* disable ERR interrupt */
    spi_i2s_interrupt_disable(spi->periph, SPI_I2S_INT_ERR);
    spi_i2s_interrupt_disable(spi->periph, SPI_I2S_INT_RBNE);

    if (HAL_ERR_NONE != _spi_end_receive(spi, SPI_TIMEOUT_VALUE)){
        spi->error_code = HAL_SPI_ERROR_TIMEOUT;
    }

    spi->state = HAL_SPI_STATE_READY;

    if(SPI_CRC_ENABLE == __HAL_SPI_GET_CRC_USED(spi->periph)){
        if(RESET != spi_i2s_flag_get(spi->periph,SPI_FLAG_CRCERR)){
            spi_crc_error_clear(spi->periph);
            p_func = (hal_spi_user_cb)spi->error_callback;
            if(NULL != p_func){
                p_func(spi);
            }
            return;
        }
    }
    if (HAL_SPI_ERROR_NONE == spi->error_code){
        p_func = (hal_spi_user_cb)spi->rx_callback;
        if(NULL != p_func){
            p_func(spi);
        }
    }else{
        p_func = (hal_spi_user_cb)spi->error_callback;
        if(NULL != p_func){
            p_func(spi);
        }
    }    
}

/*!
    \brief      SPI close transmit interrupt handler
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _spi_close_transmit_interrupt(hal_spi_dev_struct *spi)
{
    hal_spi_user_cb p_func = NULL;

    /* disable TBE and ERR interrupt */
    spi_i2s_interrupt_disable(spi->periph, SPI_I2S_INT_ERR);
    spi_i2s_interrupt_disable(spi->periph, SPI_I2S_INT_TBE);

    /* check the end of the transaction */
    if (_spi_end_transmit_receive(spi, SPI_TIMEOUT_VALUE) != HAL_ERR_NONE){
        p_func = (hal_spi_user_cb)spi->error_callback;
        if(NULL != p_func){
            p_func(spi);
        }
    }else{
         p_func = (hal_spi_user_cb)spi->tx_callback;
         spi->state = HAL_SPI_STATE_READY;
         if(NULL != p_func){
            p_func(spi);
         }
    }

}

/*!
    \brief      check SPI receive end
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
static int32_t _spi_end_receive(hal_spi_dev_struct *spi,  uint32_t timeout_ms)
{
    uint32_t tick_start = 0;
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi){
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    if ((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) && (SPI_TRANSMODE_FULLDUPLEX != __HAL_SPI_GET_DEVICE_MODE(spi->periph))){
        /* disable SPI peripheral */
        spi_disable(spi->periph);
    }

    tick_start = hal_basetick_count_get();
    while(RESET != spi_i2s_flag_get(spi->periph, SPI_FLAG_TRANS)){
        if(HAL_TIMEOUT_FOREVER != timeout_ms){
            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    if(SPI1 == spi->periph){
        if((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) && (SPI_TRANSMODE_FULLDUPLEX != __HAL_SPI_GET_DEVICE_MODE(spi->periph))){
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(spi->periph, SPI_RXLVL_EMPTY)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
        }
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      check SPI transmit and receive end
    \param[in]  spi: SPI device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
static int32_t _spi_end_transmit_receive(hal_spi_dev_struct *spi, uint32_t timeout_ms)
{
    uint32_t tick_start = 0;
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == spi){
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check if the transmit fifo is empty */
    if(SPI1 == spi->periph){
        if ((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) && (SPI_TRANSMODE_FULLDUPLEX != __HAL_SPI_GET_DEVICE_MODE(spi->periph))){
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(spi->periph,SPI_TXLVL_EMPTY)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
        }
    }
    
    /* check if the SPI trans bit is SET */
    tick_start = hal_basetick_count_get();
    while(RESET != spi_i2s_flag_get(spi->periph, SPI_FLAG_TRANS)){
        spi_i2s_interrupt_disable(spi->periph,SPI_I2S_INT_TBE);
        spi_i2s_interrupt_disable(spi->periph,SPI_I2S_INT_RBNE);
        spi_i2s_interrupt_disable(spi->periph,SPI_I2S_INT_ERR);
        spi_disable(spi->periph);
        if(HAL_TIMEOUT_FOREVER != timeout_ms){
            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                return HAL_ERR_TIMEOUT;
            }
        }
    }

    /* check if the receive fifo is empty */
    if(SPI1 == spi->periph){
        if ((SPI_MASTER == __HAL_SPI_GET_DEVICE_MODE(spi->periph)) && (SPI_TRANSMODE_FULLDUPLEX != __HAL_SPI_GET_DEVICE_MODE(spi->periph))){
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(spi->periph,SPI_RXLVL_EMPTY)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }
                }
            }
        }
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      check SPI transmit and receive end
    \param[in]  periph: SPIx(x=0,1)
    \param[in]  error_flag: error flag
                only one parameter can be selected which is shown as below:
      \arg        SPI_ERROR_FLAG_CONF: SPI configuration error 
      \arg        SPI_ERROR_FLAG_FERR: SPI format error
      \arg        SPI_ERROR_FLAG_CRCERR: SPI CRC error
      \arg        SPI_ERROR_FLAG_RXORERR: SPI reception overrun error
    \param[out] none
    \retval     none
*/
static void _spi_clear_error_flag(uint32_t periph ,uint32_t error_flag)
{
    __IO uint32_t temp = 0;
    switch(error_flag){
    case SPI_ERROR_FLAG_CONF:
        temp = SPI_STAT(periph);
        SPI_CTL0(periph) &= (uint32_t)(~SPI_CTL0_SPIEN);
         break;
    case SPI_ERROR_FLAG_RXORERR:
        temp = SPI_DATA(periph);
        temp = SPI_STAT(periph);
        break;
    case SPI_ERROR_FLAG_FERR:
        temp = SPI_STAT(periph);
        break;
    case SPI_ERROR_FLAG_CRCERR:
        SPI_STAT(periph) &= (uint32_t)(~SPI_FLAG_CRCERR);
        break;
    default:
        break;
    }
}
