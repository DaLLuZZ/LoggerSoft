/*!
    \file    gd32e23x_hal_i2s.c
    \brief   I2S driver
    
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

/* I2S private function */
static void _i2s_transmit_complete_dma(void *dma);
static void _i2s_receive_complete_dma(void *dma);
static void _i2s_dma_error(void * dma);
static void _i2s_transmit_interrupt(void *i2s);
static void _i2s_receive_interrupt(void *i2s);

/*!
    \brief      initialize the I2S structure with the default values
    \param[in]  hal_struct_type: refer to hal_i2s_struct_type_enum
    \param[in]  p_struct: point to I2S structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_i2s_struct_init(hal_i2s_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch(hal_struct_type){
    case HAL_I2S_INIT_STRUCT:
        /* initialize I2S initialization structure with the default values */
        ((hal_i2s_init_struct*)p_struct)->mode = I2S_MODE_MASTERTX ;
        ((hal_i2s_init_struct*)p_struct)->standard = I2S_STD_MSB ;
        ((hal_i2s_init_struct*)p_struct)->frameformat = I2S_FRAMEFORMAT_DT32B_CH32B;
        ((hal_i2s_init_struct*)p_struct)->mckout = I2S_MCKOUT_DISABLE;
        ((hal_i2s_init_struct*)p_struct)->audiosample = I2S_AUDIOSAMPLE_8K;
        ((hal_i2s_init_struct*)p_struct)->ckpl = I2S_CKPL_HIGH;
        break;
    
    case HAL_I2S_DEV_STRUCT:
        /* initialize I2S device information structure with the default values */
        ((hal_i2s_dev_struct*)p_struct)->periph = SPI0;
        ((hal_i2s_dev_struct*)p_struct)->i2s_irq.error_handle = NULL;
        ((hal_i2s_dev_struct*)p_struct)->i2s_irq.receive_handler =NULL;
        ((hal_i2s_dev_struct*)p_struct)->i2s_irq.transmit_handler =NULL;
        ((hal_i2s_dev_struct*)p_struct)->p_dma_rx = NULL;
        ((hal_i2s_dev_struct*)p_struct)->p_dma_tx = NULL;
        ((hal_i2s_dev_struct*)p_struct)->rx_callback = NULL;
        ((hal_i2s_dev_struct*)p_struct)->tx_callback = NULL;
        ((hal_i2s_dev_struct*)p_struct)->txbuffer.buffer = NULL;
        ((hal_i2s_dev_struct*)p_struct)->txbuffer.length = 0;
        ((hal_i2s_dev_struct*)p_struct)->txbuffer.pos = 0;
        ((hal_i2s_dev_struct*)p_struct)->rxbuffer.buffer = NULL;
        ((hal_i2s_dev_struct*)p_struct)->rxbuffer.length = 0;
        ((hal_i2s_dev_struct*)p_struct)->rxbuffer.pos = 0;
        ((hal_i2s_dev_struct*)p_struct)->state = HAL_I2S_STATE_READY;
        ((hal_i2s_dev_struct*)p_struct)->error_code = HAL_I2S_ERROR_NONE;
        break;
    
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize I2S
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_deinit(hal_i2s_dev_struct *i2s)
{
    if(SPI0 == i2s->periph){
        i2s->state = HAL_I2S_STATE_BUSY;
        spi_i2s_deinit(i2s->periph);
        hal_i2s_struct_init(HAL_I2S_DEV_STRUCT, i2s);
        i2s->state = HAL_I2S_STATE_READY;
    }else{
        HAL_DEBUGE("parameter [i2c->periph] value is invalid");
    }  
}

/*!
    \brief      initialize I2S registers
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  periph: specify which I2S is initialized
                    this parameter can only be I2S0 for gd32e23x
    \param[in]  p_init: the initialization data needed to initialize I2S
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS,HAL_ERR_VAL, details refer to gd32e23x_hal.h
*/
int32_t hal_i2s_init(hal_i2s_dev_struct *i2s, uint32_t periph, hal_i2s_init_struct *p_init)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check I2S pointer and p_init address */
    if((NULL == i2s) && (NULL == p_init)){
        return HAL_ERR_ADDRESS;
    }
        
    /* check periph parameter */
    if(SPI0 != periph){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    i2s->state = HAL_I2S_STATE_BUSY;
    i2s->periph = periph;  
    /* disable the peripheral */
    spi_disable(periph);
    /* init I2S */
    i2s_init(periph,p_init->mode,p_init->standard,p_init->ckpl);
    i2s_psc_config(periph,p_init->audiosample,p_init->frameformat,p_init->mckout);
    
    i2s->state = HAL_I2S_STATE_READY;

    return HAL_ERR_NONE;
}

/*!
    \brief      I2S transmit in polling mode
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2s_transmit_poll(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, uint32_t timeout_ms)
{
    __IO uint32_t tmp = 0x0U;
    uint32_t tmp1 = 0U;
    uint32_t tick_start = 0;
#if (1 == HAL_PARAMETER_CHECK)
    /* check input parameter */
    if((NULL == p_buffer) || (0U == length)){
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    if(HAL_I2S_STATE_READY == i2s->state){
        tmp1 = SPI_I2SCTL(i2s->periph)&((SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN));
        /* check I2S frameformat */
        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)){
            i2s->txbuffer.length= (length << 1U);
            i2s->txbuffer.pos= (length << 1U);
        }else{
            i2s->txbuffer.length= length;
            i2s->txbuffer.pos= length;
        }

        i2s->error_code = HAL_I2S_ERROR_NONE;
        i2s->state = HAL_I2S_STATE_BUSY_TX;

        /* enable I2S peripheral */
        i2s_enable(i2s->periph);

        /* transmit data */
        while(i2s->txbuffer.pos > 0U){
            SPI_DATA(i2s->periph) = (*p_buffer++);
            i2s->txbuffer.pos--;

            /* wait TBE flag is set */
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(i2s->periph, I2S_FLAG_TBE)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start,timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }            
                }
            }
           if(SET == spi_i2s_flag_get(i2s->periph, I2S_FLAG_TXURERR)){
               /* clear underrun flag */
               tmp = SPI_STAT((i2s)->periph);

               i2s->state = HAL_I2S_STATE_READY;
               i2s->error_code = HAL_I2S_ERROR_UNDERRUN;
               return HAL_ERR_HARDWARE;
            }
        }
        i2s->state = HAL_I2S_STATE_READY;

        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      I2S receive in polling mode
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  timeout_ms: timeout duration 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2s_receive_poll(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, uint32_t timeout_ms)
{
    uint32_t tmp1 = 0U;
    __IO uint32_t tmp = 0x00U;
    uint32_t tick_start = 0;
#if (1 == HAL_PARAMETER_CHECK)
    /* check input parameter */
    if((NULL == p_buffer) || (0U == length)){
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(HAL_I2S_STATE_READY == i2s->state){
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph)&(SPI_I2SCTL_DTLEN| SPI_I2SCTL_CHLEN);
        if((tmp1 == I2S_FRAMEFORMAT_DT24B_CH32B) || (tmp1 == I2S_FRAMEFORMAT_DT32B_CH32B)){
            i2s->rxbuffer.length = (length << 1U);
            i2s->rxbuffer.pos = (length << 1U);
        }else{
            i2s->rxbuffer.length = length;
            i2s->rxbuffer.pos = length;
        }

        i2s->error_code = HAL_ERR_NONE;
        i2s->state = HAL_I2S_STATE_BUSY_RX;

        if(SPI_I2SCTL_I2SEN != (SPI_I2SCTL(i2s->periph) &SPI_I2SCTL_I2SEN)){
            /* enable I2S peripheral */
            i2s_enable(i2s->periph);
        }

        /* receive data */
        while(i2s->rxbuffer.pos > 0U){
            tick_start = hal_basetick_count_get();
            while(RESET == spi_i2s_flag_get(i2s->periph, I2S_FLAG_RBNE)){
                if(HAL_TIMEOUT_FOREVER != timeout_ms){
                    if(SET == hal_basetick_timeout_check(tick_start,timeout_ms)){
                        return HAL_ERR_TIMEOUT;
                    }            
                }
            }

            if(SET == spi_i2s_flag_get(i2s->periph, I2S_FLAG_RXORERR)){
                /* clear overrun error */
                 tmp = SPI_DATA(i2s->periph);
                 tmp = SPI_STAT(i2s->periph);
                /* set the I2S state ready */
                i2s->state = HAL_I2S_STATE_READY;
                i2s->error_code = HAL_I2S_ERROR_OVERRUN;
                return HAL_ERR_HARDWARE;
            }

            (*p_buffer++) = SPI_DATA(i2s->periph);
            i2s->rxbuffer.pos--;
        }

        i2s->state = HAL_I2S_STATE_READY;
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      I2S transmit in interrupt mode
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2s_transmit_interrupt(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, hal_i2s_user_callback_struct *p_user_func)
{
    uint32_t tmp1 = 0U;
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == p_buffer) || (0U == length)){
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    if(i2s->state == HAL_I2S_STATE_READY){
        i2s->txbuffer.buffer = p_buffer;
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph)&(SPI_I2SCTL_DTLEN| SPI_I2SCTL_CHLEN);

        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)){
            i2s->txbuffer.length= (length << 1U);
            i2s->txbuffer.pos= (length << 1U);
        }else{
            i2s->txbuffer.length= length;
            i2s->txbuffer.pos= length;
        }
        i2s->i2s_irq.transmit_handler = _i2s_transmit_interrupt;
        i2s->tx_callback = (void *)p_user_func->complete_func;
        i2s->error_callback = (void *)p_user_func->error_func;
        
        i2s->state = HAL_I2S_STATE_BUSY_TX;

        /* enable TBE and ERR interrupt */
        spi_i2s_interrupt_enable(i2s->periph,SPI_I2S_INT_TBE);
        spi_i2s_interrupt_enable(i2s->periph,SPI_I2S_INT_ERR);
        
        if(SPI_I2SCTL_I2SEN != (SPI_I2SCTL(i2s->periph) &SPI_I2SCTL_I2SEN)){
            /* enable I2S peripheral */
            i2s_enable(i2s->periph);
        }
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      I2S receive in interrupt mode
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2s_receive_interrupt(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, hal_i2s_user_callback_struct *p_user_func)
{
    uint32_t tmp1 = 0U;
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == p_buffer) || (0U == length)){
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */
    
    if(i2s->state == HAL_I2S_STATE_READY){
        i2s->rxbuffer.buffer = p_buffer;
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph)&(SPI_I2SCTL_DTLEN | SPI_I2SCTL_CHLEN);

        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)){
            i2s->rxbuffer.length= (length << 1U);
            i2s->rxbuffer.pos= (length << 1U);
        }else{
            i2s->rxbuffer.length= length;
            i2s->rxbuffer.pos= length;
        }

        i2s->i2s_irq.receive_handler = _i2s_receive_interrupt;
        i2s->rx_callback = (void *)p_user_func->complete_func;
        i2s->error_callback = (void *)p_user_func->error_func;
        i2s->state     = HAL_I2S_STATE_BUSY_RX;

        /* enable RBNE and ERR interrupt */
        spi_i2s_interrupt_enable(i2s->periph,SPI_I2S_INT_RBNE);
        spi_i2s_interrupt_enable(i2s->periph,SPI_I2S_INT_ERR);

        if(SPI_I2SCTL_I2SEN != (SPI_I2SCTL(i2s->periph) &SPI_I2SCTL_I2SEN)){
            /* enable I2S peripheral */
            i2s_enable(i2s->periph);
        }
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      I2S transmit in DMA mode
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to txbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL, details refer to gd32e23x_hal.h 
*/
int32_t hal_i2s_transmit_dma(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, hal_i2s_user_callback_struct *p_user_func)
{
    uint32_t *p_tmp = NULL;
    uint32_t tmp1 = 0U;
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == p_buffer) || (0U == length)){
        return HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(HAL_I2S_STATE_READY == i2s->state){
        i2s->txbuffer.buffer = p_buffer;
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph)&(SPI_I2SCTL_DTLEN| SPI_I2SCTL_CHLEN);
        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)){
            i2s->txbuffer.length = (length << 1U);
            i2s->txbuffer.pos = (length << 1U);
        }else{
            i2s->txbuffer.length = length;
            i2s->txbuffer.pos = length;
        }
        i2s->state = HAL_I2S_STATE_BUSY_TX;
        /* I2S callback function */
        i2s->tx_callback = (void *)p_user_func->complete_func;
        i2s->error_callback = (void *)p_user_func->error_func;
        
        /* I2S DMA transmit info*/
        hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
        dma_irq.half_finish_handle = NULL;
        dma_irq.full_finish_handle = _i2s_transmit_complete_dma;
        dma_irq.error_handle = _i2s_dma_error;
        p_tmp = (uint32_t*)&p_buffer;
        hal_dma_start_interrupt(i2s->p_dma_tx,*(uint32_t*)p_tmp, (uint32_t)&SPI_DATA(i2s->periph), length, &dma_irq);

        /* enable I2S peripheral */
        i2s_enable(i2s->periph);

        /* enable transmit DMA request */
        spi_dma_enable(i2s->periph,SPI_DMA_TRANSMIT);
                
        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      I2S receive in DMA mode
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_buffer: pointer to rxbuffer 
    \param[in]  length: length of data to be sent 
    \param[in]  p_user_func: pointer to call back function for user
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_VAL details refer to gd32e23x_hal.h 
*/
int32_t hal_i2s_receive_dma(hal_i2s_dev_struct *i2s, uint16_t *p_buffer, uint16_t length, hal_i2s_user_callback_struct *p_user_func)
{
    uint32_t *p_tmp = NULL;
    __IO uint32_t tmp1 = 0U;
    hal_dma_irq_struct dma_irq;
#if (1 == HAL_PARAMETER_CHECK)
    if((NULL == p_buffer) || (0U == length)){
        return  HAL_ERR_VAL;
    }
#endif /* 1 = HAL_PARAMETER_CHECK */

    if(HAL_I2S_STATE_READY == i2s->state){
        i2s->rxbuffer.buffer = p_buffer;
        /* check I2S frameformat */
        tmp1 = SPI_I2SCTL(i2s->periph)&(SPI_I2SCTL_DTLEN| SPI_I2SCTL_CHLEN);

        if((I2S_FRAMEFORMAT_DT24B_CH32B == tmp1) || (I2S_FRAMEFORMAT_DT32B_CH32B == tmp1)){
            i2s->rxbuffer.length = (length << 1U);
            i2s->rxbuffer.pos = (length << 1U);
        }else{
            i2s->rxbuffer.length = length;
            i2s->rxbuffer.pos = length;
        }

        i2s->state = HAL_I2S_STATE_BUSY_RX;

        /* I2S callback function */
        i2s->rx_callback = (void *)p_user_func->complete_func;
        i2s->error_callback = (void *)p_user_func->error_func;

        /* I2S DMA receive info*/
        hal_dma_struct_init(HAL_DMA_IRQ_STRUCT, &dma_irq);
        dma_irq.half_finish_handle = NULL;
        dma_irq.full_finish_handle = _i2s_receive_complete_dma;
        dma_irq.error_handle = _i2s_dma_error;
        p_tmp = (uint32_t*)&p_buffer;
        hal_dma_start_interrupt(i2s->p_dma_rx, (uint32_t)&SPI_DATA(i2s->periph),*(uint32_t*)p_tmp, length, &dma_irq);

        i2s_enable(i2s->periph);
        /* enable receive DMA request */
        spi_dma_enable(i2s->periph,SPI_DMA_RECEIVE);

        return HAL_ERR_NONE;
    }else{
        return HAL_ERR_BUSY;
    }
}

/*!
    \brief      pause I2S DMA
    \param[in]  i2s: i2s device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_dma_pause(hal_i2s_dev_struct *i2s)
{
    if(HAL_I2S_STATE_BUSY_TX == i2s->state){
        /* disable the I2S DMA transmit request */
        spi_dma_disable(i2s->periph,SPI_DMA_TRANSMIT);
    }else{
        if(HAL_I2S_STATE_BUSY_RX == i2s->state){
            /* disable the I2S DMA receive request */
            spi_dma_disable(i2s->periph,SPI_DMA_RECEIVE);
        }
    }
}

/*!
    \brief      resume I2S DMA
    \param[in]  i2s: i2s device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_dma_resume(hal_i2s_dev_struct *i2s)
{

    if(HAL_I2S_STATE_BUSY_TX == i2s->state){
        /* disable the I2S DMA transmit request */
        spi_dma_enable(i2s->periph,SPI_DMA_TRANSMIT);
    }else{
        if(HAL_I2S_STATE_BUSY_RX == i2s->state){
            /* disable the I2S DMA receive request */
            spi_dma_enable(i2s->periph,SPI_DMA_RECEIVE);
        }
    }            

    if(SPI_I2SCTL_I2SEN != (SPI_I2SCTL(i2s->periph) &SPI_I2SCTL_I2SEN)){
        /* enable I2S peripheral */
        i2s_enable(i2s->periph);
    }
}

/*!
    \brief      stop I2S DMA
    \param[in]  i2s: i2s device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_dma_stop(hal_i2s_dev_struct *i2s)
{

    if(HAL_I2S_STATE_BUSY_TX == i2s->state){
        /* disable the I2S DMA transmit request */
        spi_dma_disable(i2s->periph,SPI_DMA_TRANSMIT);

        /* disable the I2S DMA channel */
        hal_dma_stop(i2s->p_dma_tx );
    }else{
        if(HAL_I2S_STATE_BUSY_RX == i2s->state){
            /* disable the I2S DMA receive request */
            spi_dma_disable(i2s->periph,SPI_DMA_RECEIVE);

            /* disable the I2S DMA channel */
            hal_dma_stop(i2s->p_dma_rx );
        }
    }
    /* disable I2S peripheral */
    spi_disable(i2s->periph);
    i2s->state = HAL_I2S_STATE_READY;
}

/*!
    \brief      start I2S module
    \param[in]  i2s: i2s device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_start(hal_i2s_dev_struct *i2s)
{
    i2s_enable(i2s->periph);
}

/*!
    \brief      stop I2S module
    \param[in]  i2s: I2S device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_stop(hal_i2s_dev_struct *i2s)
{
    i2s_disable(i2s->periph);
}

/*!
    \brief      I2S interrupt handler content function
    \param[in]  i2s: I2S device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_irq(hal_i2s_dev_struct *i2s)
{
    __IO uint32_t tmp = 0x00U;
    hal_i2s_user_cb p_func;
    
    if(HAL_I2S_STATE_BUSY_RX == i2s->state){
        if(RESET != spi_i2s_interrupt_flag_get(i2s->periph, SPI_I2S_INT_FLAG_RBNE)){
            i2s->i2s_irq.receive_handler(i2s);
        }

        /* I2S overrun error interrupt occured */
        if(RESET != spi_i2s_interrupt_flag_get(i2s->periph, SPI_I2S_INT_FLAG_RXORERR)){
            /* disable RBNE and ERR interrupt */
            spi_i2s_interrupt_disable(i2s->periph, SPI_I2S_INT_RBNE);
            spi_i2s_interrupt_disable(i2s->periph, SPI_I2S_INT_ERR);

            /* clear overrun error */
            tmp = SPI_DATA(i2s->periph);
            tmp = SPI_STAT(i2s->periph);

            i2s->state = HAL_I2S_STATE_READY;
            /* set the error code and execute error callback */
            i2s->error_code = HAL_I2S_ERROR_OVERRUN;
            p_func = (hal_i2s_user_cb)i2s->error_callback;
            if(NULL != p_func){
                 p_func(i2s);
            }      
        }
    }

    if(HAL_I2S_STATE_BUSY_TX == i2s->state){
        if(RESET != spi_i2s_interrupt_flag_get(i2s->periph, SPI_I2S_INT_FLAG_TBE)){
            i2s->i2s_irq.transmit_handler(i2s);
        }

        /* I2S underrun error interrupt occurred */
        if(RESET != spi_i2s_interrupt_flag_get(i2s->periph, I2S_INT_FLAG_TXURERR)){
            /* disable TBE and ERR interrupt */
            spi_i2s_interrupt_disable(i2s->periph, SPI_I2S_INT_TBE);
            spi_i2s_interrupt_disable(i2s->periph, SPI_I2S_INT_ERR);

            /* clear underrun flag */
            tmp = SPI_STAT((i2s)->periph);

            i2s->state = HAL_I2S_STATE_READY;
            /* set the error code and execute error callback */
            i2s->error_code = HAL_I2S_ERROR_UNDERRUN;
            p_func = (hal_i2s_user_cb)i2s->error_callback;
            if(NULL != p_func){
                p_func(i2s);
            }  
        }
    }
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: the structure that contains callback handlers of I2C interrupt
    \param[out] none
    \retval     none
*/
void hal_i2s_irq_handle_set(hal_i2s_dev_struct *i2s, hal_i2s_irq_struct *p_irq)
{
    if(NULL != p_irq->error_handle ){
        i2s->i2s_irq.error_handle = p_irq->error_handle;
        spi_i2s_interrupt_enable(i2s->periph, SPI_I2S_INT_ERR);
    }else{
        i2s->i2s_irq.error_handle = NULL;
        spi_i2s_interrupt_disable(i2s->periph, SPI_I2S_INT_ERR);
    }

    if(NULL != p_irq->receive_handler ){
        i2s->i2s_irq.receive_handler = p_irq->receive_handler;
        spi_i2s_interrupt_enable(i2s->periph, SPI_I2S_INT_RBNE);
    }else{
        i2s->i2s_irq.receive_handler = NULL;
        spi_i2s_interrupt_disable(i2s->periph, SPI_I2S_INT_RBNE);
    }
        
    if(NULL != p_irq->transmit_handler ){
        i2s->i2s_irq.transmit_handler = p_irq->transmit_handler;
        spi_i2s_interrupt_enable(i2s->periph, SPI_I2S_INT_TBE);
    }else{
        i2s->i2s_irq.transmit_handler = NULL;
        spi_i2s_interrupt_disable(i2s->periph, SPI_I2S_INT_TBE);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  i2s: I2S device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_i2s_irq_handle_all_reset(hal_i2s_dev_struct *i2s)
{
     i2s->i2s_irq.error_handle=NULL;
     i2s->i2s_irq.transmit_handler = NULL;
     i2s->i2s_irq.receive_handler =NULL;
}

/*!
    \brief      I2S DMA transmit handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _i2s_transmit_complete_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2s_dev_struct *p_i2s;
    hal_i2s_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_i2s = (hal_i2s_dev_struct*)p_dma->p_periph;
    p_func = (hal_i2s_user_cb)p_i2s->tx_callback;
    /* DMA normal mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN )){
        spi_dma_disable(p_i2s->periph,SPI_DMA_TRANSMIT);
        p_i2s->txbuffer.length = 0;
        p_i2s->state = HAL_I2S_STATE_READY;
    }
    /* DMA transmit complete callback */
    if(NULL != p_func){
        p_func(p_i2s);
    }
}

/*!
    \brief      I2S DMA receive handler
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _i2s_receive_complete_dma(void *dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2s_dev_struct *p_i2s;
    hal_i2s_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_i2s = (hal_i2s_dev_struct*)p_dma->p_periph;
    p_func = (hal_i2s_user_cb)p_i2s->rx_callback;
    /* DMA normal mode */
    if(SET != (DMA_CHCTL(p_dma->channel) & DMA_CHXCTL_CMEN )){
        spi_dma_disable(p_i2s->periph,SPI_DMA_RECEIVE);
        p_i2s->rxbuffer.length = 0;
        p_i2s->state = HAL_I2S_STATE_READY;
    }
    /* DMA receive complete callback */
    if(NULL != p_func){
        p_func(p_i2s);
    }
}

/*!
    \brief      I2S DMA communication error
    \param[in]  dma: DMA device information structrue
    \param[out] none
    \retval     none
*/
static void _i2s_dma_error(void * dma)
{
    hal_dma_dev_struct *p_dma;
    hal_i2s_dev_struct *p_i2s;
    hal_i2s_user_cb p_func;
    
    p_dma = (hal_dma_dev_struct*)dma;
    p_i2s = (hal_i2s_dev_struct*)p_dma->p_periph;
    p_func = (hal_i2s_user_cb)p_i2s->error_callback;

    /* disable SPI transmit and receive DMA */
    spi_dma_disable(p_i2s->periph,SPI_DMA_RECEIVE);
    spi_dma_disable(p_i2s->periph,SPI_DMA_TRANSMIT);
    p_i2s->txbuffer.length = 0;
    p_i2s->rxbuffer.length = 0;
    p_i2s->state = HAL_I2S_STATE_READY ;
    
    /* error callback */
    if(NULL != p_func){
        p_func(p_i2s);
    }
}

/*!
    \brief      I2S transmit interrupt handler
    \param[in]  i2s: I2S device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2s_transmit_interrupt(void *i2s)
{
    hal_i2s_dev_struct *p_i2s = i2s;
    hal_i2s_user_cb p_func = (hal_i2s_user_cb)p_i2s->tx_callback;
    /* transmit data */
    SPI_DATA(p_i2s->periph) = (*p_i2s->txbuffer.buffer++);
    p_i2s->txbuffer.pos--;

    if(0U == p_i2s->txbuffer.pos){
        /* disable TBE and ERR interrupt */
        spi_i2s_interrupt_disable(p_i2s->periph,SPI_I2S_INT_TBE );
        spi_i2s_interrupt_disable(p_i2s->periph,SPI_I2S_INT_ERR );        
        p_i2s->state = HAL_I2S_STATE_READY;
        
        if(NULL != p_func){
             p_func(p_i2s);
        }    
    }
}

/*!
    \brief      I2S receive interrupt handler
    \param[in]  i2s: I2S device information structure
                    the structure is not necessary to be reconfigured after structrue initialization,
                    the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
static void _i2s_receive_interrupt(void *i2s)
{ 
    hal_i2s_dev_struct *p_i2s = i2s;
    hal_i2s_user_cb p_func = (hal_i2s_user_cb)p_i2s->rx_callback;

    /* receive data */
    (*p_i2s->rxbuffer.buffer++)= SPI_DATA(p_i2s->periph);
    p_i2s->rxbuffer.pos--;
    
    if(0U == p_i2s->rxbuffer.pos){
        /* disable RBNE and ERR interrupt */
        spi_i2s_interrupt_disable(p_i2s->periph,SPI_I2S_INT_RBNE );
        spi_i2s_interrupt_disable(p_i2s->periph,SPI_I2S_INT_ERR );

        p_i2s->state = HAL_I2S_STATE_READY;
        
        if(NULL != p_func){
            p_func(p_i2s);
        }
    }
}
