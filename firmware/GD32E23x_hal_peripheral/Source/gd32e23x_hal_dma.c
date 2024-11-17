/*!
    \file    gd32e23x_hal_dma.c
    \brief   DMA driver
    
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

#define _DEV_VALID_ADDRESS           ((uint32_t)0x68000000)

/*!
    \brief      initialize the DMA structure with the default values
    \param[in]  struct_type: refer to hal_dma_struct_type_enum
    \param[in]  p_struct: point to DMA structure that contains the configuration information
    \param[out] none
    \retval     none
*/
void hal_dma_struct_init(hal_dma_struct_type_enum struct_type , void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(struct_type){
    case HAL_DMA_INIT_STRUCT:
        /* initialize DMA initialization structure with the default values */
        ((hal_dma_init_struct*)p_struct)->direction = DMA_DIR_PERIPH_TO_MEMORY;
        ((hal_dma_init_struct*)p_struct)->memory_inc = ENABLE;
        ((hal_dma_init_struct*)p_struct)->memory_width = DMA_MEMORY_SIZE_8BITS;
        ((hal_dma_init_struct*)p_struct)->number = 0;
        ((hal_dma_init_struct*)p_struct)->periph_inc = ENABLE;
        ((hal_dma_init_struct*)p_struct)->periph_width = DMA_PERIPH_SIZE_8BITS;
        ((hal_dma_init_struct*)p_struct)->priority = DMA_PRIORITY_LEVEL_LOW;
        ((hal_dma_init_struct*)p_struct)->mode = DMA_MODE_NORMAL;
       
        break;
    
    case HAL_DMA_DEV_STRUCT:
        /* initialize DMA device information structure with the default values */
        ((hal_dma_dev_struct*)p_struct)->channel = DMA_CH0;
        ((hal_dma_dev_struct*)p_struct)->dma_irq.error_handle = NULL;
        ((hal_dma_dev_struct*)p_struct)->dma_irq.full_finish_handle = NULL;
        ((hal_dma_dev_struct*)p_struct)->dma_irq.half_finish_handle = NULL;
        
        break;
            
    case HAL_DMA_IRQ_STRUCT:
        /* initialize DMA device interrupt callback function structure with the default values */
        ((hal_dma_irq_struct*)p_struct)->error_handle = NULL;
        ((hal_dma_irq_struct*)p_struct)->full_finish_handle = NULL;
        ((hal_dma_irq_struct*)p_struct)->half_finish_handle = NULL;
        
        break;
    
    default:
        HAL_DEBUGE("parameter [struct_type] value is invalid");
    }
}

/*!
    \brief      deinitialize DMA
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_dma_deinit(hal_dma_dev_struct *dma)
{
    /* deinitialize DMA a channel registers */
    dma_deinit(dma->channel);
    hal_dma_struct_init(HAL_DMA_DEV_STRUCT, dma);
    /* reset DMA device interrupt callback functions */
    dma->dma_irq.error_handle = NULL;
    dma->dma_irq.full_finish_handle = NULL;
    dma->dma_irq.half_finish_handle = NULL;
}

/*!
    \brief      initialize DMA channel
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channelx: specify which DMA channel is initialized
      \arg        DMA_CHx(x=0..4)
    \param[in]  p_dma_init: the initialization data needed to initialize DMA channel
                  periph_width: DMA_PERIPH_SIZE_8BITS, DMA_PERIPH_SIZE_16BITS, DMA_PERIPH_SIZE_32BITS
                  periph_inc: DMA_PERIPH_INCREASE_ENABLE,DMA_PERIPH_INCREASE_DISABLE 
                  memory_width: DMA_MEMORY_WIDTH_8BIT,DMA_MEMORY_WIDTH_16BIT,DMA_MEMORY_WIDTH_32BIT
                  memory_inc: DMA_MEMORY_INCREASE_ENABLE,DMA_MEMORY_INCREASE_DISABLE
                  direction: DMA_DIR_PERIPH_TO_MEMORY, DMA_DIR_MEMORY_TO_PERIPH, DMA_DIR_MEMORY_TO_MEMORY
                  number: the number of remaining data to be transferred by the DMA
                  priority: DMA_PRIORITY_LEVEL_LOW,DMA_PRIORITY_LEVEL_MEDIUM,DMA_PRIORITY_LEVEL_HIGH,DMA_PRIORITY_LEVEL_ULTRA_HIGH
                  mode:  DMA_MODE_NORMAL, DMA_MODE_CIRCULAR
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_dma_init(hal_dma_dev_struct *dma, dma_channel_enum channelx,\
                     hal_dma_init_struct *p_dma_init)
{
    uint32_t ctl;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check DMA pointer and p_init address */
    if((NULL == dma) && (NULL == p_dma_init)){
        HAL_DEBUGE("pointer [dma] or [p_dma_init] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check DMA mode and direction parameters */
    if((DMA_MODE_CIRCULAR == p_dma_init->mode)&&(DMA_DIR_MEMORY_TO_MEMORY == p_dma_init->direction)){
        HAL_DEBUGI("circular mode is invalid due to 'memory to memory' has been configured");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    dma->channel = channelx;
    dma_channel_disable(dma->channel);
       
    /* configure the number of remaining data to be transferred */
    DMA_CHCNT(dma->channel) = p_dma_init->number;
    
    /* configure peripheral transfer width,memory transfer width,channel priority */
    ctl = DMA_CHCTL(dma->channel);
    ctl &= ~(DMA_CHXCTL_PWIDTH | DMA_CHXCTL_MWIDTH | DMA_CHXCTL_PRIO | DMA_CHXCTL_M2M | \
             DMA_CHXCTL_DIR | DMA_CHXCTL_CMEN);
    ctl |= (p_dma_init->periph_width | p_dma_init->memory_width | p_dma_init->priority | \
            p_dma_init->direction | p_dma_init->mode);
    DMA_CHCTL(dma->channel) = ctl;

    /* configure peripheral increasing mode */
    if(ENABLE == p_dma_init->periph_inc){
        DMA_CHCTL(dma->channel) |= DMA_CHXCTL_PNAGA;
    }else{
        DMA_CHCTL(dma->channel) &= ~DMA_CHXCTL_PNAGA;
    }

    /* configure memory increasing mode */
    if(ENABLE == p_dma_init->memory_inc){
        DMA_CHCTL(dma->channel) |= DMA_CHXCTL_MNAGA;
    }else{
        DMA_CHCTL(dma->channel) &= ~DMA_CHXCTL_MNAGA;
    }
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start DMA normal mode transfer
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  src_addr: the source memory buffer address
    \param[in]  dst_addr: the destination memory buffer address
    \param[in]  length: the number of data to be transferred from source to destination
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_dma_start(hal_dma_dev_struct *dma, uint32_t src_addr, \
                                  uint32_t dst_addr, uint32_t length)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check DMA pointer address */
    if(NULL == dma){
        HAL_DEBUGE("pointer [dma] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable DMA channel */
    dma_channel_disable(dma->channel);
    
#if (1 == HAL_PARAMETER_CHECK)
    if(0 == (src_addr & _DEV_VALID_ADDRESS)){
        HAL_DEBUGE("parameter [src_addr] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    if(0 == (dst_addr & _DEV_VALID_ADDRESS)){
        HAL_DEBUGE("parameter [dst_addr] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check the direction of the data transfer */
    if(RESET != (DMA_CHCTL(dma->channel) & DMA_CHXCTL_DIR)){
    /* configure the transfer destination and source address */
    dma_memory_address_config(dma->channel, src_addr);
        dma_periph_address_config(dma->channel, dst_addr);
    }else{
        /* configure the transfer destination and source address */
        dma_memory_address_config(dma->channel, dst_addr);
        dma_periph_address_config(dma->channel, src_addr);
    }
    /* configure the transfer number */
    dma_transfer_number_config(dma->channel, length);
    
    /* enable DMA channel */
    dma_channel_enable(dma->channel);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      start DMA interrupt mode transfer
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  src_addr: the source memory buffer address
    \param[in]  dst_addr: the destination memory buffer address
    \param[in]  length: the number of data to be transferred from source to destination
    \param[in]  p_irq: device interrupt callback function structure
                  error_handle: channel error interrupt handler pointer
                  half_finish_handle: channel half transfer finish interrupt handler pointer
                  full_finish_handle: channel full transfer finish interrupt handler pointer
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h 
*/
int32_t hal_dma_start_interrupt(hal_dma_dev_struct *dma, uint32_t src_addr, \
                               uint32_t dst_addr, uint32_t length, \
                               hal_dma_irq_struct *p_irq)
{
    /* disable DMA channel */
    dma_channel_disable(dma->channel);

#if (1 == HAL_PARAMETER_CHECK)
    /* check the DMA pointer address and the number length parameter */
    if((NULL == dma) || (0 == length)){
        HAL_DEBUGE("parameter [dma] and [length] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check src_addr parameter */
    if(0 == (src_addr & _DEV_VALID_ADDRESS)){
        HAL_DEBUGE("parameter [src_addr] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check dst_addr parameter */
    if(0 == (dst_addr & _DEV_VALID_ADDRESS)){
        HAL_DEBUGE("parameter [dst_addr] value is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* set the error handler pointer and enable the channel error interrupt */
    if(NULL != p_irq->error_handle){
        dma->dma_irq.error_handle = p_irq->error_handle;
        dma_interrupt_enable(dma->channel, DMA_INT_ERR);
    }else{
        dma_interrupt_disable(dma->channel, DMA_INT_ERR);
    }

    /* set the full finish handler pointer and enable the channel full transfer finish  interrupt */
    if(NULL != p_irq->full_finish_handle){
        dma->dma_irq.full_finish_handle = p_irq->full_finish_handle;
        dma_interrupt_enable(dma->channel, DMA_INT_FTF);
    }else{
        dma_interrupt_disable(dma->channel, DMA_INT_FTF);
    }

    /* set the half finish handler pointer and enable the channel half transfer finish interrupt */
    if(NULL != p_irq->half_finish_handle){
        dma->dma_irq.half_finish_handle = p_irq->half_finish_handle;
        dma_interrupt_enable(dma->channel, DMA_INT_HTF);
    }else{
        dma_interrupt_disable(dma->channel, DMA_INT_HTF);
    }
    
    /* clear all flags */
    dma_flag_clear(dma->channel, DMA_FLAG_G);
    
    /* check the direction of the data transfer */
    if(RESET != (DMA_CHCTL(dma->channel) & DMA_CHXCTL_DIR)){
        /* configure the transfer destination and source address */
        dma_memory_address_config(dma->channel, src_addr);
        dma_periph_address_config(dma->channel, dst_addr);
    }else{
        /* configure the transfer destination and source address */
        dma_memory_address_config(dma->channel, dst_addr);
        dma_periph_address_config(dma->channel, src_addr);
    }
    /* configure the transfer number */
    dma_transfer_number_config(dma->channel, length);

    /* enable DMA channel */
    dma_channel_enable(dma->channel);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      stop DMA transfer
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_dma_stop(hal_dma_dev_struct *dma)
{
    /* disable the channel */
    dma_channel_disable(dma->channel);
    /* disable DMA IT */
    dma_interrupt_disable(dma->channel, DMA_INT_ERR | DMA_INT_HTF | DMA_INT_FTF);
    /* reset the interrupt handle */
    dma->dma_irq.error_handle = NULL;
    dma->dma_irq.full_finish_handle = NULL;
    dma->dma_irq.half_finish_handle = NULL;
    /* clear all flags */
    dma_flag_clear(dma->channel, DMA_FLAG_G);
}

/*!
    \brief      polling for transfer complete
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  transfer_state: refer to hal_dma_transfer_state_enum
    \param[in]  timeout_ms: time out duration
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_NO_SUPPORT, HAL_ERR_TIMEOUT, 
                            HAL_ERR_ADDRESS, HAL_ERR_HARDWARE, details refer to gd32e23x_hal.h
*/
int32_t hal_dma_transfer_poll(hal_dma_dev_struct *dma, hal_dma_transfer_state_enum transfer_state, \
                                    uint32_t timeout_ms)
{
    uint32_t flag;
    uint32_t tick_start;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == dma){
        HAL_DEBUGE("parameter [dma] value is invalid");
        return HAL_ERR_ADDRESS;
    }
    
    /* check the cirulation mode of the data transfer */    
    if(RESET != (DMA_CHCTL(dma->channel) & DMA_CHXCTL_CMEN)){
    /* polling mode is not supported in circular mode */
        HAL_DEBUGE("DMA poll function is invalid in circulation mode");
        return HAL_ERR_NO_SUPPORT;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* get the transfer flag */
    if(DMA_TARNSFER_HALF_COMPLETE == transfer_state){
        flag = DMA_FLAG_ADD(DMA_FLAG_HTF, dma->channel);
    }else{
        flag = DMA_FLAG_ADD(DMA_FLAG_FTF, dma->channel);
    }
    
    /* set timeout */
    tick_start = hal_basetick_count_get();
    
    while(RESET == (DMA_INTF & flag)){
        if(SET == dma_flag_get(dma->channel, DMA_FLAG_ERR)){
            /* when error occurs, clear all flags and output error message */
            dma_flag_clear(dma->channel, DMA_FLAG_G);
            HAL_DEBUGE("dma transfer error, poll stop");
            return HAL_ERR_HARDWARE;
        }
        /* check for the timeout */
        if(HAL_TIMEOUT_FOREVER != timeout_ms){
            if(SET == hal_basetick_timeout_check(tick_start, timeout_ms)){
                /* when timeout occurs, output timeout warning message */
                HAL_DEBUGW("dma transfer state poll timeout");
                return HAL_ERR_TIMEOUT;
            }            
        }
    }
    
    DMA_INTC = DMA_FLAG_G;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      DMA interrupt handler content function,which is merely used in dma_handler
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_dma_irq(hal_dma_dev_struct *dma)
{
    /* full transfer finish interrupt handler */
    if(SET == dma_interrupt_flag_get(dma->channel, DMA_INT_FLAG_FTF)){
        dma_interrupt_flag_clear(dma->channel, DMA_INT_FLAG_FTF);
        if(dma->dma_irq.full_finish_handle != NULL){
            dma->dma_irq.full_finish_handle(dma);
        }
    }

    /* half transfer finish interrupt handler */    
    if(SET == dma_interrupt_flag_get(dma->channel, DMA_INT_FLAG_HTF)){
        dma_interrupt_flag_clear(dma->channel, DMA_INT_FLAG_HTF);
        if(dma->dma_irq.half_finish_handle != NULL){
            dma->dma_irq.half_finish_handle(dma);
        }
    }
    
    /* error interrupt handler */
    if(SET == dma_interrupt_flag_get(dma->channel, DMA_INT_FLAG_ERR)){
        dma_interrupt_flag_clear(dma->channel, DMA_INT_FLAG_G);
        if(dma->dma_irq.error_handle != NULL){
            dma->dma_irq.error_handle(dma);
        }
    }
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  p_irq: point to DMA interrupt callback function pointer structure
                  The structure member can be assigned as following parameters:
      \arg        hal_irq_handle_cb function pointer: the function is user-defined,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_dma_irq_handle_set(hal_dma_dev_struct *dma, hal_dma_irq_struct *p_irq)
{ 
    /* error interrupt handler set */
    if(NULL != p_irq->error_handle){
        dma->dma_irq.error_handle = p_irq->error_handle;
        dma_interrupt_enable(dma->channel, DMA_INT_ERR);
    }else{
        dma->dma_irq.error_handle = NULL;
        dma_interrupt_disable(dma->channel, DMA_INT_ERR);
    }
 
    /* half transfer finish interrupt handler set */
    if(NULL != p_irq->half_finish_handle){
        dma->dma_irq.half_finish_handle = p_irq->half_finish_handle;
        dma_interrupt_enable(dma->channel, DMA_INT_HTF);
    }else{
        dma->dma_irq.half_finish_handle = NULL;
        dma_interrupt_disable(dma->channel, DMA_INT_HTF);
    }
    
    /* full transfer finish interrupt handler set */
    if(NULL != p_irq->full_finish_handle){
        dma->dma_irq.full_finish_handle = p_irq->full_finish_handle;
        dma_interrupt_enable(dma->channel, DMA_INT_FTF);
    }else{
        dma->dma_irq.full_finish_handle = NULL;
        dma_interrupt_disable(dma->channel, DMA_INT_FTF);
    }
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  dma: DMA device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     none
*/
void hal_dma_irq_handle_all_reset(hal_dma_dev_struct *dma)
{ 
    /* DMA interrupt handler reset */
    dma->dma_irq.error_handle = NULL;
    dma->dma_irq.half_finish_handle = NULL;
    dma->dma_irq.full_finish_handle = NULL;
}
