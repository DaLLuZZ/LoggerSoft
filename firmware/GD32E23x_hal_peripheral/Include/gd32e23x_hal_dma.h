/*!
    \file    gd32e23x_hal_dma.h
    \brief   definitions for the DMA
    
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

#ifndef GD32E23X_HAL_DMA_H
#define GD32E23X_HAL_DMA_H
#include "gd32e23x_hal.h"

/* DMA */
/* DMA structure type enum */
typedef enum {
    HAL_DMA_INIT_STRUCT,                  /*!< DMA initialize structure */
    HAL_DMA_DEV_STRUCT,                   /*!< DMA device information structure */
    HAL_DMA_IRQ_STRUCT                    /*!< DMA device interrupt callback function pointer structure */
} hal_dma_struct_type_enum;

/* DMA transfer state enum */
typedef enum {
    DMA_TARNSFER_HALF_COMPLETE,           /*!< half transfer */
    DMA_TARNSFER_FULL_COMPLETE            /*!< full transfer */
} hal_dma_transfer_state_enum;

/* DMA initialization structrue */
typedef struct {
    uint32_t periph_width;                /*!< transfer data size of peripheral */
    ControlStatus periph_inc;             /*!< peripheral increasing mode */
    uint32_t memory_width;                /*!< transfer data size of memory */
    ControlStatus memory_inc;             /*!< memory increasing mode */
    uint32_t direction;                   /*!< channel data transfer direction */
    uint32_t number;                      /*!< channel transfer number */
    uint32_t priority;                    /*!< channel priority level */
    uint32_t mode;                        /*!< circulation mode */
} hal_dma_init_struct;

/* DMA device interrupt callback function pointer structure */
typedef struct {          
    hal_irq_handle_cb error_handle;        /*!< channel error interrupt handler pointer */
    hal_irq_handle_cb half_finish_handle;  /*!< channel half transfer finish interrupt handler pointer */
    hal_irq_handle_cb full_finish_handle;  /*!< channel full transfer finish interrupt handler pointer */
} hal_dma_irq_struct;

/* DMA device information structrue */
typedef struct {
    dma_channel_enum        channel;        /*!< DMA channel */
    hal_dma_irq_struct      dma_irq;        /*!< DMA device interrupt callback function pointer structure */
    void                    *p_periph;      /*!< user private periph */
} hal_dma_dev_struct;

/* DMA circulation mode or not */
#define DMA_MODE_NORMAL                     (0x00000000U)                                             /*!< disable circulation mode */
#define DMA_MODE_CIRCULAR                   DMA_CHXCTL_CMEN                                           /*!< enable circulation mode */

/* transfer direction */
#define DMA_DIR_PERIPH_TO_MEMORY            ((uint32_t)0x00000000U)                                   /*!< read from peripheral and write to memory */
#define DMA_DIR_MEMORY_TO_PERIPH            DMA_CHXCTL_DIR                                            /*!< read from memory and write to peripheral */
#define DMA_DIR_MEMORY_TO_MEMORY            DMA_CHXCTL_M2M                                            /*!< read from memory and write to memory */

/* transfer data size of peripheral */
#define DMA_PERIPH_SIZE_8BITS               DMA_PERIPHERAL_WIDTH_8BIT                                 /*!< transfer data size of peripheral is 8-bit */
#define DMA_PERIPH_SIZE_16BITS              DMA_PERIPHERAL_WIDTH_16BIT                                /*!< transfer data size of peripheral is 16-bit */
#define DMA_PERIPH_SIZE_32BITS              DMA_PERIPHERAL_WIDTH_32BIT                                /*!< transfer data size of peripheral is 32-bit */

/* transfer data size of memory */
#define DMA_MEMORY_SIZE_8BITS               DMA_MEMORY_WIDTH_8BIT                                     /*!< transfer data size of memory is 8-bit */
#define DMA_MEMORY_SIZE_16BITS              DMA_MEMORY_WIDTH_16BIT                                    /*!< transfer data size of memory is 16-bit */
#define DMA_MEMORY_SIZE_32BITS              DMA_MEMORY_WIDTH_32BIT                                    /*!< transfer data size of memory is 32-bit */

/* channel priority level */
#define DMA_PRIORITY_LEVEL_LOW              DMA_PRIORITY_LOW                                          /*!< low priority */
#define DMA_PRIORITY_LEVEL_MEDIUM           DMA_PRIORITY_MEDIUM                                       /*!< medium priority */
#define DMA_PRIORITY_LEVEL_HIGH             DMA_PRIORITY_HIGH                                         /*!< high priority */
#define DMA_PRIORITY_LEVEL_ULTRA_HIGH       DMA_PRIORITY_ULTRA_HIGH                                   /*!< ultra high priority */

/* function declarations */
/* bind dma device information and peripheral device information */
#define hal_periph_dma_info_bind(_PERIPH_INFO, _P_DMA_TYPE, _DMA_INFO)      \
                          do{                                               \
                              (_PERIPH_INFO)._P_DMA_TYPE = &(_DMA_INFO);    \
                              (_DMA_INFO).p_periph = &(_PERIPH_INFO);       \
                          }while(0)
/* initialize the DMA structure with the default values */
void hal_dma_struct_init(hal_dma_struct_type_enum struct_type , void *p_struct);
/* deinitialize DMA */
void hal_dma_deinit(hal_dma_dev_struct *dma);
/* initialize DMA channel */
int32_t hal_dma_init(hal_dma_dev_struct *dma, dma_channel_enum channelx,\
                     hal_dma_init_struct *p_dma_init);
/* start DMA normal mode transfer */
int32_t hal_dma_start(hal_dma_dev_struct *dma, uint32_t src_addr,\
                                  uint32_t dst_addr, uint32_t length);
/* start DMA interrupt mode transfer */
int32_t hal_dma_start_interrupt(hal_dma_dev_struct *dma, uint32_t src_addr,\
                               uint32_t dst_addr, uint32_t length,\
                               hal_dma_irq_struct *p_irq);
/* stop DMA transfer */
void hal_dma_stop(hal_dma_dev_struct *dma);
/* polling for transfer complete */
int32_t hal_dma_transfer_poll(hal_dma_dev_struct *dma, hal_dma_transfer_state_enum transfer_state, \
                                    uint32_t timeout_ms);
/* DMA interrupt handler content function,which is merely used in dma_handler*/
void hal_dma_irq(hal_dma_dev_struct *dma);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_dma_irq_handle_set(hal_dma_dev_struct *dma, hal_dma_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_dma_irq_handle_all_reset(hal_dma_dev_struct *dma);

#endif /* GD32E23X_HAL_DMA_H */
