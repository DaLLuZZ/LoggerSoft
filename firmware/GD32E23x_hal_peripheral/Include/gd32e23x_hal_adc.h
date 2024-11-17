/*!
    \file    gd32e23x_hal_adc.h
    \brief   definitions for the ADC

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

#ifndef GD32E23X_HAL_ADC_H
#define GD32E23X_HAL_ADC_H

#include "gd32e23x_hal.h"
#include "gd32e23x_hal_dma.h"

/* the callback of ADC interrupt declaration */
typedef void (*hal_adc_dma_handle_cb)(void *ptr);

/* ADC structure type enum */
typedef enum {
    HAL_ADC_INIT_STRUCT,                                                    /*!< ADC initialization structure */
    HAL_ADC_REGULARCH_INIT_STRUCT,                                          /*!< ADC regular channel initialization structure */
    HAL_ADC_REGULARCH_CONFIG_STRUCT,                                        /*!< ADC regular channel configuration structure */
    HAL_ADC_INSERTEDCH_INIT_STRUCT,                                         /*!< ADC inserted channel initialization structure */
    HAL_ADC_INSERTEDCH_CONFIG_STRUCT,                                       /*!< ADC inserted channel configuration structure */
    HAL_ADC_IRQ_STRUCT,                                                     /*!< ADC device interrupt callback function pointer structure */
    HAL_ADC_DMA_HANDLE_CB_STRUCT,                                           /*!< ADC DMA callback function pointer structure */
    HAL_ADC_WATCHDOG_STRUCT,                                                /*!< ADC watchdog configuration structure */
    HAL_ADC_DEV_STRUCT,                                                     /*!< ADC device information structrue */
    HAL_ADC_OVERSAMPLE_STRUCT,                                              /*!< ADC oversampling structrue */
} hal_adc_struct_type_enum;

/* ADC state type enum */
typedef enum {
    ADC_STATE_RESET         = (uint32_t)0x00000000U,                        /*!< ADC is not initialized or disabled */
    ADC_STATE_READY         = (uint32_t)0x00000001U,                        /*!< ADC is ready */
    ADC_STATE_BUSY_SYSTEM   = (uint32_t)0x00000002U,                        /*!< ADC is busy to internal system (initialization, calibration) */
    ADC_STATE_TIMEOUT       = (uint32_t)0x00000004U,                        /*!< ADC timeout occurs */
    ADC_STATE_REGULAR_BUSY  = (uint32_t)0x00000010U,                        /*!< a conversion is ongoing on regular group */
    ADC_STATE_REGULAR_EOC   = (uint32_t)0x00000020U,                        /*!< conversion data available on regular group */
    ADC_STATE_INSERTED_BUSY = (uint32_t)0x00000100U,                        /*!< a conversion is ongoing on inserted group */
    ADC_STATE_INSERTED_EOC  = (uint32_t)0x00000200U,                        /*!< conversion data available on inserted group */
    ADC_STATE_WATCHDOG      = (uint32_t)0x00001000U,                        /*!< analog watchdog  */
} hal_adc_state_enum;

/* ADC error type enum */
typedef enum {
    ADC_ERROR_NONE   = (uint32_t)0x00U,                                     /*!< no error */
    ADC_ERROR_SYSTEM = (uint32_t)0x01U,                                     /*!< ADC internal error: if problem of clocking, enable/disable, wrong state */
    ADC_ERROR_DMA    = (uint32_t)0x02U,                                     /*!< DMA transfer error */
    ADC_ERROR_CONFIG = (uint32_t)0x04U,                                     /*!< configuration error occurs */
} hal_adc_error_enum;

/* ADC analog watchdog type enum */
typedef enum {
    ADC_WATCHDOG_NONE            = (uint32_t)0x00000000U,                   /*!< no watchdog mode */
    ADC_WATCHDOG_SINGLE_REGULAR  = (uint32_t)0x00800200U,                   /*!< regular group single channel watchdog mode */
    ADC_WATCHDOG_SINGLE_INSERTED = (uint32_t)0x00400200U,                   /*!< inserted group single channel watchdog mode */
    ADC_WATCHDOG_SINGLE_REGINS   = (uint32_t)0x00C00200U,                   /*!< regular or inserted group single channel watchdog mode */
    ADC_WATCHDOG_ALL_REGULAR     = (uint32_t)0x00800000U,                   /*!< regular group all channel watchdog mode */
    ADC_WATCHDOG_ALL_INSERTED    = (uint32_t)0x00400000U,                   /*!< inserted group all channel watchdog mode */
    ADC_WATCHDOG_ALL_REGINS      = (uint32_t)0x00C00000U,                   /*!< regular and inserted group all channel watchdog mode */
} hal_adc_watchdog_mode_enum;

/* ADC regular channel sequence type enum */
typedef enum {
    ADC_REGULAR_SEQUENCE_0  = (uint8_t)0x00U,                               /*!< ADC regular channel sequence 0 */
    ADC_REGULAR_SEQUENCE_1  = (uint8_t)0x01U,                               /*!< ADC regular channel sequence 1 */
    ADC_REGULAR_SEQUENCE_2  = (uint8_t)0x02U,                               /*!< ADC regular channel sequence 2 */
    ADC_REGULAR_SEQUENCE_3  = (uint8_t)0x03U,                               /*!< ADC regular channel sequence 3 */
    ADC_REGULAR_SEQUENCE_4  = (uint8_t)0x04U,                               /*!< ADC regular channel sequence 4 */
    ADC_REGULAR_SEQUENCE_5  = (uint8_t)0x05U,                               /*!< ADC regular channel sequence 5 */
    ADC_REGULAR_SEQUENCE_6  = (uint8_t)0x06U,                               /*!< ADC regular channel sequence 6 */
    ADC_REGULAR_SEQUENCE_7  = (uint8_t)0x07U,                               /*!< ADC regular channel sequence 7 */
    ADC_REGULAR_SEQUENCE_8  = (uint8_t)0x08U,                               /*!< ADC regular channel sequence 8 */
    ADC_REGULAR_SEQUENCE_9  = (uint8_t)0x09U,                               /*!< ADC regular channel sequence 9 */
    ADC_REGULAR_SEQUENCE_10 = (uint8_t)0x0AU,                               /*!< ADC regular channel sequence 10 */
    ADC_REGULAR_SEQUENCE_11 = (uint8_t)0x0BU,                               /*!< ADC regular channel sequence 11 */
    ADC_REGULAR_SEQUENCE_12 = (uint8_t)0x0CU,                               /*!< ADC regular channel sequence 12 */
    ADC_REGULAR_SEQUENCE_13 = (uint8_t)0x0DU,                               /*!< ADC regular channel sequence 13 */
    ADC_REGULAR_SEQUENCE_14 = (uint8_t)0x0EU,                               /*!< ADC regular channel sequence 14 */
    ADC_REGULAR_SEQUENCE_15 = (uint8_t)0x0FU,                               /*!< ADC regular channel sequence 15 */
} hal_adc_regularch_sequence_enum;

/* ADC inserted channel sequence type enum */
typedef enum {
    ADC_INSERTED_SEQUENCE_0 = (uint8_t)0x00U,                               /*!< ADC inserted channel sequence 0 */
    ADC_INSERTED_SEQUENCE_1 = (uint8_t)0x01U,                               /*!< ADC inserted channel sequence 1 */
    ADC_INSERTED_SEQUENCE_2 = (uint8_t)0x02U,                               /*!< ADC inserted channel sequence 2 */
    ADC_INSERTED_SEQUENCE_3 = (uint8_t)0x03U,                               /*!< ADC inserted channel sequence 3 */
} hal_adc_insertedch_sequence_enum;

/* ADC oversampling shift type enum */
typedef enum {
    ADC_OVERSAMPLE_SHIFT_NONE = (uint16_t)(ADC_OVERSAMPLING_SHIFT_NONE),    /*!< no oversampling shift */
    ADC_OVERSAMPLE_SHIFT_1B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_1B),      /*!< 1-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_2B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_2B),      /*!< 2-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_3B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_3B),      /*!< 3-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_4B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_4B),      /*!< 4-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_5B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_5B),      /*!< 5-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_6B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_6B),      /*!< 6-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_7B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_7B),      /*!< 7-bit oversampling shift */
    ADC_OVERSAMPLE_SHIFT_8B   = (uint16_t)(ADC_OVERSAMPLING_SHIFT_8B),      /*!< 8-bit oversampling shift */
} hal_adc_oversample_shift_enum;

/* ADC oversampling ratio */
typedef enum {
    ADC_OVERSAMPLE_RATIO_MUL2   = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL2),   /*!< oversampling ratio multiple 2 */
    ADC_OVERSAMPLE_RATIO_MUL4   = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL4),   /*!< oversampling ratio multiple 4 */
    ADC_OVERSAMPLE_RATIO_MUL8   = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL8),   /*!< oversampling ratio multiple 8 */
    ADC_OVERSAMPLE_RATIO_MUL16  = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL16),  /*!< oversampling ratio multiple 16 */
    ADC_OVERSAMPLE_RATIO_MUL32  = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL32),  /*!< oversampling ratio multiple 32 */
    ADC_OVERSAMPLE_RATIO_MUL64  = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL64),  /*!< oversampling ratio multiple 64 */
    ADC_OVERSAMPLE_RATIO_MUL128 = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL128), /*!< oversampling ratio multiple 128 */
    ADC_OVERSAMPLE_RATIO_MUL256 = (uint8_t)(ADC_OVERSAMPLING_RATIO_MUL256), /*!< oversampling ratio multiple 256 */
} hal_adc_oversample_ratio_enum;

/* ADC oversampling structure definition */ 
typedef struct
{
    ControlStatus                    oversample_mode;               /*!< oversampling mode */
    hal_adc_oversample_ratio_enum    oversample_ratio;              /*!< ADC oversampling ratio */
    hal_adc_oversample_shift_enum    oversample_shift;              /*!< ADC oversampling shift */
    uint8_t                          oversample_triggermode;        /*!< ADC oversampling trigger mode */
}hal_adc_oversample_struct;

/* ADC initialization structure */
typedef struct
{
    uint32_t                         resolution_select;             /*!< ADC data resolution select */
    uint32_t                         data_alignment;                /*!< ADC data alignment */
    ControlStatus                    scan_mode;                     /*!< scan mode */
    hal_adc_oversample_struct        oversample_config;             /*!< oversampling parameter config */
}hal_adc_init_struct;

/* ADC regular channel initialization structure */
typedef struct
{
    uint32_t                         length;                        /*!< ADC regular channel length,0~15 */
    uint32_t                         exttrigger_select;             /*!< ADC regular channel group external trigger select */
    ControlStatus                    continuous_mode;               /*!< continuous mode */
    ControlStatus                    discontinuous_mode;            /*!< discontinuous mode */
    uint32_t                         discontinuous_channel_length;  /*!< discontinuous mode length */
}hal_adc_regularch_init_struct;

/* ADC regular channel configuration structure */
typedef struct
{
    uint8_t                          regular_channel;               /*!< ADC regular channel */
    hal_adc_regularch_sequence_enum  regular_sequence;              /*!< ADC regular channel sequence */
    uint32_t                         sample_time;                   /*!< ADC regular channel sampling time */
}hal_adc_regularch_config_struct;

/* ADC inserted channel initialization structure */
typedef struct
{
    uint32_t                         length;                        /*!< ADC inserted channel group,0~3 */
    uint32_t                         exttrigger_select;             /*!< ADC inserted channel group external trigger select */
    ControlStatus                    auto_convert;                  /*!< ADC inserted channel convert automatically */
    ControlStatus                    discontinuous_mode;            /*!< ADC inserted channel discontinuous mode */
}hal_adc_insertedch_init_struct;

/* ADC inserted channel configuration structure */
typedef struct
{
    uint8_t                          inserted_channel;              /*!< ADC inserted channel */
    hal_adc_insertedch_sequence_enum inserted_sequence;             /*!< ADC inserted channel sequence */
    uint32_t                         sample_time;                   /*!< ADC inserted channel sampling time */
    uint16_t                         inserted_offset;               /*!< ADC inserted channel offset */
}hal_adc_insertedch_config_struct;

/* ADC device interrupt callback function pointer structure */
typedef struct {
    hal_irq_handle_cb                adc_eoc_handle;                /*!< EOC interrupt handler function */
    hal_irq_handle_cb                adc_eoic_handle;               /*!< EOIC interrupt handler function */
    hal_irq_handle_cb                adc_watchdog_handle;           /*!< watchdog event interrupt handler function */
} hal_adc_irq_struct;

/* ADC DMA callback function pointer structure */
typedef struct {
    hal_adc_dma_handle_cb            transcom_handle;               /*!< ADC DMA transfer complete interrupt handler function */
    hal_adc_dma_handle_cb            error_handle;                  /*!< ADC DMA error interrupt handler function */
} hal_adc_dma_handle_cb_struct;

/* ADC watchdog configuration structure */
typedef struct {
    hal_adc_watchdog_mode_enum       watchdog_mode;                 /*!< ADC analog watchdog mode */
    uint8_t                          single_channel;                /*!< ADC analog watchdog channel */
    uint16_t                         low_threshold;                 /*!< ADC analog watchdog low threshold */
    uint16_t                         high_threshold;                /*!< ADC analog watchdog high threshold */
} hal_adc_watchdog_struct;

/* ADC device information structrue */
typedef struct {
    hal_adc_irq_struct               adc_irq;                       /*!< ADC device interrupt callback function pointer structure */
    hal_dma_dev_struct               *p_dma_adc;                    /*!< DMA device information structrue */
    hal_adc_dma_handle_cb_struct     adc_dma;                       /*!< DMA callback function pointer structure */
    hal_adc_error_enum               error_state;                   /*!< ADC error state */
    hal_adc_state_enum               state;                         /*!< ADC state */
} hal_adc_dev_struct;

/* ADC internal channel */
#define ADC_CHANNEL_TEMPSENSOR                  ADC_CHANNEL_16                                       /*!< ADC temperature sensor channel */
#define ADC_CHANNEL_VREFINT                     ADC_CHANNEL_17                                       /*!< ADC VREFINT channel */

/* ADC triggered oversampling */
#define ADC_OVERSAMPLE_ALL_CONVERT              (uint8_t)(ADC_OVERSAMPLING_ALL_CONVERT)              /*!< all oversampled conversions for a channel are done consecutively after a trigger */
#define ADC_OVERSAMPLE_ONE_CONVERT              (uint8_t)(ADC_OVERSAMPLING_ONE_CONVERT)              /*!< each oversampled conversion for a channel needs a trigger */

/* get ADC init value */
#define __HAL_ADC_GET_SCAN_MODE                 (uint32_t)((ADC_CTL0) & (ADC_CTL0_SM))               /*!< get ADC scan mode init value */
#define __HAL_ADC_GET_CONTINUOUS_MODE           (uint32_t)((ADC_CTL1) & (ADC_CTL1_CTN))              /*!< get ADC continuous mode init value */
#define __HAL_ADC_GET_TEMPVREF_ENABLE           (uint32_t)((ADC_CTL1) & (ADC_CTL1_TSVREN))           /*!< get ADC internal channel enable init value */
#define __HAL_ADC_GET_OVERSAMPLE_ENABLE         (uint32_t)((ADC_OVSAMPCTL) & (ADC_OVSAMPCTL_OVSEN))  /*!< get ADC oversampling enable init value */
#define __HAL_ADC_GET_DMA_MODE                  (uint32_t)((ADC_CTL1) & (ADC_CTL1_DMA))              /*!< get ADC DMA mode init value */

#define __HAL_ADC_GET_REGULARCH_LENGTH          (uint32_t)((ADC_RSQ0) & (ADC_RSQ0_RL))               /*!< get ADC regular group length value */
#define __HAL_ADC_GET_REGULARCH_EXTTRIGGER      (uint32_t)((ADC_CTL1) & (ADC_CTL1_ETSRC))            /*!< get ADC regular external trigger select */

#define __HAL_ADC_GET_INSERTEDCH_LENGTH         (uint32_t)((ADC_RSQ0) & (ADC_RSQ0_RL))               /*!< get ADC inserted group length value */
#define __HAL_ADC_GET_INSERTEDCH_EXTTRIGGER     (uint32_t)((ADC_CTL1) & (ADC_CTL1_ETSIC))            /*!< get ADC inserted external trigger select */
#define __HAL_ADC_GET_INSERTEDCH_AUTOCONV       (uint32_t)((ADC_CTL0) & (ADC_CTL0_ICA))              /*!< get ADC inserted group automatic conversion */

/* function declarations */
/* initialization functions */
/* initialize the ADC structure with the default values */
void hal_adc_struct_init(hal_adc_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize ADC device structure and init structure */
int32_t hal_adc_deinit(hal_adc_dev_struct *adc);
/* initialize ADC */
int32_t hal_adc_init(hal_adc_dev_struct *adc, hal_adc_init_struct *p_init);
/* ADC calibration */
int32_t hal_adc_calibration(hal_adc_dev_struct *adc);

/* ADC regular channel */
/* initialize adc regular channel */
int32_t hal_adc_regular_channel_init(hal_adc_dev_struct *adc, hal_adc_regularch_init_struct *p_rinit);
/* configure ADC regular channel */
int32_t hal_adc_regular_channel_config(hal_adc_dev_struct *adc, hal_adc_regularch_config_struct *p_rchannel);
/* start ADC module function */
int32_t hal_adc_start(hal_adc_dev_struct *adc);
/* stop hal module function */
int32_t hal_adc_stop(hal_adc_dev_struct *adc);
/* polling for ADC regular group conversion */
int32_t hal_adc_regular_conversion_poll(hal_adc_dev_struct *adc, uint32_t timeout_ms);
/* enable ADC regular group software trigger */
void hal_adc_regular_software_trigger_enable(hal_adc_dev_struct *adc);
/* start ADC EOC interrupt */
int32_t hal_adc_start_interrupt(hal_adc_dev_struct *adc, hal_adc_irq_struct *p_irq);
/* stop ADC EOC interrupt */
int32_t hal_adc_stop_interrupt(hal_adc_dev_struct *adc);

/* ADC DMA */
/* enable ADC and start the conversion of regular group with DMA */
int32_t hal_adc_start_dma(hal_adc_dev_struct *adc, uint32_t* pdata, uint32_t length, hal_adc_dma_handle_cb_struct *dmacb);
/* stop the conversion of regular group, disable ADC DMA mode and disable ADC */
int32_t hal_adc_stop_dma(hal_adc_dev_struct *adc);

/* ADC inserted channel */
/* initialize ADC inserted channel */
int32_t hal_adc_inserted_channel_init(hal_adc_dev_struct *adc, hal_adc_insertedch_init_struct *p_iinit);
/* configure ADC inserted channel */
int32_t hal_adc_inserted_channel_config(hal_adc_dev_struct *adc, hal_adc_insertedch_config_struct *p_ichannel);
/* enable ADC and start the conversion of inserted group */
int32_t hal_adc_inserted_start(hal_adc_dev_struct *adc);
/* stop the conversion of inserted group and disable ADC */
int32_t hal_adc_inserted_stop(hal_adc_dev_struct *adc);
/* polling for ADC inserted group conversion */
int32_t hal_adc_inserted_conversion_poll(hal_adc_dev_struct *adc, uint32_t timeout_ms);
/* enable ADC inserted group software trigger */
void hal_adc_inserted_software_trigger_enable(hal_adc_dev_struct *adc);
/* start ADC EOIC interrupt */
int32_t hal_adc_inserted_start_interrupt(hal_adc_dev_struct *adc, hal_adc_irq_struct *p_irq);
/* stop ADC EOIC interrupt */
int32_t hal_adc_inserted_stop_interrupt(hal_adc_dev_struct *adc);

/* ADC watchdog */
/* ADC watchdog config */
int32_t hal_adc_watchdog_config(hal_adc_dev_struct *adc, hal_adc_watchdog_struct *p_watchdog);
/* enable ADC watchdog interrupt */
int32_t hal_adc_watchdog_interrupt_enable(hal_adc_dev_struct *adc, hal_adc_irq_struct *p_irq);
/* disable ADC watchdog interrupt */
int32_t hal_adc_watchdog_interrupt_disable(hal_adc_dev_struct *adc);
/* polling for ADC watchdog event conversion */
int32_t hal_adc_watchdog_event_poll(hal_adc_dev_struct *adc, uint32_t timeout_ms);

/* interrupt handle */
/* ADC interrupt handler content function, which is merely used in ADC_CMP_IRQHandler */
void hal_adc_irq(hal_adc_dev_struct *adc);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_adc_irq_handle_set(hal_adc_dev_struct *adc, hal_adc_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_adc_irq_handle_all_reset(hal_adc_dev_struct *adc);

/* get ADC value and state */
/* get regular group conversion result */
uint16_t hal_adc_regular_value_get(hal_adc_dev_struct *adc);
/* get inserted group conversion result */
uint16_t hal_adc_inserted_value_get(hal_adc_dev_struct *adc, uint8_t inschannel_sequence);
/* get ADC error */
uint32_t hal_adc_error_get(hal_adc_dev_struct *adc);
/* get ADC state */
uint32_t hal_adc_state_get(hal_adc_dev_struct *adc);

#endif /* GD32E23X_HAL_ADC_H */
