/*!
    \file    gd32e23x_hal_pmu.h
    \brief   definitions for the PMU
    
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

#ifndef GD32E23X_HAL_PMU_H
#define GD32E23X_HAL_PMU_H

#include "gd32e23x_hal.h"

/* constants definitions */
/* PMU ldo definitions */
#define HAL_PMU_LDO_NORMAL               PMU_LDO_NORMAL          /*!< LDO operates normally when PMU enter deepsleep mode */
#define HAL_PMU_LDO_LOWPOWER             PMU_LDO_LOWPOWER        /*!< LDO work at low power status when PMU enter deepsleep mode */

/* PMU command constants definitions */
#define HAL_WFI_CMD                      WFI_CMD                 /*!< use WFI command */
#define HAL_WFE_CMD                      WFE_CMD                 /*!< use WFE command */

/* interrupt or event mode to monitor the LVD */
#define HAL_PMU_INT_MODE                 ((uint32_t)0x00000000)  /*!< interrupt mode */
#define HAL_PMU_EVENT_MODE               ((uint32_t)0x00000001)  /*!< event mode */

/* the trigger edge of LVD event */
#define HAL_PMU_TRIG_RISING              ((uint32_t)0x00000000)  /*!< rising edge trigger */
#define HAL_PMU_TRIG_FALLING             ((uint32_t)0x00000001)  /*!< falling edge trigger */
#define HAL_PMU_TRIG_BOTH                ((uint32_t)0x00000002)  /*!< rising and falling edge trigger */

/* PMU LDO output voltage select definitions */
#define HAL_PMU_LDO_VOLTAGE_HIGH         PMU_LDOVS_HIGH          /*!< LDO output voltage high mode */
#define HAL_PMU_LDO_VOLTAGE_LOW          PMU_LDOVS_LOW           /*!< LDO output voltage low mode */

typedef void (*hal_pmu_irq_handle_cb)(void);

/* PMU WKUP pin definitions */
typedef enum{
    HAL_PMU_WAKEUP_PIN0 = PMU_WAKEUP_PIN0,                       /*!< WKUP Pin 0 (PA0) enable */
#ifdef GD32E230
    HAL_PMU_WAKEUP_PIN1 = PMU_WAKEUP_PIN1,                       /*!< WKUP Pin 1 (PC13) enable */
#endif /* GD32E230 */
    HAL_PMU_WAKEUP_PIN5 = PMU_WAKEUP_PIN5,                       /*!< WKUP Pin 5 (PB5) enable */
    HAL_PMU_WAKEUP_PIN6 = PMU_WAKEUP_PIN6,                       /*!< WKUP Pin 6 (PB15) enable */
} hal_pmu_wakeup_pin_enum;

/* PMU low voltage detector threshold definitions */
typedef enum{
    HAL_PMU_LVDT_0 = PMU_LVDT_0,                                 /*!< voltage threshold is 2.1V */
    HAL_PMU_LVDT_1 = PMU_LVDT_1,                                 /*!< voltage threshold is 2.3V */
    HAL_PMU_LVDT_2 = PMU_LVDT_2,                                 /*!< voltage threshold is 2.4V */
    HAL_PMU_LVDT_3 = PMU_LVDT_3,                                 /*!< voltage threshold is 2.6V */
    HAL_PMU_LVDT_4 = PMU_LVDT_4,                                 /*!< voltage threshold is 2.7V */
    HAL_PMU_LVDT_5 = PMU_LVDT_5,                                 /*!< voltage threshold is 2.9V */
    HAL_PMU_LVDT_6 = PMU_LVDT_6,                                 /*!< voltage threshold is 3.0V */
    HAL_PMU_LVDT_7 = PMU_LVDT_7,                                 /*!< voltage threshold is 3.1V */
} hal_pmu_low_voltage_enum;

/* PMU LDO mode definitions */
typedef enum{
    LDO_HIGH_VOLTAGE_MODE = 0,                                   /*!< LDO high voltage mode */
    LDO_LOW_VOLTAGE_MODE,                                        /*!< LDO low voltage mode */
} hal_pmu_ldo_mode_enum;

/* PMU LVD structure */
typedef struct {          
    uint32_t int_event_mode;                                     /*!< interrupt or event mode to monitor the LVD */
    uint32_t trig_type;                                          /*!< the trigger edge of LVD event */
    hal_pmu_low_voltage_enum lvd_threshold;                      /*!< PMU low voltage detector threshold */
} hal_pmu_lvd_struct;

/* function declarations */
/* initialize the specified structure */
void hal_pmu_struct_init(void *p_struct);
/* deinitialize PMU */
void hal_pmu_deinit(void);

/* configure EXTI_16 and then configure low voltage detector threshold */
void hal_pmu_lvd_config(hal_pmu_lvd_struct *p_lvd);
/* enable PMU lvd */
void hal_pmu_lvd_enable(void);
/* enable PMU lvd by interrupt method */
/* the function is non-blocking */
int32_t hal_pmu_lvd_enable_interrupt(hal_pmu_irq_handle_cb irq_handle);
/* disable PMU lvd */
void hal_pmu_lvd_disable(void);

/* configure LDO output voltage */
int32_t hal_pmu_ldo_output_config(uint32_t ldo_output);
/* get LDO output voltage */
hal_pmu_ldo_mode_enum hal_pmu_ldo_output_get(void);

/* set PMU mode */
/* PMU work in sleep mode */
void hal_pmu_to_sleepmode(uint8_t sleepmodecmd);
/* PMU work in deepsleep mode */
void hal_pmu_to_deepsleepmode(uint32_t ldo, uint8_t deepsleepmodecmd);
/* PMU work in standby mode */
void hal_pmu_to_standbymode(uint8_t standbymodecmd);

/* enable PMU wakeup pin */
void hal_pmu_wakeup_pin_enable(hal_pmu_wakeup_pin_enum wakeup_pin);
/* disable PMU wakeup pin */
void hal_pmu_wakeup_pin_disable(hal_pmu_wakeup_pin_enum wakeup_pin);

/* backup related functions */
/* enable backup domain write */
void hal_pmu_backup_write_enable(void);
/* disable backup domain write */
void hal_pmu_backup_write_disable(void);

/* enable sleep-on-exit */
void hal_pmu_sleep_on_exit_enable(void);
/* disable sleep-on-exit */
void hal_pmu_sleep_on_exit_disable(void);
/* enable SEVONPEND */
void hal_pmu_sev_on_pend_enable(void);
/* disable SEVONPEND */
void hal_pmu_sev_on_pend_disable(void);

/* PMU interrupt handler content function */
void hal_pmu_irq(void);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_pmu_irq_handle_set(hal_pmu_irq_handle_cb irq_handle);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_pmu_irq_handle_all_reset(void);

#endif /* GD32E23X_HAL_PMU_H */
