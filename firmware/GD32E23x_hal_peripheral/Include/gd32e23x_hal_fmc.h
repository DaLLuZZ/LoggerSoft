/*!
    \file    gd32e23x_hal_fmc.h
    \brief   definitions for the FMC

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

#ifndef GD32E23X_HAL_FMC_H
#define GD32E23X_HAL_FMC_H

#include "gd32e23x_hal.h"
#include <string.h>
#include <stdlib.h>
 
/* constants definitions */
typedef struct{
    uint32_t sector_start_addr;         /*!< sector actual start address */ 
    uint32_t sector_end_addr;           /*!< sector actual end address */ 
} hal_sector_addr_range_struct;

typedef struct  {
    uint8_t user;                       /*!< refer to field OB_USER of register FMC_OBSTAT */ 
    uint8_t security_protection;        /*!< refer to field PLEVEL of register FMC_OBSTAT */ 
    uint16_t data;                      /*!< refer to field OB_DATA of register FMC_OBSTAT */ 
    uint16_t write_protection;          /*!< refer to field OB_WP of register FMC_WP */ 
} hal_ob_parm_get_struct;

typedef struct {          
    hal_irq_handle_cb error_handle;     /*!< flash error interrupt */ 
    hal_irq_handle_cb finish_handle;    /*!< end of operation interrupt */  
} hal_fmc_irq_struct;

typedef struct  
{
    uint8_t ob_type;                    /*!< ob_type: OB_TYPE_WRP,OB_TYPE_SPC,OB_TYPE_USER and OB_TYPE_DATA.
                                        one or more parameters can be selected which are shown as above */

    uint8_t wrp_state;                  /*!< wrp_state: OB_WRP_DISABLE or OB_WRP_ENABLE */

    uint32_t wrp_addr;                  /*!< wrp_addr: specifies the target start address */

    uint32_t wrp_size;                  /*!< wrp_size: specifies the data size to be write protected */
  
    uint8_t spc_level;                  /*!< spc_level: OB_SPC_0, OB_SPC_1 or  OB_SPC_2
                                        only one parameter can be selected which is shown as above */
  
    uint8_t user;                       /*!< user: the value is used to configure OB_USER*/
  
    uint16_t data_value;                /*!< data_value:  the low byte of data_value is used to configure OB_DATA0
                                        the high byte of data_value is used to configure OB_DATA1 */  
} hal_ob_parm_config_struct;


#define OB_SPC    REG8((OB) + 0x00U)    /*!< option byte SPC value */
#define OB_USER   REG8((OB) + 0x02U)    /*!< option byte USER value */
#define OB_DATA0  REG8((OB) + 0x04U)    /*!< option byte DATA0 value */
#define OB_DATA1  REG8((OB) + 0x06U)    /*!< option byte DATA1 value */
#define OB_WP0    REG8((OB) + 0x08U)    /*!< option byte WP0 value */
#define OB_WP1    REG8((OB) + 0x0AU)    /*!< option byte WP1 value */

#define OB_TYPE_WRP       0x01U         /*!< WRP option byte configuration */
#define OB_TYPE_SPC       0x02U         /*!< SPC option byte configuration */
#define OB_TYPE_USER      0x04U         /*!< USER option byte configuration */
#define OB_TYPE_DATA      0x08U         /*!< DATA option byte configuration */

#define OB_WRP_DISABLE    0x00U         /*!< disable the write protection of the targeted pages */
#define OB_WRP_ENABLE     0x01U         /*!< enable the write protection of the targeted pages */

#define OB_SPC_0          0xA5U       /*!< no security protection */
#define OB_SPC_1          0xBBU       /*!< low security protection */
#define OB_SPC_2          0xCCU       /*!< high security protection */
                                                     
#define FMC_BASE_ADDRESS 0x8000000U     /*!< main flash start address*/
#define FMC_END_ADDRESS  0x8010000U     /*!< main flash max end address*/
#define FMC_PAGE_SIZE    0x400U         /*!< main flash page size */
#define FMC_SECTOR_SIZE  0x1000U        /*!< main flash sector size */

/* function declarations */
/* main flash operation */
/* unlock the main FMC operation */
void hal_fmc_unlock(void);
/* lock the main FMC operation */
void hal_fmc_lock(void);
/* enable flash prefetch */
void hal_fmc_prefetch_enable(void);
/* disable flash prefetch */
void hal_fmc_prefetch_disable(void);
/* set the wait state counter value */
void hal_fmc_wscnt_set(uint8_t wscnt);
/* read flash target region */
void hal_fmc_region_read(uint32_t start_addr, uint8_t *data, uint32_t size);
/* write flash target address in word */
fmc_state_enum hal_fmc_word_program(uint32_t addr, uint32_t data);
/* write flash target address in double words */
fmc_state_enum hal_fmc_doubleword_program(uint32_t addr, uint64_t data);
/* write flash target region with amounts of data */
int32_t hal_fmc_region_write(uint32_t start_addr, uint8_t *data, uint32_t data_size);
/* erase the page which start address locating in */
fmc_state_enum hal_fmc_page_erase(uint32_t start_addr);
/* erase the whole flash */
fmc_state_enum hal_fmc_mass_erase(void);
/* erase flash target region */
int32_t hal_fmc_region_erase(uint32_t start_addr, uint32_t size);

/* interrupt configuration */
/* fmc interrupt handler content function,which is merely used in fmc_handler */
void hal_fmc_irq(hal_fmc_irq_struct *p_irq);
/* set user-defined interrupt callback function, which will be registered and called when corresponding interrupt be triggered */
void hal_fmc_irq_handle_set(hal_fmc_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, which will be registered and called when corresponding interrupt be triggered */
void hal_fmc_irq_handle_all_reset(hal_fmc_irq_struct *p_irq);

/* option byte operation */
/* unlock option byte */
void hal_ob_unlock(void);
/* lock option byte */
void hal_ob_lock(void);
/* reset option byte */
void hal_ob_reset(void);
/* erase option byte */
fmc_state_enum hal_ob_erase(void);
/* configure option byte security protection */
fmc_state_enum hal_ob_security_protection_config(uint8_t ob_spc);
/* write option byte user */
fmc_state_enum hal_ob_user_write(uint8_t ob_user);
/* program the FMC data option byte */
fmc_state_enum hal_ob_data_program(uint16_t ob_data);
/* enable the targeted address region written protection */
hal_sector_addr_range_struct hal_ob_wp_enable(uint32_t start_addr, uint32_t data_size);
/* disable the targeted address region written protection */
hal_sector_addr_range_struct hal_ob_wp_disable(uint32_t start_addr, uint32_t data_size);
/* get option byte parameters, which are stored in register FMC_OBSTAT and FMC_WP */
int32_t hal_ob_parm_get(hal_ob_parm_get_struct *p_parm);
/* configure option byte parameters thoroughly */
int32_t hal_ob_parm_config(hal_ob_parm_config_struct *ob_parm);
#endif /* GD32E23X_HAL_FMC_H */
