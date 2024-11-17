/*!
    \file    gd32e23x_hal_fmc.c
    \brief   FMC driver

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

/* define array if it is necessary to reserve original data located in out of targeted scope */
#if (1U == FLASH_OPER_RESERVE_ORIGINAL_DATA)
static uint8_t _g_fmc_cache[FMC_PAGE_SIZE];
#endif

/* reserve original data located in out of targeted scope */
static void _fmc_page_erase_update(uint8_t *p_buffer, uint32_t size, uint32_t page_addr, \
                             uint32_t page_offset);
static void _fmc_program_align_word(uint32_t dst_addr, uint8_t *data, uint32_t size);

/*!
    \brief      unlock the main FMC operation
                it is better to used in pairs with fmc_lock
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_fmc_unlock(void)
{
    fmc_unlock();
}

/*!
    \brief      lock the main FMC operation
                it is better to used in pairs with fmc_unlock after an operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_fmc_lock(void)
{
    fmc_lock();
}

/*!
    \brief      enable flash prefetch
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_fmc_prefetch_enable(void)
{
    fmc_prefetch_enable();
}

/*!
    \brief      disable flash prefetch
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_fmc_prefetch_disable(void)
{
    fmc_prefetch_disable();
}

/*!
    \brief      set the wait state counter value
    \param[in]  wscnt: wait state counter value
      \arg        WS_WSCNT_0: 0 wait state added
      \arg        WS_WSCNT_1: 1 wait state added
      \arg        WS_WSCNT_2: 2 wait state added
    \param[out] none
    \retval     none
*/
void hal_fmc_wscnt_set(uint8_t wscnt)
{
    fmc_wscnt_set(wscnt);
}
/*!
    \brief      read flash target region
    \param[in]  start_addr: target region start address 
    \param[in]  data: pointer to read result
    \param[in]  size: target region size
    \param[out] none
    \retval     none
*/
void hal_fmc_region_read(uint32_t start_addr, uint8_t *data, uint32_t size)
{
    uint32_t counter;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((start_addr < FMC_BASE_ADDRESS)||(start_addr > FMC_END_ADDRESS)){
        HAL_DEBUGE("parameter [start_addr] is a invalid address.");
    }
    if(size > (FMC_END_ADDRESS - start_addr)){
        HAL_DEBUGE("target data size exceed flash atual address scope.");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* assign flash specific data to pointer */
    for(counter = 0U; counter < size; counter++){
        *(data + counter) = REG8(start_addr + counter);
    }
}

/*!
    \brief      write flash target address in word
    \param[in]  addr: target address 
    \param[in]  data: target data
    \param[out] none
    \retval     fmc_state_enum: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum hal_fmc_word_program(uint32_t addr, uint32_t data)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if((addr < FMC_BASE_ADDRESS)||(addr > FMC_END_ADDRESS)){
        HAL_DEBUGE("parameter [addr] is a invalid address.");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    return(fmc_word_program(addr, data));
}

/*!
    \brief      write flash target address in double words
    \param[in]  addr: target address 
    \param[in]  data: target data
    \param[out] none
    \retval     fmc_state_enum: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum hal_fmc_doubleword_program(uint32_t addr, uint64_t data)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameter */
    if((addr < FMC_BASE_ADDRESS)||(addr > FMC_END_ADDRESS)){
        HAL_DEBUGE("parameter [addr] is a invalid address.");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    return(fmc_doubleword_program(addr, data));
}

/*!
    \brief      write flash target region with amounts of data
    \param[in]  start_addr: target region start address 
    \param[in]  size: target region size
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_fmc_region_write(uint32_t start_addr, uint8_t *data, uint32_t size)
{
    uint32_t counter = 0U;
    uint32_t remaining = size;
    uint32_t update_offset = 0U;
    uint8_t *src_addr = data;
    uint32_t page_offset = 0U;
    uint32_t write_addr_limit = ((start_addr + size) / FMC_PAGE_SIZE + 1U) * FMC_PAGE_SIZE;
    uint16_t write_buffer_size;
    uint32_t page_addr;
    uint8_t  non_ff_flag = 0U;
    uint32_t length, left_align,right_align;
    uint8_t right_align_complement = 0U;
    uint32_t addr_size_temp = (start_addr / FMC_PAGE_SIZE)* FMC_PAGE_SIZE ;
    uint8_t single_page_flag; 
    /* check whether all data is at the same page */
    if((write_addr_limit - addr_size_temp) > FMC_PAGE_SIZE){
        single_page_flag = 0U;
    }else{
        single_page_flag = 1U;
    }
    page_offset = start_addr - (start_addr / FMC_PAGE_SIZE) * FMC_PAGE_SIZE;

    /* calculate left and right align edge */
    left_align = page_offset - page_offset % 4U;
    if((page_offset + size) % 4U){
        right_align_complement = 4U - (page_offset + size) % 4U;
    }else{
        right_align_complement = 0U;
    }
    right_align = (page_offset + size + right_align_complement) % FMC_PAGE_SIZE;
    if(0U == right_align)
    right_align = FMC_PAGE_SIZE;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if((start_addr < FMC_BASE_ADDRESS)||(start_addr > FMC_END_ADDRESS)){
        HAL_DEBUGE("parameter [start_addr] is a invalid address.");
        return HAL_ERR_ADDRESS;
    }
    if(NULL == data){
        HAL_DEBUGE("parameter [data] is a NULL pointer.");
        return HAL_ERR_ADDRESS;
    }
    if(size > (FMC_END_ADDRESS - start_addr)){
        HAL_DEBUGE("target data size exceed flash atual address scope.");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* save the original data , erase responding page */
    /* save the original data in the processing of writing data */
#if (1 == FLASH_OPER_RESERVE_ORIGINAL_DATA)
    do{
        for(counter = 0; counter < FMC_PAGE_SIZE; counter++){
            _g_fmc_cache[counter] = 0xFFU;
        }
        page_addr = (start_addr / FMC_PAGE_SIZE) * FMC_PAGE_SIZE;
        page_offset = start_addr - page_addr;
 
        /* get the currently transmitting data length */
        if(page_offset + remaining < FMC_PAGE_SIZE){
            length = remaining;
        }else{
            length = FMC_PAGE_SIZE - page_offset;
        }
        
        if(1U == single_page_flag){
            /* process the condition that all data is at the same page */
            update_offset = left_align;
            write_buffer_size = right_align - left_align;
            /* check whether target region content is 0xFF */
            for(counter = left_align; counter < right_align; counter++){
                if(0xFF != REG8(page_addr + counter)){
                    non_ff_flag = 1U;
                    break;
                }
            }
            /* backup the orignal data and erase the page */
            if(1U == non_ff_flag){
                for(counter = 0; counter < left_align; counter++){
                    if(0xFF != REG8(page_addr + counter)){
                        update_offset = counter;
                        write_buffer_size = right_align - counter;
                        break;
                    }
                }

                for(counter = FMC_PAGE_SIZE - 1U; counter >= right_align; counter--){
                    if(0xFF != REG8(page_addr + counter)){
                        write_buffer_size = counter - update_offset + 1U;
                        break;
                    }
                }
                memcpy((uint8_t *)(_g_fmc_cache + update_offset), (uint8_t *)(page_addr + update_offset), write_buffer_size);
                fmc_page_erase(page_addr);
                non_ff_flag = 0U;
            }
        }else{
            /* process the condition that all data cover more than one page */
            if(page_addr == (write_addr_limit - FMC_PAGE_SIZE)){
                /* process the data in last page */
                update_offset = page_offset;  
                write_buffer_size = right_align;
                /* check whether target region content is 0xFF */
                for(counter = 0; counter < right_align; counter++){
                    if(0xFF != REG8(page_addr + counter)){
                        non_ff_flag = 1U;
                        break;
                    }
                } 
                /* backup the orignal data and erase the page */
                if(1U == non_ff_flag){
                    for(counter = FMC_PAGE_SIZE - 1U; counter >= right_align; counter--){
                        if(0xFF != REG8(page_addr + counter)){
                            write_buffer_size = counter + 1U;
                            break;
                        }
                    }
                    /* load from the flash into the cache */
                    memcpy((uint8_t *)(_g_fmc_cache), (uint8_t *)(page_addr), write_buffer_size);
                    /* erase the page, and write the cache */
                    fmc_page_erase(page_addr);
                    non_ff_flag = 0U;
                }
            }else{
                /* process the data in middle pages */
                if(page_addr == start_addr){
                    update_offset = 0U;
                    write_buffer_size = FMC_PAGE_SIZE;
                    /* check whether target region content is 0xFF */
                    for(counter = 0U; counter <  FMC_PAGE_SIZE; counter++){
                        if(0xFF != REG8(page_addr + counter)){
                            non_ff_flag = 1U;
                            break;
                        }
                    }
                    /* erase the page if it is necessary */
                    if(1U == non_ff_flag){
                        fmc_page_erase(page_addr);
                        non_ff_flag = 0U;
                    }
                }else{ 
                    update_offset = left_align;
                    write_buffer_size = FMC_PAGE_SIZE - update_offset;
                    /* process the data in first page */
                    /* check whether target region content is 0xFF */
                    for(counter = left_align; counter < FMC_PAGE_SIZE; counter++){
                        if(0xFF != REG8(page_addr + counter)){
                            non_ff_flag = 1U;
                            break;
                        }
                    }  
                    /* backup the orignal data and erase the page */
                    if(1U == non_ff_flag){
                        for(counter = 0; counter < left_align; counter++){
                            if(0xFF != REG8(page_addr + counter)){
                                update_offset = counter;
                                write_buffer_size = FMC_PAGE_SIZE - update_offset;
                                break;
                            }
                        }
                        /* load from the flash into the cache */
                        memcpy((uint8_t *)(_g_fmc_cache + update_offset), (uint8_t *)(page_addr + update_offset), \
                                           write_buffer_size);
                        /* erase the page, and write the cache */
                        fmc_page_erase(page_addr);
                        non_ff_flag = 0U;
                    }
                }
            }
        }

        /* update the cache from the source */
        memcpy((uint8_t *)_g_fmc_cache + page_offset, src_addr, length);

        _fmc_program_align_word(page_addr + update_offset , (uint8_t *)(_g_fmc_cache + update_offset), write_buffer_size);
        start_addr += length;
        src_addr += length;
        remaining -= length;

    }while(remaining > 0);
#else
    while(counter < size){   
        if(0xffU == (REG8(start_addr + counter))){
            counter++;
        }else{
            /* erase the pgae unless all the original data is 0xFF */
            page_addr = ((start_addr + counter)/FMC_PAGE_SIZE) * FMC_PAGE_SIZE;
            fmc_page_erase(page_addr);
            counter++;
        }
    }
    /* save the original data which is outside the writing target range */
    _fmc_program_align_word(start_addr, data, size);
#endif /* save the original data which is outside the writing target range */
    return HAL_ERR_NONE;
}

/*!
    \brief      erase the page which start address locating in 
    \param[in]  start_addr: target region start address 
    \param[out] none
    \retval     fmc_state_enum: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum hal_fmc_page_erase(uint32_t start_addr)
{   
    fmc_state_enum state;
    uint32_t page_offset = 0U;
    
#if (1 == HAL_PARAMETER_CHECK)  
    /* check the parameter */
    if((start_addr < FMC_BASE_ADDRESS)||(start_addr > FMC_END_ADDRESS)){
        HAL_DEBUGE("parameter [start_addr] is a invalid address.");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    page_offset = start_addr % FMC_PAGE_SIZE;
    state = fmc_page_erase(start_addr - page_offset); 
    return(state);
}

/*!
    \brief      erase the whole flash 
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum hal_fmc_mass_erase(void)
{
    return(fmc_mass_erase());
}

/*!
    \brief      erase flash target region
    \param[in]  start_addr: target region start address 
    \param[in]  size: target region size
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/
int32_t hal_fmc_region_erase(uint32_t start_addr, uint32_t size)
{
    uint32_t page_offset, page_number, counter = 0U;
    uint32_t page_addr;
    
#if (1 == HAL_PARAMETER_CHECK)  
    /* check the parameters */
    if((start_addr < FMC_BASE_ADDRESS)||(start_addr > FMC_END_ADDRESS)){
        HAL_DEBUGE("parameter [start_addr] is a invalid address.");
        return HAL_ERR_ADDRESS;
    }
    if(size > (FMC_END_ADDRESS - start_addr)){
        HAL_DEBUGE("target data size exceed flash atual address scope.");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    page_offset = start_addr % FMC_PAGE_SIZE;
    if((size + page_offset) % FMC_PAGE_SIZE ) {
        page_number =(size + page_offset) / FMC_PAGE_SIZE + 1U;
    }else{
        page_number =(size + page_offset) / FMC_PAGE_SIZE;
    }
    page_addr = start_addr - page_offset;
#if (1 == FLASH_OPER_RESERVE_ORIGINAL_DATA)
    /* page nubmer which buffer refer is equal to one */
    if(1U == page_number){
        _fmc_page_erase_update(_g_fmc_cache, size, page_addr, page_offset);
    }else if(2U == page_number){
        /* page nubmer which buffer refer is equal to two */
        _fmc_page_erase_update(_g_fmc_cache, FMC_PAGE_SIZE - page_offset, \
                         page_addr, page_offset);

        _fmc_page_erase_update(_g_fmc_cache, page_offset + size - FMC_PAGE_SIZE, \
                         page_addr + FMC_PAGE_SIZE, 0);

    }else{
        /* page nubmer which buffer refer is more than two */
        _fmc_page_erase_update(_g_fmc_cache, FMC_PAGE_SIZE - page_offset, \
                         page_addr, page_offset);

        /* erase middle pages */
        for (counter = 1; counter < (page_number - 1U); counter++){
            fmc_page_erase(page_addr + (FMC_PAGE_SIZE * counter));
        }
        counter = (page_offset + size) % FMC_PAGE_SIZE;
        if(0U == counter){
            counter = FMC_PAGE_SIZE;
        }
        _fmc_page_erase_update(_g_fmc_cache, counter, \
                        page_addr + (FMC_PAGE_SIZE * (page_number - 1U)), 0);
    }
#else
    /* erase all the pages which buffer refer */
    for(counter =0U; counter < page_number; counter++){
        fmc_page_erase(page_addr + (counter * FMC_PAGE_SIZE));
    }
#endif /* save the original data which is outside the erasing target range */
    return HAL_ERR_NONE;
}

/*!
    \brief      fmc interrupt handler content function, which is merely used in fmc_handler
    \param[in]  p_irq: pointer to fmc interrupt handler callback function 
    \param[out] none
    \retval     none 
*/ 
void hal_fmc_irq(hal_fmc_irq_struct *p_irq)
{
    if(SET == fmc_interrupt_flag_get(FMC_INT_FLAG_PGERR | FMC_INT_FLAG_PGAERR | FMC_INT_FLAG_WPERR)){
        fmc_interrupt_flag_clear(FMC_INT_FLAG_PGERR | FMC_INT_FLAG_PGAERR | FMC_INT_FLAG_WPERR);
        if(p_irq->error_handle != NULL){
            p_irq->error_handle(p_irq);
        }
    }
    
    if(SET == fmc_interrupt_flag_get(FMC_INT_FLAG_END)){
        fmc_interrupt_flag_clear(FMC_INT_FLAG_END);
        if(p_irq->finish_handle != NULL){
            p_irq->finish_handle(p_irq);
        }
    }
}

/*!
    \brief      set user-defined interrupt callback function, which will be registered and called 
                when corresponding interrupt be triggered
    \param[in]  p_irq: pointer to fmc interrupt handler callback function 
    \param[out] none
    \retval     none 
*/ 
void hal_fmc_irq_handle_set(hal_fmc_irq_struct *p_irq)
{
    if(NULL != p_irq->error_handle){
        p_irq->error_handle(p_irq);
        fmc_interrupt_enable(FMC_INTEN_ERR);
    }else{
        fmc_interrupt_disable(FMC_INTEN_ERR);
    }
    
    if(NULL != p_irq->finish_handle){
        p_irq->finish_handle(p_irq);
        fmc_interrupt_enable(FMC_INTEN_END);
    }else{
        fmc_interrupt_disable(FMC_INTEN_END);
    }
}
 
/*!
    \brief      reset user-defined interrupt callback function, which will be registered and called 
                when corresponding interrupt be triggered
    \param[in]  p_irq: pointer to fmc interrupt handler callback function 
    \param[out] none
    \retval     none 
*/ 
void hal_fmc_irq_handle_all_reset(hal_fmc_irq_struct *p_irq)
{
    p_irq->error_handle = NULL;
    p_irq->finish_handle = NULL;
}

/*!
    \brief      unlock option byte 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_ob_unlock(void)
{
    ob_unlock();
}

/*!
    \brief      lock option byte 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_ob_lock(void)
{
    ob_lock();
}

/*!
    \brief      reset option byte 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_ob_reset(void)
{
    ob_reset();
}

/*!
    \brief      erase option byte 
    \param[in]  none
    \param[out] none
    \retval     fmc_state_enum: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum hal_ob_erase(void)
{
    return(ob_erase());
}

/*!
    \brief      configure option byte security protection
    \param[in]  ob_spc: specify security protection code
      \arg        FMC_NSPC: no security protection
      \arg        FMC_LSPC: low security protection
      \arg        FMC_HSPC: high security protection
    \param[out] none
    \retval     fmc_state_enum: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum hal_ob_security_protection_config(uint8_t ob_spc)
{
    return (ob_security_protection_config(ob_spc));
}

/*!
    \brief      write option byte user
                this function can only clear the corresponding bits to be 0 rather than 1.
                the function ob_erase is used to set all the bits to be 1.
    \param[in]  ob_user: user option byte
                one or more parameters (bitwise AND) can be selected which are shown as below:
      \arg        OB_FWDGT_HW: hardware free watchdog timer
      \arg        OB_DEEPSLEEP_RST: no reset when entering deepsleep mode
      \arg        OB_STDBY_RST: no reset when entering standby mode
      \arg        OB_BOOT1_SET_1: BOOT1 bit is 1
      \arg        OB_VDDA_DISABLE: disable VDDA monitor
      \arg        OB_SRAM_PARITY_ENABLE: enable sram parity check
    \param[out] none
    \retval     fmc_state_enum: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum hal_ob_user_write(uint8_t ob_user)
{
    return (ob_user_write(ob_user));
}

/*!
    \brief      program the FMC data option byte
    \param[in]  addr: OB_DATA_ADDR0 or OB_DATA_ADDR1
      \arg        OB_DATA_ADDR0: option byte data address 0
      \arg        OB_DATA_ADDR1: option byte data address 1
    \param[in]  data: the byte to be programmed
    \param[out] none
    \retval     fmc_state_enum: state of FMC
      \arg        FMC_READY: the operation has been completed
      \arg        FMC_BUSY: the operation is in progress
      \arg        FMC_PGERR: program error
      \arg        FMC_PGAERR: program alignment error
      \arg        FMC_WPERR: erase/program protection error
      \arg        FMC_TOERR: timeout error
      \arg        FMC_OB_HSPC: option byte security protection code high
*/
fmc_state_enum hal_ob_data_program(uint16_t ob_data)
{
    return(ob_data_program(ob_data));
}

/*!
    \brief      enable the targeted address region written protection
    \param[in]  start_addr: target region start address 
    \param[in]  data_size: target region size
    \param[out] none
    \retval     sector_addr_range_struct: actual start and end address
*/
hal_sector_addr_range_struct hal_ob_wp_enable(uint32_t start_addr, uint32_t data_size)
{
    uint8_t index, counter, sub_counter;
    hal_sector_addr_range_struct sector;
    uint16_t wp_temp0 = (uint16_t)REG8(OB + 8U);
    uint16_t wp_temp1 = (uint16_t)(REG8(OB + 0xaU) << 8U);
    uint16_t wp_prev = wp_temp0 | wp_temp1; 
    uint16_t wp = 0U;
    uint16_t wp_sector = 1U ;
    uint8_t sector_number;
    
#if (1 == HAL_PARAMETER_CHECK)  
    /* check the parameters */
    if((start_addr < FMC_BASE_ADDRESS)||(start_addr > FMC_END_ADDRESS)){
        HAL_DEBUGE("parameter [start_addr] is a invalid address.");
    }
    if(data_size > (FMC_END_ADDRESS - start_addr)){
        HAL_DEBUGE("target data size exceed flash atual address scope.");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    index = (start_addr & 0x000f000U) >> 12U;
    if(data_size % FMC_SECTOR_SIZE){
        sector_number = data_size / FMC_SECTOR_SIZE + 1U;
    }else{
        sector_number = data_size / FMC_SECTOR_SIZE;
    }      
    for(counter = 0U; counter < sector_number ; counter++){
        for(sub_counter = 0U; sub_counter < (index + counter); sub_counter++){
            wp_sector = 2U * wp_sector;
        }
        wp = wp + wp_sector;
        wp_sector = 1U;
    }
    /* check current write protection sector, if there is confict, erase option byte */
    if(((0xFFU != (uint8_t)wp_prev)&&(0U != (uint8_t)wp)) || ((0xFFU != (uint8_t)(wp_prev >> 8U))&&(0 != (uint8_t)(wp >> 8U)))){  
        wp = wp | (~wp_prev);
        ob_erase();  
    }
    /* enable the target sector write protected */
    ob_write_protection_enable(wp);
 
    /* define actual address range which is writen protected */
    sector.sector_start_addr = FMC_BASE_ADDRESS + index * FMC_SECTOR_SIZE;
    sector.sector_end_addr = sector.sector_start_addr + sector_number * FMC_SECTOR_SIZE;
 
    return(sector);
 }

/*!
    \brief      disable the targeted address region written protection
    \param[in]  start_addr: target region start address 
    \param[in]  data_size: target region size
    \param[out] none
    \retval     sector_addr_range_struct: actual start and end address
*/
hal_sector_addr_range_struct hal_ob_wp_disable(uint32_t start_addr, uint32_t data_size)
{
    uint8_t index, counter, sub_counter;
    hal_sector_addr_range_struct sector;
    uint16_t wp_temp0 = (uint16_t)REG8(OB + 8U);
    uint16_t wp_temp1 = (uint16_t)(REG8(OB + 0xaU) << 8U);
    uint16_t wp_prev = wp_temp0 | wp_temp1;  
   
    uint16_t wp = 0U;
    uint16_t wp_sector = 1U ;
    uint8_t page_number;
    
#if (1 == HAL_PARAMETER_CHECK)  
    /* check the parameters */
    if((start_addr < FMC_BASE_ADDRESS)||(start_addr > FMC_END_ADDRESS)){
        HAL_DEBUGE("parameter [start_addr] is a invalid address.");
    }
    if(data_size > (FMC_END_ADDRESS - start_addr)){
        HAL_DEBUGE("target data size exceed flash atual address scope.");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    index = (start_addr & 0x000f000U) >> 12U;
    if(data_size % FMC_SECTOR_SIZE){
        page_number = data_size / FMC_SECTOR_SIZE + 1U;
    }else{
        page_number = data_size / FMC_SECTOR_SIZE;
    }      

    for(counter = 0U; counter < page_number ; counter++){
        for(sub_counter = 0U; sub_counter < (index + counter); sub_counter++){
            wp_sector = 2U * wp_sector;
        }
        wp = wp + wp_sector;
        wp_sector = 1U;
    }
    /* disable the target sector write protected */
    wp = (~wp) & (~wp_prev);
    
    ob_erase();    

    ob_write_protection_enable(wp);
 
    /* define actual address range which is writen protected */
    sector.sector_start_addr = FMC_BASE_ADDRESS + index * FMC_SECTOR_SIZE;
    sector.sector_end_addr = sector.sector_start_addr + page_number * FMC_SECTOR_SIZE;
 
    return(sector);
 }

/*!
    \brief      get option byte parameters, which are stored in register FMC_OBSTAT and FMC_WP
    \param[in]  p_parm: pointer to currently valid option byte parameters 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/ 
int32_t hal_ob_parm_get(hal_ob_parm_get_struct *p_parm)
{
#if (1 == HAL_PARAMETER_CHECK)  
    /* check the parameter */
    if(NULL == p_parm){
        HAL_DEBUGE("parameter [p_parm] is a NULL pointer.");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* get the information of option byte */
    p_parm->data = ob_data_get();
    p_parm->user = ob_user_get();
    p_parm->security_protection = ob_obstat_plevel_get();
    p_parm->write_protection = ob_write_protection_get();

    return HAL_ERR_NONE;
}

/*!
    \brief      configure option byte parameters thoroughly
    \param[in]  ob_parm: pointer to option byte struct used to configure its parameters 
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_VAL details refer to gd32e23x_hal.h
*/ 
int32_t hal_ob_parm_config(hal_ob_parm_config_struct *ob_parm)
{

    uint8_t spc = 0xA5U;
    uint8_t user = 0xFFU;
    uint16_t data = 0xFFFFU;
    uint16_t wrp = 0xFFFFU;
    uint8_t wrp_flag = 0U;
    uint64_t ob_parameter = 0U;
    uint32_t data_temp0, data_temp1, wp_temp0, wp_temp1;
    fmc_state_enum fmc_state;
    
#if (1 == HAL_PARAMETER_CHECK)  
    /* check the parameter */
    if(NULL == ob_parm){
        HAL_DEBUGE("parameter [ob_parm] is a NULL pointer.");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* unlock flash and option byte */
    hal_fmc_unlock();
    hal_ob_unlock();

    /* check whether spc is default value */
    if(OB_SPC != spc){
        spc = OB_SPC;
    }
    /* check whether user is default value */
    if(OB_USER != user){
        user = OB_USER;
    }
    /* check whether data is default value */
    if((OB_DATA0 != (uint8_t)data) || (OB_DATA1 != (uint8_t)(data >> 8U))){
        data_temp0 = (uint16_t) OB_DATA0;
        data_temp1 = (uint16_t)(OB_DATA1 << 8U);
        data = data_temp0 | data_temp1;
    } 
    /* check whether WP is default value */
    if ((OB_WP0 != (uint8_t)wrp) || (OB_WP1 != (uint8_t)(wrp >> 8U))){
        wp_temp0 = (uint16_t) OB_WP0;
        wp_temp1 = (uint16_t)(OB_WP1 << 8U);
        wrp = wp_temp0 | wp_temp1;
        wrp_flag = 1U;
    }
    /* write protection process */
    if(ob_parm->ob_type & OB_TYPE_WRP){
        if(OB_WRP_ENABLE == ob_parm->wrp_state){
            hal_ob_wp_enable(ob_parm->wrp_addr, ob_parm->wrp_size);
            wp_temp0 = (uint16_t) OB_WP0;
            wp_temp1 = (uint16_t)(OB_WP1 << 8U);
            wrp = wp_temp0 | wp_temp1;
            wrp_flag = 0U;
        }else{
            hal_ob_wp_disable(ob_parm->wrp_addr, ob_parm->wrp_size);
            wp_temp0 = (uint16_t) OB_WP0;
            wp_temp1 = (uint16_t)(OB_WP1 << 8U);
            wrp = wp_temp0 | wp_temp1;
            wrp_flag = 0U;
        }
    } 
    /* security protection process */
    if(ob_parm->ob_type & (OB_TYPE_SPC | OB_TYPE_USER | OB_TYPE_DATA)){
        wrp_flag = 1U;
        if(ob_parm->ob_type & OB_TYPE_SPC)
        spc = ob_parm->spc_level;
        if(ob_parm->ob_type & OB_TYPE_USER)
        user &= ob_parm->user;
        if(ob_parm->ob_type & OB_TYPE_DATA)
        data = ob_parm->data_value;
        
        ob_parameter = ((uint64_t)spc |(uint64_t)(user << 16U) | (uint64_t)((uint8_t)data) << 32U
                       | (uint64_t)(data >> 8U) << 48U);

        FMC_WS |= FMC_WS_PGW;
        /* start erase the option byte */
        FMC_CTL |= FMC_CTL_OBER;
        FMC_CTL |= FMC_CTL_START;
        /* wait for the FMC ready */
        fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT);
    
        if(FMC_READY == fmc_state){
        
            /* reset the OBER bit */
            FMC_CTL &= ~FMC_CTL_OBER;
      
            /* enable the option bytes programming */
            FMC_CTL |= FMC_CTL_OBPG;
       
            OB_SPC_USER = (uint32_t)ob_parameter;
            OB_DATA = (uint32_t)(ob_parameter >> 32U);
            
            /* wait for the FMC ready */
            fmc_state = fmc_ready_wait(FMC_TIMEOUT_COUNT); 

            FMC_WS &= ~(FMC_WS_PGW);
            if(FMC_TOERR != fmc_state){
                /* reset the OBPG bit */
                FMC_CTL &= ~FMC_CTL_OBPG;
            }
        }else{
            if(FMC_TOERR != fmc_state){
                /* reset the OBER bit */
                FMC_CTL &= ~FMC_CTL_OBER;
            }
        }
        
    }
    /* if previously actual option byte value exist, write it back */
    if(1U == wrp_flag){
        ob_write_protection_enable(~wrp);
    }
    /* lock flash and option byte */
    hal_ob_lock();
    hal_fmc_lock();

    return HAL_ERR_NONE;
}
/*!
    \brief      write target data aligned in word
    \param[in]  p_buffer: pointer to data buffer
    \param[in]  size: target region size
    \param[in]  page_addr: page head address
    \param[in]  page_offset: the offset with respect to page head address
    \param[out] none
    \retval     none
*/
static void _fmc_page_erase_update(uint8_t *p_buffer, uint32_t size, uint32_t page_addr, \
                             uint32_t page_offset)
{
    uint32_t temp;
    uint32_t counter;
    uint32_t update_index = 0U;
    
    temp = (uint32_t)p_buffer;

    for(counter = 0U; counter < FMC_PAGE_SIZE; counter += 4){
        if(0xFFFFFFFF != REG32(page_addr + counter)){
            update_index = counter;
            break;
        }
    }
    
    if(FMC_PAGE_SIZE != counter){
        for(counter = update_index; counter < FMC_PAGE_SIZE; counter += 4){
            REG32(temp + counter) = REG32(page_addr + counter);
        }

        /* configure the target region to 0xFF */
        for(counter = 0U;counter < size; counter++){
            p_buffer[page_offset + counter] = 0xFFU;
        }
        
        fmc_page_erase(page_addr);
        
        /* write the orignal data to address outside the target range */
        temp = (uint32_t)p_buffer;
        for(counter = update_index; counter < FMC_PAGE_SIZE; counter += 4){
            fmc_word_program(page_addr + counter, REG32(temp + counter));
        }
    }
}

/*!
    \brief      write target data aligned in word
    \param[in]  start_addr: target region start address 
    \param[in]  data: pointer to target data
    \param[in]  data_size: target region size
    \param[out] none
    \retval     none
*/
static void _fmc_program_align_word(uint32_t dst_addr, uint8_t *data, uint32_t data_size)
{
    uint32_t head_offset, tail_offset, data_offset;
    uint32_t data_addr_align, data_size_align;
    uint32_t counter;
    uint32_t temp_addr;
    uint8_t align_buffer[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    
    head_offset = dst_addr % 4U;
    tail_offset = (data_size + head_offset) % 4U;
    
    if(0 != head_offset){
        /* write the target region if front address is non-aligned */
        data_addr_align = dst_addr + (4 - head_offset);
        data_size_align = data_size - (4 - head_offset);
        
        temp_addr = (uint32_t)data;
        for(counter = head_offset; counter < 4; counter++){
            align_buffer[counter] = REG8(temp_addr);
            temp_addr++;
        }
        data_offset = 4 - head_offset;
        fmc_word_program(dst_addr - head_offset, REG32(align_buffer));
    }else{
        /* write the target region aligned address */
        data_addr_align = dst_addr;
        data_size_align = data_size;
        data_offset = 0;
    }
    /* write the target region if back address is non-aligned */
    if(0 != tail_offset){
        data_size_align = data_size - tail_offset;
        
        temp_addr = (uint32_t)data;
        temp_addr = temp_addr + data_size - tail_offset;
        for(counter = 0; counter < tail_offset; counter++){
            align_buffer[counter] = REG8(temp_addr);
            temp_addr++;
        }
        
        fmc_word_program(dst_addr + data_size - tail_offset, REG32(align_buffer));
    }
    /* write the target region middle aligned address */
    temp_addr = (uint32_t)data + data_offset;
    for(counter = 0; counter < data_size_align; counter += 4){
        fmc_word_program(data_addr_align + counter, REG32(temp_addr + counter));
    }
}
