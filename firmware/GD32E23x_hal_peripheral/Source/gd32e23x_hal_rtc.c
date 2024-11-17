/*!
    \file    gd32e23x_hal_rtc.c
    \brief   RTC driver
    
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

static hal_rtc_irq_struct rtc_irq = {NULL,NULL,NULL,NULL};

/*!
    \brief      initialize the specified structure
    \param[in]  hal_struct_type: 
                  the argument could be selected from enumeration <hal_rtc_struct_type_enum>
    \param[in]  p_struct: point to the structure to be deinitialized    
    \param[out] none
    \retval     none
*/
void hal_rtc_struct_init(hal_rtc_struct_type_enum hal_struct_type, void *p_struct)
{
#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == p_struct){
        HAL_DEBUGE("parameter [*p_struct] value is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    switch(hal_struct_type){
    case HAL_RTC_INIT_STRUCT:
        /* initialize RTC initialization structure with the default values */
        ((hal_rtc_init_struct*)p_struct)->rtc_year = 0x00;
        ((hal_rtc_init_struct*)p_struct)->rtc_month = HAL_RTC_JAN;
        ((hal_rtc_init_struct*)p_struct)->rtc_date = 0x1;
        ((hal_rtc_init_struct*)p_struct)->rtc_day_of_week = HAL_RTC_MONDAY;
        ((hal_rtc_init_struct*)p_struct)->rtc_hour = 0x0;
        ((hal_rtc_init_struct*)p_struct)->rtc_minute = 0x0;
        ((hal_rtc_init_struct*)p_struct)->rtc_second = 0x0;
        ((hal_rtc_init_struct*)p_struct)->rtc_subsecond = 0x0;    
        ((hal_rtc_init_struct*)p_struct)->rtc_factor_asyn = 0x63;    
        ((hal_rtc_init_struct*)p_struct)->rtc_factor_syn = 0x18F;
        ((hal_rtc_init_struct*)p_struct)->rtc_am_pm = HAL_RTC_AM;
        ((hal_rtc_init_struct*)p_struct)->rtc_display_format = HAL_RTC_24HOUR;
        
        break;
    
    case HAL_RTC_ALARM_STRUCT:
        /* initialize RTC device alarm function structure with the default values */
        ((hal_rtc_alarm_struct*)p_struct)->rtc_alarm_mask = HAL_RTC_ALARM_NONE_MASK;
        ((hal_rtc_alarm_struct*)p_struct)->rtc_weekday_or_date = HAL_RTC_ALARM_DATE_SELECTED;
        ((hal_rtc_alarm_struct*)p_struct)->rtc_alarm_day = 0x1;
        ((hal_rtc_alarm_struct*)p_struct)->rtc_alarm_hour = 0x0;
        ((hal_rtc_alarm_struct*)p_struct)->rtc_alarm_minute = 0x0;
        ((hal_rtc_alarm_struct*)p_struct)->rtc_alarm_second = 0x0;
        ((hal_rtc_alarm_struct*)p_struct)->rtc_alarm_subsecond = 0x0;
        ((hal_rtc_alarm_struct*)p_struct)->rtc_alarm_subsecond_mask = HAL_RTC_MASK_SUBSECOND;
        ((hal_rtc_alarm_struct*)p_struct)->rtc_am_pm = HAL_RTC_AM;
    
        break;

    case HAL_RTC_TIMESTAMP_STRUCT:
        /* initialize RTC device timestamp function structure with the default values */
        ((hal_rtc_timestamp_struct*)p_struct)->rtc_timestamp_month = RTC_JAN;
        ((hal_rtc_timestamp_struct*)p_struct)->rtc_timestamp_date = 0x1;
        ((hal_rtc_timestamp_struct*)p_struct)->rtc_timestamp_day = RTC_MONDAY;
        ((hal_rtc_timestamp_struct*)p_struct)->rtc_timestamp_hour = 0x0;
        ((hal_rtc_timestamp_struct*)p_struct)->rtc_timestamp_minute = 0x0;
        ((hal_rtc_timestamp_struct*)p_struct)->rtc_timestamp_second = 0x0;
        ((hal_rtc_timestamp_struct*)p_struct)->rtc_timestamp_subsecond = 0x0;
        ((hal_rtc_timestamp_struct*)p_struct)->rtc_am_pm = HAL_RTC_AM;
    
        break;

    case HAL_RTC_TAMPER_STRUCT:
        /* initialize RTC device tamper function structure with the default values */
        ((hal_rtc_tamper_struct*)p_struct)->rtc_tamper_source = HAL_RTC_TAMPER1;
        ((hal_rtc_tamper_struct*)p_struct)->rtc_tamper_trigger = HAL_RTC_TAMPER_EDGE_RISING;
        ((hal_rtc_tamper_struct*)p_struct)->rtc_tamper_filter = HAL_RTC_FILTER_EDGE;
        ((hal_rtc_tamper_struct*)p_struct)->rtc_tamper_sample_frequency = HAL_RTC_FREQ_DIV32768;
        ((hal_rtc_tamper_struct*)p_struct)->rtc_tamper_precharge_enable = DISABLE;
        ((hal_rtc_tamper_struct*)p_struct)->rtc_tamper_precharge_time = HAL_RTC_1_CLK_PRECHARGE;
        ((hal_rtc_tamper_struct*)p_struct)->rtc_tamper_with_timestamp = DISABLE;
    
        break;    

    case HAL_RTC_IRQ_STRUCT:
        /* initialize RTC device interrupt callback function structure with the default values */
        ((hal_rtc_irq_struct*)p_struct)->alarm_handle = NULL;
        ((hal_rtc_irq_struct*)p_struct)->timestamp_handle = NULL;
        ((hal_rtc_irq_struct*)p_struct)->tamper0_handle = NULL;
        ((hal_rtc_irq_struct*)p_struct)->tamper1_handle = NULL;
        
        break;
    
    default:
        HAL_DEBUGW("parameter [hal_struct_type] value is undefine");
        break;
    }
}

/*!
    \brief      deinitialize RTC 
    \param[in]  none
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_deinit(void)
{
    ErrStatus error_status = ERROR;
    int32_t ret_status = HAL_ERR_TIMEOUT;
    
    /* RTC_TAMP register is not under write protection */
    RTC_TAMP = RTC_REGISTER_RESET;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* reset RTC_CTL register, this can be done without the init mode */
    RTC_CTL &= RTC_REGISTER_RESET;

    /* enter init mode */
    error_status = rtc_init_mode_enter();

    if(ERROR != error_status){
        /* before reset RTC_TIME and RTC_DATE, BPSHAD bit in RTC_CTL should be reset as the condition.
           in order to read calendar from shadow register, not the real registers being reset */
        RTC_TIME = RTC_REGISTER_RESET;
        RTC_DATE = RTC_DATE_RESET;

        RTC_PSC = RTC_PSC_RESET;

        /* reset RTC_STAT register, also exit init mode.
           at the same time, RTC_STAT_SOPF bit is reset, as the condition to reset RTC_SHIFTCTL register later */
        RTC_STAT = RTC_STAT_RESET;
      
        /* to write RTC_ALRM0SS register, ALRM0EN bit in RTC_CTL register should be reset as the condition */
        RTC_ALRM0TD = RTC_REGISTER_RESET;
        RTC_ALRM0SS = RTC_REGISTER_RESET;

        /* reset RTC_SHIFTCTL and RTC_HRFC register, this can be done without the init mode */
        RTC_SHIFTCTL = RTC_REGISTER_RESET;
        RTC_HRFC = RTC_REGISTER_RESET;

        ret_status = hal_rtc_register_sync_wait();
        if(HAL_ERR_TIMEOUT == ret_status){
            HAL_DEBUGE("rtc register synchronized with APB clock timout");
            return HAL_ERR_TIMEOUT;             
        }
    }else{
        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
        
        HAL_DEBUGE("rtc enter init mode timout");
        return HAL_ERR_TIMEOUT;        
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return HAL_ERR_NONE;
}

/*!
    \brief      initialize RTC
    \param[in]  rtc_initpara_struct: pointer to a hal_rtc_init_struct structure which contains 
                parameters for initialization of the rtc peripheral
                members of the structure and the member values are shown as below:
                  rtc_year: 0x0 - 0x99(BCD format)
                  rtc_month: 
                    the argument could be selected from enumeration <hal_rtc_month_enum>
                  rtc_date: 0x1 - 0x31(BCD format)
                  rtc_day_of_week:
                    the argument could be selected from enumeration <hal_rtc_day_of_week_enum>
                  rtc_hour: 0x1 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc_display_format chose
                  rtc_minute: 0x0 - 0x59(BCD format)
                  rtc_second: 0x0 - 0x59(BCD format)
                  rtc_subsecond: 0x0 - 0xFFFF
                  rtc_factor_asyn: 0x0 - 0x7F
                  rtc_factor_syn: 0x0 - 0x7FFF
                  rtc_am_pm: HAL_RTC_AM, HAL_RTC_PM
                  rtc_display_format: HAL_RTC_24HOUR, HAL_RTC_12HOUR
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_TIMEOUT, HAL_ERR_VAL, HAL_ERR_NONE,
                details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_init(hal_rtc_init_struct* rtc_initpara_struct)
{
    ErrStatus error_status = ERROR;
    int32_t ret_status = HAL_ERR_TIMEOUT;
    uint32_t reg_time = 0x00U, reg_date = 0x00U;
    uint32_t hour_binary = 0x00U, minute_binary = 0x00U, second_binary = 0x00U;
    uint32_t year_binary = 0x00U, date_binary = 0x00U;

#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == rtc_initpara_struct){
        HAL_DEBUGE("pointer [rtc_initpara_struct] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* check parameter rtc_initpara_struct->rtc_year */
    year_binary = rtc_bcd_2_normal(rtc_initpara_struct->rtc_year);
    if(year_binary > 99){
        HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_year] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter rtc_initpara_struct->rtc_date */
    date_binary = rtc_bcd_2_normal(rtc_initpara_struct->rtc_date);
    if((date_binary < 1U)||(date_binary > 31U)){
        HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_date] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter rtc_initpara_struct->rtc_display_format */
    if((HAL_RTC_24HOUR != rtc_initpara_struct->rtc_display_format)&&(HAL_RTC_12HOUR != rtc_initpara_struct->rtc_display_format)){
        HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_display_format] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter rtc_initpara_struct->rtc_hour, and rtc_initpara_struct->rtc_am_pm */
    hour_binary = rtc_bcd_2_normal(rtc_initpara_struct->rtc_hour);
    if(HAL_RTC_12HOUR == rtc_initpara_struct->rtc_display_format){
        
        if((hour_binary < 1U)||(hour_binary > 12U)){
            HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_hour] value is invalid");
            return HAL_ERR_VAL;
        }
        if((HAL_RTC_AM != rtc_initpara_struct->rtc_am_pm)&&(HAL_RTC_PM != rtc_initpara_struct->rtc_am_pm)){
            HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_am_pm] value is invalid");
            return HAL_ERR_VAL;
        }        
    }else{
        if(hour_binary > 23U){
            HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_hour] value is invalid");
            return HAL_ERR_VAL;
        }
        rtc_initpara_struct->rtc_am_pm = HAL_RTC_AM;
    }

    /* check parameter rtc_initpara_struct->rtc_minute */
    minute_binary = rtc_bcd_2_normal(rtc_initpara_struct->rtc_minute);
    if(minute_binary > 59){
        HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_minute] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter rtc_initpara_struct->rtc_second */
    second_binary = rtc_bcd_2_normal(rtc_initpara_struct->rtc_second);
    if(second_binary > 59){
        HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_second] value is invalid");
        return HAL_ERR_VAL;
    }    

    /* check parameter rtc_initpara_struct->rtc_factor_asyn */
    if(rtc_initpara_struct->rtc_factor_asyn > 0x7F){
        HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_factor_asyn] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter rtc_initpara_struct->rtc_factor_syn */
    if(rtc_initpara_struct->rtc_factor_syn > 0x7FFF){
        HAL_DEBUGE("parameter [rtc_initpara_struct->rtc_factor_syn] value is invalid");
        return HAL_ERR_VAL;
    }    
    
    
    reg_date = (DATE_YR(rtc_initpara_struct->rtc_year) | \
                DATE_DOW(rtc_initpara_struct->rtc_day_of_week) | \
                DATE_MON(rtc_initpara_struct->rtc_month) | \
                DATE_DAY(rtc_initpara_struct->rtc_date)); 
    
    reg_time = (rtc_initpara_struct->rtc_am_pm| \
                TIME_HR(rtc_initpara_struct->rtc_hour) | \
                TIME_MN(rtc_initpara_struct->rtc_minute) | \
                TIME_SC(rtc_initpara_struct->rtc_second)); 
              
    /* 1st: disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* 2nd: enter init mode */
    error_status = rtc_init_mode_enter();

    if(ERROR != error_status){
        RTC_PSC = (uint32_t)(PSC_FACTOR_A(rtc_initpara_struct->rtc_factor_asyn)| \
                             PSC_FACTOR_S(rtc_initpara_struct->rtc_factor_syn));

        RTC_TIME = (uint32_t)reg_time;
        RTC_DATE = (uint32_t)reg_date;

        RTC_CTL &= (uint32_t)(~RTC_CTL_CS);
        RTC_CTL |=  rtc_initpara_struct->rtc_display_format;
        
        /* 3rd: exit init mode */
        rtc_init_mode_exit();
        
        /* 4th: wait the RSYNF flag to set */
        ret_status = hal_rtc_register_sync_wait();
    }else{
        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
        
        HAL_DEBUGE("rtc enter init mode timout");
        return HAL_ERR_TIMEOUT;        
    }

    /* 5th: enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;   

    return ret_status;
}

/*!
    \brief      wait until RTC_TIME and RTC_DATE registers are synchronized with APB clock, and the shadow 
                registers are updated
    \param[in]  none
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_register_sync_wait(void)
{
    volatile uint32_t time_index = RTC_RSYNF_TIMEOUT;
    volatile uint32_t flag_status = RESET;
    volatile int32_t ret_status = HAL_ERR_NONE;

    if((uint32_t)RESET == (RTC_CTL & RTC_CTL_BPSHAD)){
        /* disable the write protection */
        RTC_WPK = RTC_UNLOCK_KEY1;
        RTC_WPK = RTC_UNLOCK_KEY2;

        /* firstly clear RSYNF flag */
        RTC_STAT &= (uint32_t)(~RTC_STAT_RSYNF);

        /* wait until RSYNF flag to be set */
        do{
            flag_status = rtc_flag_get(RTC_FLAG_RSYN);
        }while((--time_index > 0x00U) && ((uint32_t)RESET == flag_status));

        if((uint32_t)SET != flag_status){
            ret_status = HAL_ERR_TIMEOUT;
        }
        
        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
    }

    return ret_status;
}

/*!
    \brief      get current time and date
    \param[in]  none
    \param[out] rtc_initpara_struct: pointer to a hal_rtc_init_struct structure which contains 
                parameters for initialization of the rtc peripheral
                members of the structure and the member values are shown as below:
                  rtc_year: 0x0 - 0x99(BCD format)
                  rtc_month: 
                    the argument could be selected from enumeration <hal_rtc_month_enum>
                  rtc_date: 0x1 - 0x31(BCD format)
                  rtc_day_of_week:
                    the argument could be selected from enumeration <hal_rtc_day_of_week_enum>
                  rtc_hour: 0x1 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc_display_format chose
                  rtc_minute: 0x0 - 0x59(BCD format)
                  rtc_second: 0x0 - 0x59(BCD format)
                  rtc_subsecond: 0x0 - 0xFFFF
                  rtc_factor_asyn: 0x0 - 0x7F
                  rtc_factor_syn: 0x0 - 0x7FFF
                  rtc_am_pm: HAL_RTC_AM, HAL_RTC_PM
                  rtc_display_format: HAL_RTC_24HOUR, HAL_RTC_12HOUR
    \retval     none
*/
void hal_rtc_current_time_get(hal_rtc_init_struct* rtc_initpara_struct)
{
    uint32_t temp_tr = 0x00U, temp_dr = 0x00U, temp_pscr = 0x00U, temp_ctlr = 0x00U;
    uint16_t temp_ss = 0x00U;

    /* if BPSHAD bit is reset, reading RTC_SS will lock RTC_TIME and RTC_DATE automatically */
    temp_ss = (uint16_t)RTC_SS;    
    temp_tr = (uint32_t)RTC_TIME;   
    temp_dr = (uint32_t)RTC_DATE;
    temp_pscr = (uint32_t)RTC_PSC;
    temp_ctlr = (uint32_t)RTC_CTL;
    /* read RTC_DATE to unlock the 3 shadow registers */
    (void) (RTC_DATE);    
  
    /* get current time and construct rtc_parameter_struct structure */
    rtc_initpara_struct->rtc_year = (uint8_t)GET_DATE_YR(temp_dr);
    rtc_initpara_struct->rtc_month = (hal_rtc_month_enum)GET_DATE_MON(temp_dr);
    rtc_initpara_struct->rtc_date = (uint8_t)GET_DATE_DAY(temp_dr);
    rtc_initpara_struct->rtc_day_of_week = (hal_rtc_day_of_week_enum)GET_DATE_DOW(temp_dr);  
    rtc_initpara_struct->rtc_hour = (uint8_t)GET_TIME_HR(temp_tr);
    rtc_initpara_struct->rtc_minute = (uint8_t)GET_TIME_MN(temp_tr);
    rtc_initpara_struct->rtc_second = (uint8_t)GET_TIME_SC(temp_tr);
    rtc_initpara_struct->rtc_subsecond = temp_ss;    
    rtc_initpara_struct->rtc_factor_asyn = (uint16_t)GET_PSC_FACTOR_A(temp_pscr);
    rtc_initpara_struct->rtc_factor_syn = (uint16_t)GET_PSC_FACTOR_S(temp_pscr);
    rtc_initpara_struct->rtc_am_pm = (uint32_t)(temp_pscr & RTC_TIME_PM); 
    rtc_initpara_struct->rtc_display_format = (uint32_t)(temp_ctlr & RTC_CTL_CS);
}

/*!
    \brief      configure RTC alarm
    \param[in]  rtc_alarm_time: pointer to a hal_rtc_alarm_struct structure which contains 
                parameters for RTC alarm configuration
                members of the structure and the member values are shown as below:
                  rtc_alarm_mask: HAL_RTC_ALARM_NONE_MASK, HAL_RTC_ALARM_DATE_MASK, HAL_RTC_ALARM_HOUR_MASK,
                                  HAL_RTC_ALARM_MINUTE_MASK, HAL_RTC_ALARM_SECOND_MASK, HAL_RTC_ALARM_ALL_MASK
                  rtc_weekday_or_date: HAL_RTC_ALARM_DATE_SELECTED, HAL_RTC_ALARM_WEEKDAY_SELECTED
                  rtc_alarm_day: 1) 0x1 - 0x31(BCD format) if RTC_ALARM_DATE_SELECTED is set
                                 2) the argument could be selected from enumeration <hal_rtc_day_of_week_enum> 
                                    if RTC_ALARM_WEEKDAY_SELECTED is set
                  rtc_alarm_hour: 0x1 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc_display_format
                  rtc_alarm_minute: 0x0 - 0x59(BCD format)
                  rtc_alarm_second: 0x0 - 0x59(BCD format)
                  rtc_alarm_subsecond: 0x000 - 0x7FFF
                  rtc_alarm_subsecond_mask: HAL_RTC_MASK_SUBSECOND, HAL_RTC_MASK_SUBSECOND_1_14, HAL_RTC_MASK_SUBSECOND_2_14,
                                            HAL_RTC_MASK_SUBSECOND_3_14, HAL_RTC_MASK_SUBSECOND_4_14, HAL_RTC_MASK_SUBSECOND_5_14,
                                            HAL_RTC_MASK_SUBSECOND_6_14, HAL_RTC_MASK_SUBSECOND_7_14, HAL_RTC_MASK_SUBSECOND_8_14,
                                            HAL_RTC_MASK_SUBSECOND_9_14, HAL_RTC_MASK_SUBSECOND_10_14, HAL_RTC_MASK_SUBSECOND_11_14,
                                            HAL_RTC_MASK_SUBSECOND_12_14, HAL_RTC_MASK_SUBSECOND_13_14, HAL_RTC_MASK_SUBSECOND_14,
                                            HAL_RTC_MASK_SUBSECOND_NONE
                  rtc_am_pm: HAL_RTC_AM, HAL_RTC_PM
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_TIMEOUT, HAL_ERR_NONE,
                details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_alarm_config(hal_rtc_alarm_struct* rtc_alarm_time)
{
    uint32_t reg_alrm0td = 0x00U;
    uint32_t day_binary = 0x00U, hour_binary = 0x00U, minute_binary = 0x00U, second_binary = 0x00U;
    uint32_t rtc_display_format = 0U, subsecond_mask = 0U;

#if (1 == HAL_PARAMETER_CHECK)
    if(NULL == rtc_alarm_time){
        HAL_DEBUGE("pointer [rtc_alarm_time] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* check parameter rtc_alarm_time->rtc_alarm_mask */
    if(0x80808080U != (rtc_alarm_time->rtc_alarm_mask | 0x80808080U)){
        HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_mask] value is invalid");
        return HAL_ERR_VAL;
    } 
    
    rtc_display_format = RTC_CTL & RTC_CTL_CS;
    /* check parameter rtc_alarm_time->rtc_alarm_hour, and rtc_alarm_time->rtc_am_pm */
    hour_binary = rtc_bcd_2_normal(rtc_alarm_time->rtc_alarm_hour);
    if(0U != rtc_display_format){
        
        if((hour_binary < 1U)||(hour_binary > 12U)){
            HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_hour] value is invalid");
            return HAL_ERR_VAL;
        }
        if((HAL_RTC_AM != rtc_alarm_time->rtc_am_pm)&&(HAL_RTC_PM != rtc_alarm_time->rtc_am_pm)){
            HAL_DEBUGE("parameter [rtc_alarm_time->rtc_am_pm] value is invalid");
            return HAL_ERR_VAL;
        }        
    }else{
        if(hour_binary > 23U){
            HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_hour] value is invalid");
            return HAL_ERR_VAL;
        }
        rtc_alarm_time->rtc_am_pm = HAL_RTC_AM;
    }

    /* check parameter rtc_alarm_time->rtc_alarm_minute */
    minute_binary = rtc_bcd_2_normal(rtc_alarm_time->rtc_alarm_minute);
    if(minute_binary > 59){
        HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_minute] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter rtc_alarm_time->rtc_alarm_second */
    second_binary = rtc_bcd_2_normal(rtc_alarm_time->rtc_alarm_second);
    if(second_binary > 59){
        HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_second] value is invalid");
        return HAL_ERR_VAL;
    }   
    
    /* check parameter rtc_alarm_time->rtc_weekday_or_date, and rtc_alarm_time->rtc_alarm_day */
    day_binary = rtc_bcd_2_normal(rtc_alarm_time->rtc_alarm_day);
    if(HAL_RTC_ALARM_DATE_SELECTED == rtc_alarm_time->rtc_weekday_or_date){
        if((day_binary < 1U)||(day_binary > 31U)){
            HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_day] value is invalid");
            return HAL_ERR_VAL;
        }
    }else if(HAL_RTC_ALARM_WEEKDAY_SELECTED == rtc_alarm_time->rtc_weekday_or_date){
        if((day_binary < 1U)||(day_binary > 7U)){
            HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_day] value is invalid");
            return HAL_ERR_VAL;
        }    
    }else{
        HAL_DEBUGE("parameter [rtc_alarm_time->rtc_weekday_or_date] value is invalid");
        return HAL_ERR_VAL;    
    }  

    /* check parameter rtc_alarm_time->rtc_alarm_subsecond */
    if(rtc_alarm_time->rtc_alarm_subsecond > 0x7FFFU){
        HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_subsecond] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_alarm_time->rtc_alarm_subsecond_mask */
    subsecond_mask = rtc_alarm_time->rtc_alarm_subsecond_mask | RTC_ALRM0SS_MASKSSC;
    if(RTC_ALRM0SS_MASKSSC != subsecond_mask){
        HAL_DEBUGE("parameter [rtc_alarm_time->rtc_alarm_subsecond_mask] value is invalid");
        return HAL_ERR_VAL;
    }    
    
    reg_alrm0td = (rtc_alarm_time->rtc_alarm_mask | \
                 rtc_alarm_time->rtc_weekday_or_date | \
                 rtc_alarm_time->rtc_am_pm | \
                 ALRM0TD_DAY(rtc_alarm_time->rtc_alarm_day) | \
                 ALRM0TD_HR(rtc_alarm_time->rtc_alarm_hour) | \
                 ALRM0TD_MN(rtc_alarm_time->rtc_alarm_minute) | \
                 ALRM0TD_SC(rtc_alarm_time->rtc_alarm_second));

    /* disable RTC alarm */
    if(HAL_ERR_TIMEOUT == hal_rtc_alarm_disable()){
        HAL_DEBUGE("disable RTC alarm timeout");
        return HAL_ERR_TIMEOUT;    
    }    
    
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;     
    
    RTC_ALRM0TD = (uint32_t)reg_alrm0td;
    RTC_ALRM0SS = rtc_alarm_time->rtc_alarm_subsecond_mask | rtc_alarm_time->rtc_alarm_subsecond;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable RTC alarm
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rtc_alarm_enable(void)
{

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;     
    
    /* enable RTC alarm */
    RTC_CTL |= RTC_CTL_ALRM0EN;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      configure and enable RTC alarm by interrupt method, the function is non-blocking
    \param[in]  irq_handle: the callback handler of RTC interrupt
                  The structure member can be assigned as following parameters:
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
      \arg        hal_irq_handle_cb function pointer: the function is user customized,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_alarm_enable_interrupt(hal_rtc_irq_handle_cb irq_handle)
{
    if(NULL != irq_handle){
        /* clear alarm event flag */
        RTC_STAT &= (uint32_t)(~RTC_STAT_ALRM0F);
        
        /* disable the write protection */
        RTC_WPK = RTC_UNLOCK_KEY1;
        RTC_WPK = RTC_UNLOCK_KEY2; 
        
        /* enable alarm interrupt, and configure EXTI line with rising edge */
        rtc_flag_clear(RTC_FLAG_ALARM0);
        RTC_CTL |= RTC_CTL_ALRM0IE;
        hal_exti_internal_init(EXTI_LINE_17_RTC_ALARM, EXTI_INTERRUPT_TRIG_RISING);

        /* enable RTC alarm */
        RTC_CTL |= RTC_CTL_ALRM0EN;
                
        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
        
        rtc_irq.alarm_handle = irq_handle;  
    }else{
        HAL_DEBUGE("pointer [irq_handle] address is invalid");
        return HAL_ERR_ADDRESS;
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      disable RTC alarm
    \param[in]  none
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_alarm_disable(void)
{
    volatile uint32_t time_index = RTC_ALRM0WF_TIMEOUT;
    int32_t ret_status = HAL_ERR_TIMEOUT;
    uint32_t flag_status = RESET;

    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
    
    /* clear the state of alarm */
    RTC_CTL &= (uint32_t)(~RTC_CTL_ALRM0EN);

    /* disable alarm interrupt */
    RTC_CTL &= (uint32_t)~RTC_CTL_ALRM0IE;
    rtc_irq.alarm_handle = NULL;
    rtc_flag_clear(RTC_FLAG_ALARM0);
    EXTI_PD = (uint32_t)EXTI_17;    
    
    /* wait until ALRM0WF flag to be set after the alarm is disabled */
    do{
        flag_status = RTC_STAT & RTC_STAT_ALRM0WF;
    }while((--time_index > 0x00U) && ((uint32_t)RESET == flag_status));
    
    if((uint32_t)RESET != flag_status){     
        ret_status = HAL_ERR_NONE;
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return ret_status;
}

/*!
    \brief      get RTC alarm
    \param[in]  none
    \param[out] rtc_alarm_time: pointer to a hal_rtc_alarm_struct structure which contains 
                parameters for RTC alarm configuration
                members of the structure and the member values are shown as below:
                  rtc_alarm_mask: HAL_RTC_ALARM_NONE_MASK, HAL_RTC_ALARM_DATE_MASK, HAL_RTC_ALARM_HOUR_MASK,
                                  HAL_RTC_ALARM_MINUTE_MASK, HAL_RTC_ALARM_SECOND_MASK, HAL_RTC_ALARM_ALL_MASK
                  rtc_weekday_or_date: HAL_RTC_ALARM_DATE_SELECTED, HAL_RTC_ALARM_WEEKDAY_SELECTED
                  rtc_alarm_day: 1) 0x1 - 0x31(BCD format) if RTC_ALARM_DATE_SELECTED is set
                                 2) the argument could be selected from enumeration <hal_rtc_day_of_week_enum>
                                    if RTC_ALARM_WEEKDAY_SELECTED is set
                  rtc_alarm_hour: 0x1 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc_display_format
                  rtc_alarm_minute: 0x0 - 0x59(BCD format)
                  rtc_alarm_second: 0x0 - 0x59(BCD format)
                  rtc_alarm_subsecond: 0x000 - 0x7FFF
                  rtc_alarm_subsecond_mask: HAL_RTC_MASK_SUBSECOND, HAL_RTC_MASK_SUBSECOND_1_14, HAL_RTC_MASK_SUBSECOND_2_14,
                                            HAL_RTC_MASK_SUBSECOND_3_14, HAL_RTC_MASK_SUBSECOND_4_14, HAL_RTC_MASK_SUBSECOND_5_14,
                                            HAL_RTC_MASK_SUBSECOND_6_14, HAL_RTC_MASK_SUBSECOND_7_14, HAL_RTC_MASK_SUBSECOND_8_14,
                                            HAL_RTC_MASK_SUBSECOND_9_14, HAL_RTC_MASK_SUBSECOND_10_14, HAL_RTC_MASK_SUBSECOND_11_14,
                                            HAL_RTC_MASK_SUBSECOND_12_14, HAL_RTC_MASK_SUBSECOND_13_14, HAL_RTC_MASK_SUBSECOND_14,
                                            HAL_RTC_MASK_SUBSECOND_NONE
                  rtc_am_pm: HAL_RTC_AM, HAL_RTC_PM
    \retval     none
*/
void hal_rtc_alarm_get(hal_rtc_alarm_struct* rtc_alarm_time)
{
    uint32_t reg_alrm0td = 0x00U, reg_alrm0ss = 0x00U;

    /* get the value of RTC_ALRM0TD and RTC_ALRM0SS register */
    reg_alrm0td = RTC_ALRM0TD;
    reg_alrm0ss = RTC_ALRM0SS;

    /* get alarm parameters and construct the rtc_alarm_struct structure */
    rtc_alarm_time->rtc_alarm_mask = reg_alrm0td & RTC_ALARM_ALL_MASK; 
    rtc_alarm_time->rtc_am_pm = (uint32_t)(reg_alrm0td & RTC_ALRM0TD_PM);
    rtc_alarm_time->rtc_weekday_or_date = (uint32_t)(reg_alrm0td & RTC_ALRM0TD_DOWS);
    rtc_alarm_time->rtc_alarm_day = (uint8_t)GET_ALRM0TD_DAY(reg_alrm0td);
    rtc_alarm_time->rtc_alarm_hour = (uint8_t)GET_ALRM0TD_HR(reg_alrm0td);
    rtc_alarm_time->rtc_alarm_minute = (uint8_t)GET_ALRM0TD_MN(reg_alrm0td);
    rtc_alarm_time->rtc_alarm_second = (uint8_t)GET_ALRM0TD_SC(reg_alrm0td);
    rtc_alarm_time->rtc_alarm_subsecond = reg_alrm0ss & RTC_ALRM0SS_SSC;    
}

/*!
    \brief      poll for RTC alarm event, the function is blocking
    \param[in]  timeout_ms: the time cost for event polling
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_alarm_event_poll(uint32_t timeout_ms)
{    
    uint32_t flag_status = RESET;
    uint32_t tick_start = 0;
    
    tick_start = hal_basetick_count_get();
    /* wait until ALRM0F flag to be set after the alarm is enabled */
    do{
        flag_status = RTC_STAT & RTC_STAT_ALRM0F;
    }while((SET == hal_basetick_timeout_check(tick_start, timeout_ms)) && ((uint32_t)RESET == flag_status));
    
    if((uint32_t)RESET == flag_status){     
        return HAL_ERR_TIMEOUT;
    }
    
    rtc_flag_clear(RTC_FLAG_ALARM0);
    return HAL_ERR_NONE;
}

#if defined(GD32E230)
/*!
    \brief      configure RTC time-stamp
    \param[in]  edge: specify which edge to detect of time-stamp
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_TIMESTAMP_RISING_EDGE: rising edge is valid event edge for timestamp event
      \arg        HAL_RTC_TIMESTAMP_FALLING_EDGE: falling edge is valid event edge for timestamp event
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_timestamp_config(uint32_t edge)
{
    uint32_t reg_ctl = 0x00U;
    
#if (1 == HAL_PARAMETER_CHECK)
    if((HAL_RTC_TIMESTAMP_RISING_EDGE != edge) && (HAL_RTC_TIMESTAMP_FALLING_EDGE != edge)){
        HAL_DEBUGE("parameter [edge] value is invalid");
        return HAL_ERR_VAL;    
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* clear the bits to be configured in RTC_CTL */
    reg_ctl = (uint32_t)(RTC_CTL & (uint32_t)(~(RTC_CTL_TSEG | RTC_CTL_TSEN)));

    /* new configuration */
    reg_ctl |= (uint32_t)edge;
   
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL = (uint32_t)reg_ctl;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable RTC time-stamp
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rtc_timestamp_enable(void)
{   
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL |= RTC_CTL_TSEN;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      enable RTC time-stamp by interrupt method, the function is non-blocking
    \param[in]  irq_handle: the callback handler of RTC interrupt
                  The structure member can be assigned as following parameters:
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
      \arg        hal_irq_handle_cb function pointer: the function is user customized,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_timestamp_enable_interrupt(hal_rtc_irq_handle_cb irq_handle)
{   
    if(NULL != irq_handle){
        /* disable the write protection */
        RTC_WPK = RTC_UNLOCK_KEY1;
        RTC_WPK = RTC_UNLOCK_KEY2;
  
        /* enable timestamp interrupt, and configure EXTI line with rising edge */
        rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);        
        RTC_CTL |= RTC_CTL_TSIE;
        hal_exti_internal_init(EXTI_LINE_19_RTC_TAMPER_TIMESTAMP, EXTI_INTERRUPT_TRIG_RISING); 

        RTC_CTL |= RTC_CTL_TSEN;
             
        /* enable the write protection */
        RTC_WPK = RTC_LOCK_KEY;
        
        rtc_irq.timestamp_handle = irq_handle;              
    }else{
        HAL_DEBUGE("pointer [irq_handle] address is invalid");
        return HAL_ERR_ADDRESS;    
    }
    return HAL_ERR_NONE;
}

/*!
    \brief      disable RTC time-stamp
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rtc_timestamp_disable(void)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
    
    /* clear the TSEN bit */
    RTC_CTL &= (uint32_t)(~(RTC_CTL_TSEG | RTC_CTL_TSEN));
    
    /* disable timestamp interrupt */
    RTC_CTL &= ~RTC_CTL_TSIE;
    rtc_irq.timestamp_handle = NULL;
    rtc_flag_clear(RTC_FLAG_TIMESTAMP);
    rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);
    EXTI_PD = (uint32_t)EXTI_19;     

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}
#endif /* GD32E230 */

/*!
    \brief      get RTC timestamp time and date
    \param[in]  none
    \param[out] rtc_timestamp: pointer to a hal_rtc_timestamp_struct structure which contains 
                parameters for RTC time-stamp configuration
                members of the structure and the member values are shown as below:
                  rtc_timestamp_month: RTC_JAN, RTC_FEB, RTC_MAR, RTC_APR, RTC_MAY, RTC_JUN,
                                       RTC_JUL, RTC_AUG, RTC_SEP, RTC_OCT, RTC_NOV, RTC_DEC
                  rtc_timestamp_date: 0x1 - 0x31(BCD format)
                  rtc_timestamp_day: RTC_MONDAY, RTC_TUESDAY, RTC_WEDSDAY, RTC_THURSDAY, RTC_FRIDAY,
                                     RTC_SATURDAY, RTC_SUNDAY if RTC_ALARM_WEEKDAY_SELECTED is set
                  rtc_timestamp_hour: 0x0 - 0x12(BCD format) or 0x0 - 0x23(BCD format) depending on the rtc_display_format
                  rtc_timestamp_minute: 0x0 - 0x59(BCD format)
                  rtc_timestamp_second: 0x0 - 0x59(BCD format)
                  rtc_timestamp_subsecond: 0x0 - 0xFFFF
                  rtc_am_pm: HAL_RTC_AM, HAL_RTC_PM
    \retval     none
*/
void hal_rtc_timestamp_get(hal_rtc_timestamp_struct* rtc_timestamp)
{
    uint32_t temp_tts = 0x00U, temp_dts = 0x00U,temp_ssts = 0x00U;

    /* get the value of time_stamp registers */
    temp_tts = (uint32_t)RTC_TTS;
    temp_dts = (uint32_t)RTC_DTS;
    temp_ssts = (uint32_t)RTC_SSTS;
  
    /* get timestamp time and construct the rtc_timestamp_struct structure */
    rtc_timestamp->rtc_am_pm = (uint32_t)(temp_tts & RTC_TTS_PM);
    rtc_timestamp->rtc_timestamp_month = (uint8_t)GET_DTS_MON(temp_dts);
    rtc_timestamp->rtc_timestamp_date = (uint8_t)GET_DTS_DAY(temp_dts);
    rtc_timestamp->rtc_timestamp_day = (uint8_t)GET_DTS_DOW(temp_dts);
    rtc_timestamp->rtc_timestamp_hour = (uint8_t)GET_TTS_HR(temp_tts);
    rtc_timestamp->rtc_timestamp_minute = (uint8_t)GET_TTS_MN(temp_tts);
    rtc_timestamp->rtc_timestamp_second = (uint8_t)GET_TTS_SC(temp_tts);
    rtc_timestamp->rtc_timestamp_subsecond = temp_ssts;
}

/*!
    \brief      configure RTC tamper
    \param[in]  rtc_tamper: pointer to a hal_rtc_tamper_struct structure which contains 
                parameters for RTC tamper configuration
                members of the structure and the member values are shown as below:
                  rtc_tamper_source: HAL_RTC_TAMPER0, HAL_RTC_TAMPER1,(HAL_RTC_TAMPER0 only for GD32E230)
                  rtc_tamper_trigger: HAL_RTC_TAMPER_EDGE_RISING, HAL_RTC_TAMPER_EDGE_FALLING,
                                      HAL_RTC_TAMPER_LEVEL_LOW, HAL_RTC_TAMPER_LEVEL_HIGH
                  rtc_tamper_filter: HAL_RTC_FILTER_EDGE, HAL_RTC_FILTER_2_SAMPLE, HAL_RTC_FILTER_4_SAMPLE,
                                     HAL_RTC_FILTER_8_SAMPLE
                  rtc_tamper_sample_frequency: 
                    the argument could be selected from enumeration <hal_rtc_tamper_freq_enum>
                  rtc_tamper_precharge_enable: DISABLE, ENABLE
                  rtc_tamper_precharge_time: HAL_RTC_1_CLK_PRECHARGE, HAL_RTC_2_CLK_PRECHARGE, HAL_RTC_4_CLK_PRECHARGE,
                                             HAL_RTC_8_CLK_PRECHARGE
                  rtc_tamper_with_timestamp: DISABLE, ENABLE
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_tamper_config(hal_rtc_tamper_struct* rtc_tamper)
{
#if (1 == HAL_PARAMETER_CHECK)
    uint32_t val_mask = 0x0U;
    
    if(NULL == rtc_tamper){
        HAL_DEBUGE("pointer [rtc_tamper] address is invalid");
        return HAL_ERR_ADDRESS;
    }    

    /* check parameter rtc_tamper->rtc_tamper_source */
    if((HAL_RTC_TAMPER0 != rtc_tamper->rtc_tamper_source)&&(HAL_RTC_TAMPER1 != rtc_tamper->rtc_tamper_source)){
        HAL_DEBUGE("parameter [rtc_tamper->rtc_tamper_source] value is invalid");
        return HAL_ERR_VAL;
    } 

    /* check parameter rtc_tamper->rtc_tamper_trigger */
    val_mask = rtc_tamper->rtc_tamper_trigger | RTC_TAMP_TP0EG;
    if(RTC_TAMP_TP0EG != val_mask){
        HAL_DEBUGE("parameter [rtc_tamper->rtc_tamper_trigger] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter rtc_tamper->rtc_tamper_filter */
    val_mask = rtc_tamper->rtc_tamper_filter | RTC_TAMP_FLT;
    if(RTC_TAMP_FLT != val_mask){
        HAL_DEBUGE("parameter [rtc_tamper->rtc_tamper_filter] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter rtc_tamper->rtc_tamper_precharge_enable */
    if((DISABLE != rtc_tamper->rtc_tamper_precharge_enable)&&(ENABLE != rtc_tamper->rtc_tamper_precharge_enable)){
        HAL_DEBUGE("parameter [rtc_tamper->rtc_tamper_precharge_enable] value is invalid");
        return HAL_ERR_VAL;
    }

    /* check parameter rtc_tamper->rtc_tamper_precharge_time */
    val_mask = rtc_tamper->rtc_tamper_precharge_time | RTC_TAMP_PRCH;
    if(RTC_TAMP_PRCH != val_mask){
        HAL_DEBUGE("parameter [rtc_tamper->rtc_tamper_precharge_time] value is invalid");
        return HAL_ERR_VAL;
    }    

    /* check parameter rtc_tamper->rtc_tamper_with_timestamp */
    if((DISABLE != rtc_tamper->rtc_tamper_with_timestamp)&&(ENABLE != rtc_tamper->rtc_tamper_with_timestamp)){
        HAL_DEBUGE("parameter [rtc_tamper->rtc_tamper_with_timestamp] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable tamper */
    RTC_TAMP &= (uint32_t)~(rtc_tamper->rtc_tamper_source); 

    /* tamper filter must be used when the tamper source is voltage level detection */
    RTC_TAMP &= (uint32_t)~RTC_TAMP_FLT;
    
    /* the tamper source is voltage level detection */
    if(RTC_FLT_EDGE !=  rtc_tamper->rtc_tamper_filter){ 
        RTC_TAMP &= (uint32_t)~(RTC_TAMP_DISPU | RTC_TAMP_PRCH | RTC_TAMP_FREQ | RTC_TAMP_FLT);

        /* check if the tamper pin need precharge, if need, then configure the precharge time */
        if(DISABLE == rtc_tamper->rtc_tamper_precharge_enable){
            RTC_TAMP |=  (uint32_t)RTC_TAMP_DISPU;    
        }else{
            RTC_TAMP |= (uint32_t)(rtc_tamper->rtc_tamper_precharge_time);
        }

        RTC_TAMP |= (uint32_t)(rtc_tamper->rtc_tamper_sample_frequency);
        RTC_TAMP |= (uint32_t)(rtc_tamper->rtc_tamper_filter);
    }
    
    RTC_TAMP &= (uint32_t)~RTC_TAMP_TPTS;  
    
    if(DISABLE != rtc_tamper->rtc_tamper_with_timestamp){           
        /* the tamper event also cause a time-stamp event */
        RTC_TAMP |= (uint32_t)RTC_TAMP_TPTS;
    } 
    
    /* configure the tamper trigger */
    RTC_TAMP &= ((uint32_t)~((rtc_tamper->rtc_tamper_source) << RTC_TAMPER_TRIGGER_POS));    
    if(RTC_TAMPER_TRIGGER_EDGE_RISING != rtc_tamper->rtc_tamper_trigger){
        RTC_TAMP |= (uint32_t)((rtc_tamper->rtc_tamper_source)<< RTC_TAMPER_TRIGGER_POS);  
    }    

    return HAL_ERR_NONE;
}


/*!
    \brief      enable RTC tamper
    \param[in]  source: specify which tamper source to be enabled
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_TAMPER0
      \arg        HAL_RTC_TAMPER1
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_tamper_enable(uint32_t source)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter source */
    if((HAL_RTC_TAMPER0 != source)&&(HAL_RTC_TAMPER1 != source)){
        HAL_DEBUGE("parameter [source] value is invalid");
        return HAL_ERR_VAL;
    } 
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* enable tamper */
    RTC_TAMP |= source;

    return HAL_ERR_NONE;
}

/*!
    \brief      enable RTC tamper by interrupt method, the function is non-blocking
    \param[in]  source: specify which tamper source to be enabled
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_TAMPER0
      \arg        HAL_RTC_TAMPER1
    \param[in]  irq_handle: the callback handler of RTC interrupt
                  The structure member can be assigned as following parameters:
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
      \arg        hal_irq_handle_cb function pointer: the function is user customized,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     error code: HAL_ERR_ADDRESS, HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_tamper_enable_interrupt(uint32_t source, hal_rtc_irq_handle_cb irq_handle)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter source */
    if((HAL_RTC_TAMPER0 != source)&&(HAL_RTC_TAMPER1 != source)){
        HAL_DEBUGE("parameter [source] value is invalid");
        return HAL_ERR_VAL;
    } 
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    if(NULL != irq_handle){
        /* enable tamper interrupt, and configure EXTI line with rising edge */
        if(HAL_RTC_TAMPER1 == source){
            rtc_flag_clear(RTC_FLAG_TAMP1);        
        }
#if defined(GD32E230)
        else{

            rtc_flag_clear(RTC_FLAG_TAMP0);         
        }
        
        if(RESET != (RTC_TAMP & RTC_TAMP_TPTS)){
            rtc_flag_clear(RTC_FLAG_TIMESTAMP);
            rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);        
        }        
        RTC_TAMP |= RTC_TAMP_TPIE;
        hal_exti_internal_init(EXTI_LINE_19_RTC_TAMPER_TIMESTAMP, EXTI_INTERRUPT_TRIG_RISING);
#endif /* GD32E230 */
        /* enable tamper */
        RTC_TAMP |= source;
        
        if(HAL_RTC_TAMPER1 == source){
            rtc_irq.tamper1_handle = irq_handle;     
        }else{
            rtc_irq.tamper0_handle = irq_handle;    
        }
    }else{
        HAL_DEBUGE("pointer [irq_handle] address is invalid");
        return HAL_ERR_ADDRESS;    
    }   
    return HAL_ERR_NONE;
}

/*!
    \brief      disable RTC tamper
    \param[in]  source: specify which tamper source to be disabled
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_TAMPER0
      \arg        HAL_RTC_TAMPER1
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_tamper_disable(uint32_t source)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter source */
    if((HAL_RTC_TAMPER0 != source)&&(HAL_RTC_TAMPER1 != source)){
        HAL_DEBUGE("parameter [source] value is invalid");
        return HAL_ERR_VAL;
    } 
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable tamper */    
    RTC_TAMP &= (uint32_t)~source;
    RTC_TAMP &= ~RTC_TAMP_TPIE;
    if(HAL_RTC_TAMPER1 == source){
        rtc_irq.tamper1_handle = NULL;
        rtc_flag_clear(RTC_FLAG_TAMP1);         
    }else{
#if defined(GD32E230)
        rtc_irq.tamper0_handle = NULL;
        rtc_flag_clear(RTC_FLAG_TAMP0);
#endif /* GD32E230 */
    }
#if defined(GD32E230)
    if(RESET != (RTC_TAMP & RTC_TAMP_TPTS)){
        rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);        
    }
#endif /* GD32E230 */
    EXTI_PD = (uint32_t)EXTI_19;

    return HAL_ERR_NONE;    
}

#if defined(GD32E230)
/*!
    \brief      poll for RTC timestamp event, the function is blocking
    \param[in]  timeout_ms: the time cost for event polling
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_HARDWARE, HAL_ERR_NONE,
                details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_timestamp_event_poll(uint32_t timeout_ms)
{  
    uint32_t flag_status = RESET;
    uint32_t tick_start = 0;
    
    /* wait until TSF flag to be set */
    flag_status = RTC_STAT & RTC_STAT_TSF;
    tick_start = hal_basetick_count_get();
    while((SET == hal_basetick_timeout_check(tick_start, timeout_ms)) && ((uint32_t)RESET == flag_status)){
        flag_status = RTC_STAT & RTC_STAT_TSF;
        if((uint32_t)RESET != (RTC_STAT & RTC_STAT_TSOVRF)){
            return HAL_ERR_HARDWARE;
        }
    }
    
    if((uint32_t)RESET == flag_status){     
        return HAL_ERR_TIMEOUT;
    }
    
    rtc_flag_clear(RTC_FLAG_TIMESTAMP);
    rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      poll for RTC tamper0 event, the function is blocking
    \param[in]  timeout_ms: the time cost for event polling
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_tamper0_event_poll(uint32_t timeout_ms)
{  
    uint32_t flag_status = RESET;
    uint32_t tick_start = 0;
    
    tick_start = hal_basetick_count_get();
    /* wait until TP0F flag to be set */
    do{
        flag_status = RTC_STAT & RTC_STAT_TP0F;
    }while((SET == hal_basetick_timeout_check(tick_start, timeout_ms)) && ((uint32_t)RESET == flag_status));
    
    if((uint32_t)RESET == flag_status){     
        return HAL_ERR_TIMEOUT;
    }
    
    rtc_flag_clear(RTC_FLAG_TAMP0);
    
    return HAL_ERR_NONE;
}
#endif /* GD32E230 */

/*!
    \brief      poll for RTC tamper1 event, the function is blocking
    \param[in]  timeout_ms: the time cost for event polling
    \param[out] none
    \retval     error code: HAL_ERR_TIMEOUT, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_tamper1_event_poll(uint32_t timeout_ms)
{   
    uint32_t flag_status = RESET;
    uint32_t tick_start = 0;

    tick_start = hal_basetick_count_get();
    /* wait until TP1F flag to be set */
    do{
        flag_status = RTC_STAT & RTC_STAT_TP1F;
    }while((SET == hal_basetick_timeout_check(tick_start, timeout_ms)) && ((uint32_t)RESET == flag_status));
    
    if((uint32_t)RESET == flag_status){     
        return HAL_ERR_TIMEOUT;
    }
    
    rtc_flag_clear(RTC_FLAG_TAMP1);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      RTC interrupt handler content function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rtc_irq(void)
{
    uint32_t int_en = 0U;
    
    /* alarm interrupt handle */
    int_en = RTC_CTL & RTC_CTL_ALRM0IE;
    if((RESET != rtc_flag_get(RTC_FLAG_ALARM0)) && (RESET != int_en)){
        rtc_flag_clear(RTC_FLAG_ALARM0);
        EXTI_PD = (uint32_t)EXTI_17;
        
        if(NULL != rtc_irq.alarm_handle){
            rtc_irq.alarm_handle();
        }        
    }
#if defined(GD32E230)
    /* timestamp interrupt handle */
    int_en = RTC_CTL & RTC_CTL_TSIE;
    if((RESET != rtc_flag_get(RTC_FLAG_TIMESTAMP)) && (RESET != int_en)){
        if(NULL != rtc_irq.timestamp_handle){
            rtc_irq.timestamp_handle();
        }        
        
        rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);
        EXTI_PD = (uint32_t)EXTI_19;              
    }
    
    /* tamper0 interrupt handle */
    int_en = RTC_TAMP & RTC_TAMP_TPIE;
    if((RESET != rtc_flag_get(RTC_FLAG_TAMP0)) && (RESET != int_en)){
        rtc_flag_clear(RTC_FLAG_TAMP0);
        
        if(NULL != rtc_irq.tamper0_handle){
            rtc_irq.tamper0_handle();
        }
        
        rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);        
        EXTI_PD = (uint32_t)EXTI_19;
               
    }
#endif /* GD32E230 */
    /* tamper1 interrupt handle */
    int_en = RTC_TAMP & RTC_TAMP_TPIE;
    if((RESET != rtc_flag_get(RTC_FLAG_TAMP1)) && (RESET != int_en)){
        rtc_flag_clear(RTC_FLAG_TAMP1);
        
        if(NULL != rtc_irq.tamper1_handle){
            rtc_irq.tamper1_handle();
        } 
#if defined(GD32E230)
        rtc_flag_clear(RTC_FLAG_TIMESTAMP);
        rtc_flag_clear(RTC_FLAG_TIMESTAMP_OVERFLOW);
#endif /* GD32E230 */
        EXTI_PD = (uint32_t)EXTI_19;
               
    }
}

/*!
    \brief      set user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  p_irq: the structure that contains callback handlers of RTC interrupt
                  The structure member can be assigned as following parameters:
      \arg        NULL: The corresponding callback mechanism is out of use, and
                    disable corresponding interrupt
      \arg        HAL_INTERRUPT_ENABLE_ONLY: The corresponding callback mechanism is out of use, 
                    while enable corresponding interrupt
      \arg        hal_irq_handle_cb function pointer: the function is user customized,
                    the corresponding callback mechanism is in use, and enable corresponding interrupt
    \param[out] none
    \retval     none
*/
void hal_rtc_irq_handle_set(hal_rtc_irq_struct *p_irq)
{
    if(NULL != p_irq->alarm_handle){
        /* enable alarm interrupt, and configure EXTI line with rising edge */
        RTC_CTL |= RTC_CTL_ALRM0IE;
        hal_exti_internal_init(EXTI_LINE_17_RTC_ALARM, EXTI_INTERRUPT_TRIG_RISING);
        
        rtc_irq.alarm_handle = p_irq->alarm_handle; 
    }else{
        RTC_CTL &= (uint32_t)~RTC_CTL_ALRM0IE;
        EXTI_INTEN &= (uint32_t)~EXTI_17;
        
        rtc_irq.alarm_handle = NULL; 
    }
#if defined(GD32E230)
    if(NULL != p_irq->timestamp_handle){
        /* enable timestamp interrupt, and configure EXTI line with rising edge */
        RTC_CTL |= RTC_CTL_TSIE;
        hal_exti_internal_init(EXTI_LINE_19_RTC_TAMPER_TIMESTAMP, EXTI_INTERRUPT_TRIG_RISING);

        rtc_irq.timestamp_handle = p_irq->timestamp_handle;        
    }else{
        RTC_CTL &= (uint32_t)~RTC_CTL_TSIE;
        
        rtc_irq.timestamp_handle = NULL;         
    }
#endif /* GD32E230 */
    if(NULL != p_irq->tamper0_handle){
        /* enable tamper0 interrupt, and configure EXTI line with rising edge */
        RTC_TAMP |= RTC_TAMP_TPIE;
        hal_exti_internal_init(EXTI_LINE_19_RTC_TAMPER_TIMESTAMP, EXTI_INTERRUPT_TRIG_RISING);

        rtc_irq.tamper0_handle = p_irq->tamper0_handle;          
    }else{
        rtc_irq.tamper0_handle = NULL;    
    }

    if(NULL != p_irq->tamper1_handle){
        /* enable tamper1 interrupt, and configure EXTI line with rising edge */
        RTC_TAMP |= RTC_TAMP_TPIE;
        hal_exti_internal_init(EXTI_LINE_19_RTC_TAMPER_TIMESTAMP, EXTI_INTERRUPT_TRIG_RISING); 

        rtc_irq.tamper1_handle = p_irq->tamper1_handle;          
    }else{
        rtc_irq.tamper1_handle = NULL;    
    }

    if((NULL == p_irq->tamper0_handle)&&(NULL == p_irq->tamper1_handle)){
        RTC_CTL &= (uint32_t)~RTC_TAMP_TPIE;   
    }
    
    if((NULL == p_irq->tamper0_handle)&&(NULL == p_irq->tamper1_handle)&&(NULL == p_irq->timestamp_handle)){
        EXTI_INTEN &= (uint32_t)~EXTI_19;   
    }    
}

/*!
    \brief      reset all user-defined interrupt callback function, 
                which will be registered and called when corresponding interrupt be triggered
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rtc_irq_handle_all_reset(void)
{
    rtc_irq.alarm_handle = NULL; 
    rtc_irq.timestamp_handle = NULL;         
    rtc_irq.tamper0_handle = NULL;    
    rtc_irq.tamper1_handle = NULL;    
}

/*!
    \brief      write data to backup register
    \param[in]  backup_index: the index of backup register
      \arg        0 - 4
    \param[in]  data: the data to be written to backup register
      \arg        0x0 - 0xFFFFFFFF
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_backup_data_write(uint32_t backup_index, uint32_t data)
{
    uint32_t reg = 0x00U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter backup_index */
    if(backup_index > 4U){
        HAL_DEBUGE("parameter [backup_index] value is invalid");
        return HAL_ERR_VAL;
    }    
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    reg = RTC_BKP0 + (backup_index << 2);
    *(__IO uint32_t *)reg = data;
    
    return HAL_ERR_NONE;
}

/*!
    \brief      read data from backup register
    \param[in]  backup_index: the index of backup register
      \arg        0 - 4
    \param[out] data: the data read from backup register
      \arg        0x0 - 0xFFFFFFFF
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_backup_data_read(uint32_t backup_index, uint32_t *data)
{
    uint32_t reg = 0x00U;
    
#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter backup_index */
    if(backup_index > 4U){
        HAL_DEBUGE("parameter [backup_index] value is invalid");
        return HAL_ERR_VAL;
    } 
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    reg = RTC_BKP0 + (backup_index << 2);
    *data = (*(__IO uint32_t *)reg);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      enable RTC bypass shadow registers function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rtc_bypass_shadow_enable(void)
{ 
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL |= (uint8_t)RTC_CTL_BPSHAD;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

/*!
    \brief      disable RTC bypass shadow registers function
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hal_rtc_bypass_shadow_disable(void)
{ 
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL &= (uint8_t)~RTC_CTL_BPSHAD;

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}

#if defined(GD32E230)
/*!
    \brief      enable RTC reference clock detection function
    \param[in]  none
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_refclock_detection_enable(void)
{
    ErrStatus error_status = ERROR;
    int32_t ret_status = HAL_ERR_TIMEOUT;
    
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* enter init mode */
    error_status = rtc_init_mode_enter();

    if(ERROR != error_status){
        RTC_CTL |= (uint32_t)RTC_CTL_REFEN;
        /* exit init mode */
        rtc_init_mode_exit();
        
        ret_status = HAL_ERR_NONE;
    }else{
        HAL_DEBUGE("rtc enter init mode timeout");
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return ret_status;
}

/*!
    \brief      disable RTC reference clock detection function
    \param[in]  none
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_TIMEOUT, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_refclock_detection_disable(void)
{
    ErrStatus error_status = ERROR;
    int32_t ret_status = HAL_ERR_TIMEOUT;
    
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* enter init mode */
    error_status = rtc_init_mode_enter();

    if(ERROR != error_status){ 
        RTC_CTL &= (uint32_t)~RTC_CTL_REFEN;
        /* exit init mode */
        rtc_init_mode_exit();
    
        ret_status = HAL_ERR_NONE;
    }else{
        HAL_DEBUGE("rtc enter init mode timeout");
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return ret_status;
}

/*!
    \brief      configure rtc alternate output source
    \param[in]  source: specify signal to output
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_CALIBRATION_OUT_512HZ: when the LSE freqency is 32768Hz and the RTC_PSC 
                                         is the default value, output 512Hz signal
      \arg        HAL_RTC_CALIBRATION_OUT_1HZ: when the LSE freqency is 32768Hz and the RTC_PSC 
                                       is the default value, output 512Hz signal
      \arg        HAL_RTC_CALIBRATION_OUT_DISABLE: calibration output is disabled
      \arg        HAL_RTC_ALARM_OUT_HIGH: when the  alarm flag is set, the output pin is high
      \arg        HAL_RTC_ALARM_OUT_LOW: when the  Alarm flag is set, the output pin is low
      \arg        HAL_RTC_ALARM_OUT_DISABLE: disable alarm flag output
    \param[in]  mode: specify the output pin (PC13) mode when output alarm signal
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_ALARM_OUTPUT_OD: open drain mode
      \arg        HAL_RTC_ALARM_OUTPUT_PP: push pull mode
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_NONE, details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_output_config(uint32_t source, uint32_t mode)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter add */
    if((HAL_RTC_CALIBRATION_OUT_512HZ != source)&&(HAL_RTC_CALIBRATION_OUT_1HZ != source)&& \
       (HAL_RTC_ALARM_OUT_HIGH != source)&&(HAL_RTC_ALARM_OUT_LOW != source)&& \
       (HAL_RTC_CALIBRATION_OUT_DISABLE != source)&&(HAL_RTC_ALARM_OUT_DISABLE != source)){
        HAL_DEBUGE("parameter [source] value is invalid");
        return HAL_ERR_VAL;
    } 

    /* check parameter mode */
    if((HAL_RTC_ALARM_OUTPUT_OD != mode)&&(HAL_RTC_ALARM_OUTPUT_PP != mode)){
        HAL_DEBUGE("parameter [mode] value is invalid");
        return HAL_ERR_VAL;
    } 
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    /* disable calibration output */
    if(HAL_RTC_CALIBRATION_OUT_DISABLE == source){
        RTC_CTL &= (uint32_t)~RTC_CTL_COEN;
    /* disable alarm output */    
    }else if(HAL_RTC_CALIBRATION_OUT_DISABLE == source){
        RTC_CTL &= (uint32_t)~RTC_CTL_OS;
    /* alarm/calibration output */    
    }else{
        RTC_CTL &= (uint32_t)~(RTC_CTL_COEN | RTC_CTL_OS | RTC_CTL_OPOL | RTC_CTL_COS);
        RTC_CTL |= (uint32_t)(source);
    }
    /* alarm output */
    if((uint32_t)RESET != (source & RTC_OS_ENABLE)){
        RTC_TAMP &= (uint32_t)~(RTC_TAMP_PC13VAL);
        RTC_TAMP |= (uint32_t)(mode);  
    }
    
    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
    
    return HAL_ERR_NONE;
}
#endif /* GD32E230 */

/*!
    \brief      configure RTC calibration register
    \param[in]  window: select calibration window
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_CALIBRATION_32S: 2exp20 RTCCLK cycles, 32s if RTCCLK = 32768 Hz
      \arg        HAL_RTC_CALIBRATION_16S: 2exp19 RTCCLK cycles, 16s if RTCCLK = 32768 Hz
      \arg        HAL_RTC_CALIBRATION_8S: 2exp18 RTCCLK cycles, 8s if RTCCLK = 32768 Hz
    \param[in]  plus: add RTC clock or not
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_CALIBRATION_PLUS_SET: add one RTC clock every 2048 rtc clock
      \arg        HAL_RTC_CALIBRATION_PLUS_RESET: no effect
    \param[in]  minus: the RTC clock to minus during the calibration window(0x0 - 0x1FF)
    \param[out] none
    \retval     error code: HAL_ERR_VAL, HAL_ERR_TIMEOUT, HAL_ERR_NONE,
                details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_calibration_config(uint32_t window, uint32_t plus, uint32_t minus)
{
    uint32_t time_index = RTC_HRFC_TIMEOUT;
    int32_t ret_status = HAL_ERR_TIMEOUT;
    uint32_t flag_status = RESET;

#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter window */
    if((HAL_RTC_CALIBRATION_32S != window)&&(HAL_RTC_CALIBRATION_16S != window)&&(HAL_RTC_CALIBRATION_8S != window)){
        HAL_DEBUGE("parameter [window] value is invalid");
        return HAL_ERR_VAL;
    }    

    /* check parameter plus */
    if((HAL_RTC_CALIBRATION_PLUS_SET != plus)&&(HAL_RTC_CALIBRATION_PLUS_RESET != plus)){
        HAL_DEBUGE("parameter [plus] value is invalid");
        return HAL_ERR_VAL;
    }
    
    /* check parameter minus */
    if(minus > 0x1FFU){
        HAL_DEBUGE("parameter [minus] value is invalid");
        return HAL_ERR_VAL;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;    
    
    /* check if a calibration operation is ongoing */        
    do{
        flag_status = RTC_STAT & RTC_STAT_SCPF;
    }while((--time_index > 0x00U) && ((uint32_t)RESET != flag_status));
    
    if((uint32_t)RESET == flag_status){
        RTC_HRFC = (uint32_t)(window | plus | HRFC_CMSK(minus));
        ret_status = HAL_ERR_NONE;
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return ret_status;
}

/*!
    \brief      ajust the daylight saving time by adding or substracting one hour from the current time
    \param[in]  operation: hour ajustment operation
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_DAYLIGHTSAVING_1HOUR_ADD: add one hour
      \arg        HAL_RTC_DAYLIGHTSAVING_1HOUR_SUBTRACT: substract one hour
      \arg        HAL_RTC_DAYLIGHTSAVING_NONE: no add or subtract one hour
    \param[in]  record: daylight saving mark operation
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_RECORD_DAYLIGHTSAVING_SET: set daylight saving mark
      \arg        HAL_RTC_RECORD_DAYLIGHTSAVING_RESET: reset daylight saving mark
    \param[out] none
    \retval     none
*/
void hal_rtc_daylight_saving_time_adjust(uint32_t operation, uint32_t record)
{
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;

    RTC_CTL &= (uint32_t)(~(RTC_CTL_A1H | RTC_CTL_S1H | RTC_CTL_DSM));
    RTC_CTL |= (uint32_t)(operation | record);

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;
}


/*!
    \brief      ajust RTC second or subsecond value of current time
    \param[in]  add: add 1s to current time or not
                only one parameter can be selected which is shown as below:
      \arg        HAL_RTC_ADD1S_RESET: no effect
      \arg        HAL_RTC_ADD1S_SET: add 1s to current time
    \param[in]  minus: number of subsecond to minus from current time(0x0 - 0x7FFF)
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_NO_SUPPORT, HAL_ERR_TIMEOUT, HAL_ERR_VAL,
                details refer to gd32e23x_hal.h
*/
int32_t hal_rtc_sync_with_remote_clock_adjust(uint32_t add, uint32_t minus)
{
    uint32_t time_index = RTC_SHIFTCTL_TIMEOUT;
    int32_t ret_status = HAL_ERR_NONE;
    uint32_t flag_status = RESET;
    uint32_t temp=0U;

#if (1 == HAL_PARAMETER_CHECK)
    /* check parameter add */
    if((HAL_RTC_ADD1S_RESET != add)&&(HAL_RTC_ADD1S_SET != add)){
        HAL_DEBUGE("parameter [add] value is invalid");
        return HAL_ERR_VAL;
    }    

    /* check parameter minus */
    if(minus > 0x7FFFU){
        HAL_DEBUGE("parameter [minus] value is invalid");
        return HAL_ERR_VAL;
    }    
#endif /* 1 == HAL_PARAMETER_CHECK */
    
    /* disable the write protection */
    RTC_WPK = RTC_UNLOCK_KEY1;
    RTC_WPK = RTC_UNLOCK_KEY2;
    
    /* check if a shift operation is ongoing */    
    do{
        flag_status = RTC_STAT & RTC_STAT_SOPF;
    }while((--time_index > 0x00U) && ((uint32_t)RESET != flag_status));
  
    temp = RTC_CTL & RTC_CTL_REFEN;
    /* check if the function of reference clock detection is disabled */
    if(((uint32_t)RESET == flag_status) && (RESET == temp)){  
        RTC_SHIFTCTL = (uint32_t)(add | SHIFTCTL_SFS(minus));
        if((uint32_t)RESET == (RTC_CTL & RTC_CTL_BPSHAD)){
            ret_status = hal_rtc_register_sync_wait();
        }            
    }else if(((uint32_t)RESET == flag_status) && (RESET != temp)){
        HAL_DEBUGE("reference clock detection function is enabled ");
        ret_status = HAL_ERR_NO_SUPPORT;
    }else{
        HAL_DEBUGE("shift operation timeout ");
        ret_status = HAL_ERR_TIMEOUT;    
    }

    /* enable the write protection */
    RTC_WPK = RTC_LOCK_KEY;

    return ret_status;
}

/*!
    \brief      convert from  BCD format to binary format
    \param[in]  data: data to be converted
    \param[out] none
    \retval     converted data
*/
uint8_t rtc_bcd_2_normal(uint8_t data)
{
    uint8_t temp = 0U;
    temp = ((uint8_t)(data & (uint8_t)0xF0) >> (uint8_t)0x4) * 10;
    return (temp + (data & (uint8_t)0x0F));
}

/*!
    \brief      convert from  BCD format to binary format
    \param[in]  data: data to be converted
    \param[out] none
    \retval     converted data
*/
uint8_t rtc_normal_2_bcd(uint8_t data)
{
    uint8_t bcd_high = 0;

    while (data >= 10){
        bcd_high++;
        data -= 10;
    }

    return  ((uint8_t)(bcd_high << 4) | data);
}
