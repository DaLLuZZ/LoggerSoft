/*!
    \file    gd32e23x_hal_rtc.h
    \brief   definitions for the RTC
    
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

#ifndef GD32E23X_HAL_RTC_H
#define GD32E23X_HAL_RTC_H

#include "gd32e23x_hal.h"

/* constants definitions */
typedef enum {
    HAL_RTC_INIT_STRUCT,                                                    /*!< structure <hal_rtc_init_struct> */
    HAL_RTC_ALARM_STRUCT,                                                   /*!< structure <hal_rtc_alarm_struct> */
    HAL_RTC_TIMESTAMP_STRUCT,                                               /*!< structure <hal_rtc_timestamp_struct> */
    HAL_RTC_TAMPER_STRUCT,                                                  /*!< structure <hal_rtc_tamper_struct> */
    HAL_RTC_IRQ_STRUCT,                                                     /*!< structure <hal_rtc_irq_struct> */
} hal_rtc_struct_type_enum;

/* RTC month definitions */
typedef enum{
    HAL_RTC_JAN = RTC_JAN,                                                  /*!< Janurary */
    HAL_RTC_FEB = RTC_FEB,                                                  /*!< February */
    HAL_RTC_MAR = RTC_MAR,                                                  /*!< March */
    HAL_RTC_APR = RTC_APR,                                                  /*!< April */
    HAL_RTC_MAY = RTC_MAY,                                                  /*!< May */
    HAL_RTC_JUN = RTC_JUN,                                                  /*!< June */
    HAL_RTC_JUL = RTC_JUL,                                                  /*!< July */
    HAL_RTC_AUG = RTC_AUG,                                                  /*!< August */
    HAL_RTC_SEP = RTC_SEP,                                                  /*!< September */
    HAL_RTC_OCT = RTC_OCT,                                                  /*!< October */
    HAL_RTC_NOV = RTC_NOV,                                                  /*!< November */
    HAL_RTC_DEC = RTC_DEC,                                                  /*!< December */    
} hal_rtc_month_enum;

/* day of a week definitions */
typedef enum{
    HAL_RTC_MONDAY = RTC_MONDAY,                                            /*!< Monday */
    HAL_RTC_TUESDAY = RTC_TUESDAY,                                          /*!< Tuesday */
    HAL_RTC_WEDSDAY = RTC_WEDSDAY,                                          /*!< Wednesday */
    HAL_RTC_THURSDAY = RTC_THURSDAY,                                        /*!< Thursday */
    HAL_RTC_FRIDAY = RTC_FRIDAY,                                            /*!< Friday */
    HAL_RTC_SATURDAY = RTC_SATURDAY,                                        /*!< Saturday */
    HAL_RTC_SUNDAY = RTC_SUNDAY,                                            /*!< Sunday */
} hal_rtc_day_of_week_enum;

/* RTC tamper event sampling frequency definitions */
typedef enum{
    HAL_RTC_FREQ_DIV32768 = RTC_FREQ_DIV32768,                              /*!< sample once every 32768 RTCCLK(1Hz if RTCCLK=32.768KHz) */
    HAL_RTC_FREQ_DIV16384 = RTC_FREQ_DIV16384,                              /*!< sample once every 16384 RTCCLK(2Hz if RTCCLK=32.768KHz) */
    HAL_RTC_FREQ_DIV8192 = RTC_FREQ_DIV8192,                                /*!< sample once every 8192 RTCCLK(4Hz if RTCCLK=32.768KHz) */
    HAL_RTC_FREQ_DIV4096 = RTC_FREQ_DIV4096,                                /*!< sample once every 4096 RTCCLK(8Hz if RTCCLK=32.768KHz) */
    HAL_RTC_FREQ_DIV2048 = RTC_FREQ_DIV2048,                                /*!< sample once every 2048 RTCCLK(16Hz if RTCCLK=32.768KHz) */
    HAL_RTC_FREQ_DIV1024 = RTC_FREQ_DIV1024,                                /*!< sample once every 1024 RTCCLK(32Hz if RTCCLK=32.768KHz) */
    HAL_RTC_FREQ_DIV512 = RTC_FREQ_DIV512,                                  /*!< sample once every 512 RTCCLK(64Hz if RTCCLK=32.768KHz) */
    HAL_RTC_FREQ_DIV256 = RTC_FREQ_DIV256,                                  /*!< sample once every 256 RTCCLK(128Hz if RTCCLK=32.768KHz) */
} hal_rtc_tamper_freq_enum;

typedef void (*hal_rtc_irq_handle_cb)(void);

/* structure for initialization of the RTC */
typedef struct
{
    uint8_t rtc_year;                                                       /*!< RTC year value: 0x0 - 0x99(BCD format) */
    hal_rtc_month_enum rtc_month;                                           /*!< RTC month value */
    uint8_t rtc_date;                                                       /*!< RTC date value: 0x1 - 0x31(BCD format) */
    hal_rtc_day_of_week_enum rtc_day_of_week;                               /*!< RTC weekday value */
    uint8_t rtc_hour;                                                       /*!< RTC hour value */
    uint8_t rtc_minute;                                                     /*!< RTC minute value: 0x0 - 0x59(BCD format) */
    uint8_t rtc_second;                                                     /*!< RTC second value: 0x0 - 0x59(BCD format) */
    uint16_t rtc_subsecond;                                                 /*!< RTC subsecond value: 0x0 - 0xFFFF */    
    uint16_t rtc_factor_asyn;                                               /*!< RTC asynchronous prescaler value: 0x0 - 0x7F */
    uint16_t rtc_factor_syn;                                                /*!< RTC synchronous prescaler value: 0x0 - 0x7FFF */
    uint32_t rtc_am_pm;                                                     /*!< RTC AM/PM value */
    uint32_t rtc_display_format;                                            /*!< RTC time notation */
}hal_rtc_init_struct;

/* structure for RTC alarm configuration */
typedef struct
{
    uint32_t rtc_alarm_mask;                                                /*!< RTC alarm mask */   
    uint32_t rtc_weekday_or_date;                                           /*!< specify RTC alarm is on date or weekday */
    uint8_t rtc_alarm_day;                                                  /*!< RTC alarm date or weekday value*/
    uint8_t rtc_alarm_hour;                                                 /*!< RTC alarm hour value */
    uint8_t rtc_alarm_minute;                                               /*!< RTC alarm minute value: 0x0 - 0x59(BCD format) */
    uint8_t rtc_alarm_second;                                               /*!< RTC alarm second value: 0x0 - 0x59(BCD format) */
    uint32_t rtc_alarm_subsecond;                                           /*!< RTC alarm subsecond value: (0x000 - 0x7FFF) */    
    uint32_t rtc_alarm_subsecond_mask;                                      /*!< RTC alarm subsecond mask */ 
    uint32_t rtc_am_pm;                                                     /*!< RTC alarm AM/PM value */
}hal_rtc_alarm_struct;

/* structure for RTC time-stamp configuration */
typedef struct
{
    uint8_t rtc_timestamp_month;                                            /*!< RTC time-stamp month value */
    uint8_t rtc_timestamp_date;                                             /*!< RTC time-stamp date value: 0x1 - 0x31(BCD format) */
    uint8_t rtc_timestamp_day;                                              /*!< RTC time-stamp weekday value */
    uint8_t rtc_timestamp_hour;                                             /*!< RTC time-stamp hour value */
    uint8_t rtc_timestamp_minute;                                           /*!< RTC time-stamp minute value: 0x0 - 0x59(BCD format) */
    uint8_t rtc_timestamp_second;                                           /*!< RTC time-stamp second value: 0x0 - 0x59(BCD format) */
    uint32_t rtc_timestamp_subsecond;                                       /*!< RTC time-stamp subsecond value: 0x0 - 0xFFFF */    
    uint32_t rtc_am_pm;                                                     /*!< RTC time-stamp AM/PM value */
}hal_rtc_timestamp_struct;

/* structure for RTC tamper configuration */
typedef struct
{
    uint32_t rtc_tamper_source;                                             /*!< RTC tamper source */
    uint32_t rtc_tamper_trigger;                                            /*!< RTC tamper trigger */
    uint32_t rtc_tamper_filter;                                             /*!< RTC tamper consecutive samples needed during a voltage level detection */
    hal_rtc_tamper_freq_enum rtc_tamper_sample_frequency;                   /*!< RTC tamper sampling frequency during a voltage level detection */
    ControlStatus rtc_tamper_precharge_enable;                              /*!< RTC tamper precharge feature during a voltage level detection */
    uint32_t rtc_tamper_precharge_time;                                     /*!< RTC tamper precharge duration if precharge feature is enabled */
    ControlStatus rtc_tamper_with_timestamp;                                /*!< RTC tamper time-stamp feature */
}hal_rtc_tamper_struct; 

/* RTC IRQ structure */
typedef struct {
    hal_rtc_irq_handle_cb alarm_handle;                                     /*!< RTC alarm handler */
    hal_rtc_irq_handle_cb timestamp_handle;                                 /*!< RTC timestamp handler */
    hal_rtc_irq_handle_cb tamper0_handle;                                   /*!< RTC tamper handler */
    hal_rtc_irq_handle_cb tamper1_handle;                                   /*!< RTC tamper handler */
} hal_rtc_irq_struct;

/* hour format */
#define HAL_RTC_24HOUR                           RTC_24HOUR                 /*!< 24-hour format */
#define HAL_RTC_12HOUR                           RTC_12HOUR                 /*!< 12-hour format */

/* AM/PM mark */
#define HAL_RTC_AM                               RTC_AM                     /*!< AM format */
#define HAL_RTC_PM                               RTC_PM                     /*!< PM format */

/* winter/summer time change */
#define HAL_RTC_DAYLIGHTSAVING_1HOUR_ADD         RTC_CTL_A1H                /*!< add 1 hour(summer time change) */
#define HAL_RTC_DAYLIGHTSAVING_1HOUR_SUBTRACT    RTC_CTL_S1H                /*!< subtract 1 hour(winter time change) */
#define HAL_RTC_DAYLIGHTSAVING_NONE              ((uint32_t)0x00000000U)    /*!< no add or subtract 1 hour */

/* daylight saving mark */
#define HAL_RTC_RECORD_DAYLIGHTSAVING_SET        RTC_CTL_DSM                /*!< set daylight saving mark */
#define HAL_RTC_RECORD_DAYLIGHTSAVING_RESET      ((uint32_t)0x00000000U)    /*!< reset daylight saving mark */

/* alarm mark */
#define HAL_RTC_ALARM_NONE_MASK                  RTC_ALARM_NONE_MASK        /*!< alarm none mask */
#define HAL_RTC_ALARM_DATE_MASK                  RTC_ALARM_DATE_MASK        /*!< alarm date mask */
#define HAL_RTC_ALARM_HOUR_MASK                  RTC_ALARM_HOUR_MASK        /*!< alarm hour mask */
#define HAL_RTC_ALARM_MINUTE_MASK                RTC_ALARM_MINUTE_MASK      /*!< alarm minute mask */
#define HAL_RTC_ALARM_SECOND_MASK                RTC_ALARM_SECOND_MASK      /*!< alarm second mask */
#define HAL_RTC_ALARM_ALL_MASK                   RTC_ALARM_ALL_MASK         /*!< alarm all mask */

/* day of the week selected */
#define HAL_RTC_ALARM_DATE_SELECTED              RTC_ALARM_DATE_SELECTED    /*!< alarm date format selected */
#define HAL_RTC_ALARM_WEEKDAY_SELECTED           RTC_ALARM_WEEKDAY_SELECTED /*!< alarm weekday format selected */

/* alarm subsecond mark */
#define HAL_RTC_MASK_SUBSECOND                   RTC_MASKSSC_0_14           /*!< mask alarm subsecond configuration */
#define HAL_RTC_MASK_SUBSECOND_1_14              RTC_MASKSSC_1_14           /*!< mask RTC_ALRM0SS_SSC[14:1], and RTC_ALRM0SS_SSC[0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_2_14              RTC_MASKSSC_2_14           /*!< mask RTC_ALRM0SS_SSC[14:2], and RTC_ALRM0SS_SSC[1:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_3_14              RTC_MASKSSC_3_14           /*!< mask RTC_ALRM0SS_SSC[14:3], and RTC_ALRM0SS_SSC[2:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_4_14              RTC_MASKSSC_4_14           /*!< mask RTC_ALRM0SS_SSC[14:4], and RTC_ALRM0SS_SSC[3:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_5_14              RTC_MASKSSC_5_14           /*!< mask RTC_ALRM0SS_SSC[14:5], and RTC_ALRM0SS_SSC[4:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_6_14              RTC_MASKSSC_6_14           /*!< mask RTC_ALRM0SS_SSC[14:6], and RTC_ALRM0SS_SSC[5:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_7_14              RTC_MASKSSC_7_14           /*!< mask RTC_ALRM0SS_SSC[14:7], and RTC_ALRM0SS_SSC[6:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_8_14              RTC_MASKSSC_8_14           /*!< mask RTC_ALRM0SS_SSC[14:8], and RTC_ALRM0SS_SSC[7:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_9_14              RTC_MASKSSC_9_14           /*!< mask RTC_ALRM0SS_SSC[14:9], and RTC_ALRM0SS_SSC[8:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_10_14             RTC_MASKSSC_10_14          /*!< mask RTC_ALRM0SS_SSC[14:10], and RTC_ALRM0SS_SSC[9:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_11_14             RTC_MASKSSC_11_14          /*!< mask RTC_ALRM0SS_SSC[14:11], and RTC_ALRM0SS_SSC[10:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_12_14             RTC_MASKSSC_12_14          /*!< mask RTC_ALRM0SS_SSC[14:12], and RTC_ALRM0SS_SSC[11:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_13_14             RTC_MASKSSC_13_14          /*!< mask RTC_ALRM0SS_SSC[14:13], and RTC_ALRM0SS_SSC[12:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_14                RTC_MASKSSC_14             /*!< mask RTC_ALRM0SS_SSC[14], and RTC_ALRM0SS_SSC[13:0] is to be compared */
#define HAL_RTC_MASK_SUBSECOND_NONE              RTC_MASKSSC_NONE           /*!< mask none, and RTC_ALRM0SS_SSC[14:0] is to be compared */

/* timeout */
#define RTC_ALRM0F_TIMEOUT                       ((uint32_t)0xFFFFFFFFU)    /*!< alarm occur flag timeout */
#define RTC_TIMESTAMP_TIMEOUT                    ((uint32_t)0xFFFFFFFFU)    /*!< timestamp occur flag timeout */
#define RTC_TAMPER0_TIMEOUT                      ((uint32_t)0xFFFFFFFFU)    /*!< tamper 0 occur flag timeout */
#define RTC_TAMPER1_TIMEOUT                      ((uint32_t)0xFFFFFFFFU)    /*!< tamper 1 occur flag timeout */

/* rising/falling edge is valid event edge for time-stamp event */
#define HAL_RTC_TIMESTAMP_RISING_EDGE            RTC_TIMESTAMP_RISING_EDGE  /*!< rising edge is valid event edge for time-stamp event */
#define HAL_RTC_TIMESTAMP_FALLING_EDGE           RTC_TIMESTAMP_FALLING_EDGE /*!< falling edge is valid event edge for time-stamp event */

/* tamper source */
#if defined(GD32E230)
#define HAL_RTC_TAMPER0                          RTC_TAMPER0                /*!< tamper 0 detection enable */
#endif /* GD32E230 */
#define HAL_RTC_TAMPER1                          RTC_TAMPER1                /*!< tamper 1 detection enable */

/* tamper detection mode */
#define HAL_RTC_TAMPER_EDGE_RISING               RTC_TAMPER_TRIGGER_EDGE_RISING  /*!< tamper detection is in rising edge mode */
#define HAL_RTC_TAMPER_EDGE_FALLING              RTC_TAMPER_TRIGGER_EDGE_FALLING /*!< tamper detection is in falling edge mode */
#define HAL_RTC_TAMPER_LEVEL_LOW                 RTC_TAMPER_TRIGGER_LEVEL_LOW    /*!< tamper detection is in low level mode */
#define HAL_RTC_TAMPER_LEVEL_HIGH                RTC_TAMPER_TRIGGER_LEVEL_HIGH   /*!< tamper detection is in high level mode */

/* tamper sampling type and the number of consecutive sample */
#define HAL_RTC_FILTER_EDGE                      RTC_FLT_EDGE               /*!< detecting tamper event using edge mode. precharge duration is disabled automatically */
#define HAL_RTC_FILTER_2_SAMPLE                  RTC_FLT_2S                 /*!< detecting tamper event using level mode.2 consecutive valid level samples will make a effective tamper event  */
#define HAL_RTC_FILTER_4_SAMPLE                  RTC_FLT_4S                 /*!< detecting tamper event using level mode.4 consecutive valid level samples will make an effective tamper event */
#define HAL_RTC_FILTER_8_SAMPLE                  RTC_FLT_8S                 /*!< detecting tamper event using level mode.8 consecutive valid level samples will make a effective tamper event  */

/* the precharge time before each sampling */
#define HAL_RTC_1_CLK_PRECHARGE                  RTC_PRCH_1C                /*!< 1 RTC clock prechagre time before each sampling */
#define HAL_RTC_2_CLK_PRECHARGE                  RTC_PRCH_2C                /*!< 2 RTC clock prechagre time before each sampling  */
#define HAL_RTC_4_CLK_PRECHARGE                  RTC_PRCH_4C                /*!< 4 RTC clock prechagre time before each sampling */
#define HAL_RTC_8_CLK_PRECHARGE                  RTC_PRCH_8C                /*!< 8 RTC clock prechagre time before each sampling */

/* frequency compensation window */
#define HAL_RTC_CALIBRATION_32S                  RTC_CALIBRATION_WINDOW_32S /*!< 2exp20 RTCCLK cycles, 32s if RTCCLK = 32768 Hz */
#define HAL_RTC_CALIBRATION_16S                  RTC_CALIBRATION_WINDOW_16S /*!< 2exp19 RTCCLK cycles, 16s if RTCCLK = 32768 Hz */
#define HAL_RTC_CALIBRATION_8S                   RTC_CALIBRATION_WINDOW_8S  /*!< 2exp18 RTCCLK cycles, 8s if RTCCLK = 32768 Hz */

/* increase RTC frequency by 488.5PPM */
#define HAL_RTC_CALIBRATION_PLUS_SET             RTC_CALIBRATION_PLUS_SET   /*!< increase RTC frequency by 488.5ppm */
#define HAL_RTC_CALIBRATION_PLUS_RESET           RTC_CALIBRATION_PLUS_RESET /*!< no effect */

/* add 1 second to the clock */
#define HAL_RTC_ADD1S_RESET                      RTC_SHIFT_ADD1S_RESET      /*!< not add 1 second */
#define HAL_RTC_ADD1S_SET                        RTC_SHIFT_ADD1S_SET        /*!< add one second to the clock */

/* calibration/alarm output */
#define HAL_RTC_CALIBRATION_OUT_512HZ            RTC_CALIBRATION_512HZ      /*!< calibration output of 512Hz is enable */
#define HAL_RTC_CALIBRATION_OUT_1HZ              RTC_CALIBRATION_1HZ        /*!< calibration output of 1Hz is enable */
#define HAL_RTC_CALIBRATION_OUT_DISABLE          0U                         /*!< calibration output is disable */
#define HAL_RTC_ALARM_OUT_HIGH                   RTC_ALARM_HIGH             /*!< enable alarm flag output with high level */
#define HAL_RTC_ALARM_OUT_LOW                    RTC_ALARM_LOW              /*!< enable alarm flag output with low level*/
#define HAL_RTC_ALARM_OUT_DISABLE                1U                         /*!< disable alarm flag output*/

/* alarm output mode */
#define HAL_RTC_ALARM_OUTPUT_OD                  RTC_ALARM_OUTPUT_OD        /*!< RTC alarm output open-drain mode */
#define HAL_RTC_ALARM_OUTPUT_PP                  RTC_ALARM_OUTPUT_PP        /*!< RTC alarm output push-pull mode */

/* function declarations */
/* initialize the specified structure */
void hal_rtc_struct_init(hal_rtc_struct_type_enum hal_struct_type, void *p_struct);
/* deinitialize RTC */
int32_t hal_rtc_deinit(void);
/* initialize RTC */
int32_t hal_rtc_init(hal_rtc_init_struct* rtc_initpara_struct);
/* wait until RTC_TIME and RTC_DATE registers are synchronized with APB clock, and the shadow registers are updated */
int32_t hal_rtc_register_sync_wait(void);

/* get current time and date */
void hal_rtc_current_time_get(hal_rtc_init_struct* rtc_initpara_struct);

/* configure RTC alarm */
int32_t hal_rtc_alarm_config(hal_rtc_alarm_struct* rtc_alarm_time);
/* enable RTC alarm */
void hal_rtc_alarm_enable(void);
/* enable RTC alarm by interrupt method */
/* the function is non-blocking */
int32_t hal_rtc_alarm_enable_interrupt(hal_rtc_irq_handle_cb irq_handle);
/* disable RTC alarm */
int32_t hal_rtc_alarm_disable(void);
/* get RTC alarm */
void hal_rtc_alarm_get(hal_rtc_alarm_struct* rtc_alarm_time);
/* poll for RTC alarm event */
/* the function is blocking */
int32_t hal_rtc_alarm_event_poll(uint32_t timeout_ms);

/* configure RTC time-stamp */
int32_t hal_rtc_timestamp_config(uint32_t edge);
/* enable RTC time-stamp */
void hal_rtc_timestamp_enable(void);
/* enable RTC time-stamp by interrupt method */
/* the function is non-blocking */
int32_t hal_rtc_timestamp_enable_interrupt(hal_rtc_irq_handle_cb irq_handle);
/* disable RTC time-stamp */
void hal_rtc_timestamp_disable(void);
/* get RTC timestamp time and date */
void hal_rtc_timestamp_get(hal_rtc_timestamp_struct* rtc_timestamp);
/* configure RTC tamper */
int32_t hal_rtc_tamper_config(hal_rtc_tamper_struct* rtc_tamper);
/* enable RTC tamper */
int32_t hal_rtc_tamper_enable(uint32_t source);
/* enable RTC tamper by interrupt method */
/* the function is non-blocking */
int32_t hal_rtc_tamper_enable_interrupt(uint32_t source, hal_rtc_irq_handle_cb irq_handle);
/* disable RTC tamper */
int32_t hal_rtc_tamper_disable(uint32_t source);
/* poll for RTC timestamp event */
/* the function is blocking */
int32_t hal_rtc_timestamp_event_poll(uint32_t timeout_ms);
/* poll for RTC tamper0 event */
/* the function is blocking */
int32_t hal_rtc_tamper0_event_poll(uint32_t timeout_ms);
/* poll for RTC tamper1 event */
/* the function is blocking */
int32_t hal_rtc_tamper1_event_poll(uint32_t timeout_ms);

/* RTC interrupt handler content function */
void hal_rtc_irq(void);
/* set user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_rtc_irq_handle_set(hal_rtc_irq_struct *p_irq);
/* reset all user-defined interrupt callback function, 
which will be registered and called when corresponding interrupt be triggered */
void hal_rtc_irq_handle_all_reset(void);

/* write data to backup register */
int32_t hal_rtc_backup_data_write(uint32_t backup_index, uint32_t data);
/* read data from backup register */
int32_t hal_rtc_backup_data_read(uint32_t backup_index, uint32_t *data);
/* enable RTC bypass shadow registers function */
void hal_rtc_bypass_shadow_enable(void);
/* disable RTC bypass shadow registers function */
void hal_rtc_bypass_shadow_disable(void);
/* enable RTC reference clock detection function */
int32_t hal_rtc_refclock_detection_enable(void);
/* disable RTC reference clock detection function */
int32_t hal_rtc_refclock_detection_disable(void);

/* configure RTC alternate output source */
int32_t hal_rtc_output_config(uint32_t source, uint32_t mode);
/* configure RTC calibration register */
int32_t hal_rtc_calibration_config(uint32_t window, uint32_t plus, uint32_t minus);
/* ajust the daylight saving time by adding or substracting one hour from the current time */
void hal_rtc_daylight_saving_time_adjust(uint32_t operation, uint32_t record);
/* ajust RTC second or subsecond value of current time */
int32_t hal_rtc_sync_with_remote_clock_adjust(uint32_t add, uint32_t minus);

/* convert from  BCD format to binary format */
uint8_t rtc_bcd_2_normal(uint8_t data);
/* convert from  BCD format to binary format */
uint8_t rtc_normal_2_bcd(uint8_t data);

#endif /* GD32E23X_HAL_RTC_H */
