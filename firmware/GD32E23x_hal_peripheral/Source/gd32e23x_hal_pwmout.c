/*!
    \file    gd32e23x_hal_pwmout.c
    \brief   PWMOUT driver
    
    \version 2019-03-14, V1.0.0, firmware for GD32E10x
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

/* adjust period and prescaler */
static uint16_t num_temp = 0u;

/* get TIMER clock */
static uint32_t timer_get_clock(uint32_t timer_periph);

/*!
    \brief      initialize pwmout
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  timer_periph: specify which TIMER timebase is initialized 
      \arg        TIMERx(x=0,2,5,13..16)
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_init(hal_timer_dev_struct *timer_dev, uint32_t timer_periph , uint16_t channel)
{
    uint32_t timerclk;
    hal_timer_basic_struct timer0_basic_parameter;
    hal_timer_clocksource_struct timer0_clocksource_parameter;
    hal_timer_outputcompare_struct timer0_outputcompare_parameter;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    switch (timer_periph) {
        case TIMER0:
            rcu_periph_clock_enable(RCU_TIMER0);
            break;
        
        case TIMER2:
            rcu_periph_clock_enable(RCU_TIMER2);
            break;
        
        case TIMER5:
            rcu_periph_clock_enable(RCU_TIMER5);
            break;
        
        case TIMER13:
            rcu_periph_clock_enable(RCU_TIMER13);
            break;
        
        case TIMER14:
            rcu_periph_clock_enable(RCU_TIMER14);
            break;
        
        case TIMER15:
            rcu_periph_clock_enable(RCU_TIMER15);
            break;
        case TIMER16:
            rcu_periph_clock_enable(RCU_TIMER16);
            break;
        default:
            HAL_DEBUGE("parameter [timer_periph] value is invalid");
            return HAL_ERR_NO_SUPPORT;
    } 
    timer_dev->periph = timer_periph;
    hal_timer_stop_pwm(timer_dev, channel);
    hal_timer_stop_pwm_negtive(timer_dev, channel);
    
    timerclk = timer_get_clock(timer_periph);
    
    
    hal_timer_struct_init(HAL_TIMER_BASIC_STRUCT, &timer0_basic_parameter);
    hal_timer_struct_init(HAL_TIMER_CLOCKSOURCE_STRUCT, &timer0_clocksource_parameter);
    hal_timer_struct_init(HAL_TIMER_OUTPUTCOMPARE_STRUCT, &timer0_outputcompare_parameter);
    
    /* initialize TIMER timebase  */
    timer0_basic_parameter.prescaler = (uint32_t)(timerclk/1000000) - 1;
    timer0_basic_parameter.alignedmode = TIMER_COUNTER_EDGE;
    timer0_basic_parameter.counterdirection = TIMER_COUNTER_UP;
    timer0_basic_parameter.period = 9999;
    timer0_basic_parameter.clockdivision = TIMER_CKDIV_DIV1;
    timer0_basic_parameter.repetitioncounter = 0;
    timer0_basic_parameter.autoreload_shadow = AUTO_RELOAD_SHADOW_DISABLE;
    timer0_basic_parameter.trgo_selection = TIMRE_TRGO_SRC_RESET;
    timer0_basic_parameter.master_slave_mode = TIMER_MASTER_SLAVE_MODE_DISABLE;
    hal_timer_basic_init(timer_dev, timer_periph, &timer0_basic_parameter);
    
    /* timer clock source config */
    timer0_clocksource_parameter.clock_source = TIMER_CLOCK_SOURCE_CK_TIMER;
    hal_timer_clock_source_config(timer_dev, &timer0_clocksource_parameter);
    
    /* initialize TIMER pwm mode */
    timer0_outputcompare_parameter.output_mode = TIMER_OUTPUT_PWM1_MODE;
    timer0_outputcompare_parameter.output_pulse = 4999;
    timer0_outputcompare_parameter.output_fastmode = TIMER_OC_FAST_DISABLE;
    timer0_outputcompare_parameter.output_polarity = TIMER_OC_POLARITY_HIGH;
    timer0_outputcompare_parameter.output_idlestate = TIMER_OC_IDLE_STATE_LOW;
    timer0_outputcompare_parameter.outputn_polarity = TIMER_OCN_POLARITY_HIGH;
    timer0_outputcompare_parameter.outputn_idlestate = TIMER_OCN_IDLE_STATE_LOW;
    hal_timer_pwm_init(timer_dev, channel, &timer0_outputcompare_parameter);
    
    /* start TIMER pwm mode */
    hal_timer_start_pwm(timer_dev, channel);
    hal_timer_start_pwm_negtive(timer_dev, channel);
    return HAL_ERR_NONE;
}

/*!
    \brief      free pwmout
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, HAL_ERR_NO_SUPPORT, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_free(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    int32_t ret_val;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    /* stop TIMER pwm mode */
    ret_val = hal_timer_stop_pwm(timer_dev, channel);
    return ret_val;
}

/*!
    \brief      pwm duty config
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  duty: 0~1
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_duty_write(hal_timer_dev_struct *timer_dev, uint16_t channel, float duty)
{
    uint16_t period;
    uint16_t pulse;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* stop TIMER pwm mode */
    hal_timer_stop_pwm(timer_dev, channel);
    
    /* overflow protection */
    if (duty < (float)0.0) {
        duty = 0.0;
    } else if (duty > (float)1.0) {
        duty = 1.0;
    }
    
    /* ger period */
    period = TIMER_CAR(timer_dev->periph);
    /* get pulse */
    pulse = (uint16_t)(period * duty);
    
    /* write channely capture/compare value, TIMERx_CHyCV */
    hal_timer_write_channelx_value(timer_dev, channel, pulse);

    /* start TIMER pwm mode */
    hal_timer_start_pwm(timer_dev, channel);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      get pwm duty 
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[out] none
    \retval     duty value:0~1
*/
float hal_timer_pwmout_duty_read(hal_timer_dev_struct *timer_dev, uint16_t channel)
{
    float duty = 0;
    uint16_t period;
    uint16_t pulse;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* ger period */
    period = TIMER_CAR(timer_dev->periph);
    
    /* read channely capture/compare value, TIMERx_CHyCV */
    pulse = hal_timer_read_channelx_value(timer_dev, channel);
    
    /* calculated waveform duty ratio */
    duty = (float)(pulse) / (float)(period);

    if(duty > (float)1.0) {
        duty = (float)1.0;
    }
    
    return duty;
}

/*!
    \brief      config pwm period, uint: s
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  seconds: 0x0~max_svalue, max_svalue is depend on TIMER configuration,0~4294
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_period_s_write(hal_timer_dev_struct *timer_dev, float seconds)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    hal_timer_pwmout_period_us_write(timer_dev, (uint32_t)(seconds * 1000000));
    return HAL_ERR_NONE;
}

/*!
    \brief      config pwm period, uint: ms
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  ms: 0x0~max_msvalue, max_msvalue is depend on TIMER configuration,0~4294967
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_period_ms_write(hal_timer_dev_struct *timer_dev, float ms)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    hal_timer_pwmout_period_us_write(timer_dev, (uint32_t)(ms * 1000));
    
    return HAL_ERR_NONE;
}


/*!
    \brief      config pwm period, uint:us
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  us: 0x0~max_usvalue, max_usvalue is depend on TIMER configuration, 0~4294967295
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_period_us_write(hal_timer_dev_struct *timer_dev, uint32_t us)
{
    uint32_t ultemp = 0u;
    uint32_t timer_clk = 0u;
    uint32_t period = us - 1u;
    uint32_t prescaler = 0u;
    float duty_ratio_ch0;
    float duty_ratio_ch1;
    float duty_ratio_ch2;
    float duty_ratio_ch3;

#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* get duty */
    duty_ratio_ch0 = hal_timer_pwmout_duty_read(timer_dev, TIMER_CH_0);
    duty_ratio_ch1 = hal_timer_pwmout_duty_read(timer_dev, TIMER_CH_1);
    duty_ratio_ch2 = hal_timer_pwmout_duty_read(timer_dev, TIMER_CH_2);
    duty_ratio_ch3 = hal_timer_pwmout_duty_read(timer_dev, TIMER_CH_3);
    
    /* TIMER counter disable */
    hal_timer_stop_counter(timer_dev);
    
    /* get TIMER clock */
    timer_clk = timer_get_clock(timer_dev->periph);
    
    /* calculate period and prescaler */
    ultemp = timer_clk/1000000u;
    prescaler = ultemp;
    num_temp = 0x1u;
    while(period > 0xFFFF){
        num_temp = num_temp << 1u;
        period = period >> 1u;
        prescaler = num_temp * ultemp;
    }
    if(prescaler > 0xFFFF){
        HAL_DEBUGE("Error: TIMER prescaler value is overflow \r\n");
        return HAL_ERR_NO_SUPPORT;
    }
    
    hal_timer_write_autoreload_value(timer_dev, period);
    hal_timer_write_prescaler_value(timer_dev, prescaler);
    
    /* config channel 0 */
    ultemp = (uint32_t)(duty_ratio_ch0*us);
    hal_timer_pwmout_pulse_us_write(timer_dev, TIMER_CH_0, ultemp);
    
    /* config channel 1 */
    ultemp = (uint32_t)(duty_ratio_ch1*us);
    hal_timer_pwmout_pulse_us_write(timer_dev, TIMER_CH_1, ultemp);
    
    /* config channel 2 */
    ultemp = (uint32_t)(duty_ratio_ch2*us);
    hal_timer_pwmout_pulse_us_write(timer_dev, TIMER_CH_2, ultemp);
    
    /* config channel 3 */
    ultemp = (uint32_t)(duty_ratio_ch3*us);
    hal_timer_pwmout_pulse_us_write(timer_dev, TIMER_CH_3, ultemp);
    
    /* TIMER counter enable */
    hal_timer_start_counter(timer_dev);
    
    return HAL_ERR_NONE;
    
}

/*!
    \brief      config pwm pulse width, uint: s
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  us: 0x0~max_svalue,max_svalue is depend on TIMER configuration,0~4294
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_pulse_s_write(hal_timer_dev_struct *timer_dev, uint16_t channel, float seconds)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    hal_timer_pwmout_pulse_us_write(timer_dev, channel, seconds*1000000);
    return HAL_ERR_NONE;
}

/*!
    \brief      config pwm pulse width, uint: ms
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  us: 0x0~max_msvalue,max_msvalue is depend on TIMER configuration,0~4294967
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_pulse_ms_write(hal_timer_dev_struct *timer_dev, uint16_t channel, float ms)
{
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    hal_timer_pwmout_pulse_us_write(timer_dev, channel, ms*1000);
    return HAL_ERR_NONE;
}

/*!
    \brief      config pwm pulse width, uint: us
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[in]  channel:
                only one parameter can be selected which is shown as below:
      \arg        TIMER_CH_0: TIMER channel0(TIMERx(x=0,2,13..16))
      \arg        TIMER_CH_1: TIMER channel1(TIMERx(x=0,2,14))
      \arg        TIMER_CH_2: TIMER channel2(TIMERx(x=0,2))
      \arg        TIMER_CH_3: TIMER channel3(TIMERx(x=0,2))
    \param[in]  us: 0x0~max_value,max_value is depend on TIMER configuration, 0~4294967295
    \param[out] none
    \retval     error code: HAL_ERR_NONE, HAL_ERR_ADDRESS, details refer to gd32e23x_hal.h
*/
int32_t hal_timer_pwmout_pulse_us_write(hal_timer_dev_struct *timer_dev, uint16_t channel, float us)
{
    uint32_t pulse;
    uint32_t period;
#if (1 == HAL_PARAMETER_CHECK)
    /* check the parameters */
    if(NULL == timer_dev){
        HAL_DEBUGE("pointer [timer_dev] address is invalid");
        return HAL_ERR_ADDRESS;
    }
#endif /* 1 == HAL_PARAMETER_CHECK */

    
    /* read counter auto reload value, TIMERx_CAR */
    period = hal_timer_read_autoreload_value(timer_dev);
    pulse = (uint32_t)(us / num_temp);
    if(pulse > period) {
        pulse = period;
    }
    
    /* write channely capture/compare value, TIMERx_CHyCV */
    hal_timer_write_channelx_value(timer_dev, channel, pulse);
    
    return HAL_ERR_NONE;
}

/*!
    \brief      get TIMER clock
    \param[in]  timer_dev: TIMER device information structrue
                  the structure is not necessary to be reconfigured after structrue initialization,
                  the structure parameters altering is automatically configured by core
    \param[out] none
    \retval     TIMER clock
*/
static uint32_t timer_get_clock(uint32_t timer_periph)
{
    uint32_t timerclk;
    
    if((TIMER0 == timer_periph) || (TIMER14 == timer_periph) || 
    (TIMER15 == timer_periph) || (TIMER16 == timer_periph)){
        /* get APB2`s TIMER clock */
        timerclk = hal_rcu_periph_clkfreq_get(RCU_PERIPH_CLKTYPE_APB2TIMER);
    }else{
        /* get APB1`s TIMER clock */
        timerclk = hal_rcu_periph_clkfreq_get(RCU_PERIPH_CLKTYPE_APB1TIMER);
    }
    
    return timerclk;
}
