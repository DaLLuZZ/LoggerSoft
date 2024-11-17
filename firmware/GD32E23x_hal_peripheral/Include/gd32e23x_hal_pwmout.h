/*!
    \file    gd32e23x_hal_pwmout.h
    \brief   definitions for the PWM out
    
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

#ifndef GD32E23X_HAL_PWMOUT_H
#define GD32E23X_HAL_PWMOUT_H

#include "gd32e23x_hal.h"

/* function declarations */
/* initialize pwmout */
int32_t hal_timer_pwmout_init(hal_timer_dev_struct *timer_dev, uint32_t timer_periph , uint16_t channel);
/* free pwmout */
int32_t hal_timer_pwmout_free(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* pwm duty config */
int32_t hal_timer_pwmout_duty_write(hal_timer_dev_struct *timer_dev, uint16_t channel, float duty);
/* get pwm duty */
float hal_timer_pwmout_duty_read(hal_timer_dev_struct *timer_dev, uint16_t channel);
/* pwm period congfig, unit:s */
int32_t hal_timer_pwmout_period_s_write(hal_timer_dev_struct *timer_dev, float seconds);
/* pwm period congfig, unit:ms */
int32_t hal_timer_pwmout_period_ms_write(hal_timer_dev_struct *timer_dev, float ms);
/* pwm period congfig, unit:us */
int32_t hal_timer_pwmout_period_us_write(hal_timer_dev_struct *timer_dev, uint32_t us);
/* pwm pulse congfig, unit:s */
int32_t hal_timer_pwmout_pulse_s_write(hal_timer_dev_struct *timer_dev, uint16_t channel, float seconds);
/* pwm pulse congfig, unit:ms */
int32_t hal_timer_pwmout_pulse_ms_write(hal_timer_dev_struct *timer_dev, uint16_t channel, float ms);
/* pwm pulse congfig, unit:us */
int32_t hal_timer_pwmout_pulse_us_write(hal_timer_dev_struct *timer_dev, uint16_t channel, float us);

#endif /* GD32E23X_HAL_PWMOUT_H */
