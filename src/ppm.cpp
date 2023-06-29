/*
 * Copyright (C) ExpressLRS_relay
 *
 *
 * License GPLv3: http://www.gnu.org/licenses/gpl-3.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "ppm.h"
#include "tools.h"
#include "config.h"
#include <inttypes.h>
#include "hardware/pio.h"
#include "ppm.pio.h"

PIO ppmPio = pio1;
uint ppmSm = 2 ;// to get the telemetry from crsf
uint ppmOffset ; 
#define PPM_PIO_PIN 0

extern uint32_t lastSbusReceived;
extern uint16_t sbusChannelsValue[16]; // contains the sbus 16 channel values

// remapping from sbus value to pwm value
uint16_t fromSbusMin = FROM_SBUS_MIN;
uint16_t toPwmMin = TO_PWM_MIN; 
uint16_t fromSbusMax = FROM_SBUS_MAX;
uint16_t toPwmMax = TO_PWM_MAX; 

void setupPpm(){
    // Set up the state machine for transmit and start it (but dma is not yet started) 
    ppmOffset = pio_add_program(ppmPio, &ppm_program);
    ppm_program_init(ppmPio, ppmSm, ppmOffset, PPM_PIO_PIN);  
}

#define PPM_OFF 0
#define PPM_ON 1
//#define PPM_PULSE_OFF 2

#define MAX_PPM_CHANNELS 12
#define PPM_PULSE_INTERVAL 50
#define PPM_SYNCHRO_INTERVAL 2000

uint16_t  fmap(uint16_t x)
{
    return (uint16_t)(((int)x - (int)FROM_SBUS_MIN) * (int)(TO_PWM_MAX - TO_PWM_MIN) * 2 / (int)(FROM_SBUS_MAX - FROM_SBUS_MIN) + (int) TO_PWM_MIN * 2 + 1) / 2;
}

void handlePpm(){
    static uint8_t nextPpmChannel = 0; // next channel to generate
    static uint8_t ppmState = PPM_OFF ; 
    static uint32_t nextPulseStartUs = 0; //
    uint32_t pulseWidthUs;  
    if ( (millis() - lastSbusReceived) > 1000) {
        ppmState = PPM_OFF ;
    }
    if ( ppmState == PPM_OFF && lastSbusReceived && ( (millis() - lastSbusReceived) < 1000) ) {
        ppmState = PPM_ON ;
        nextPpmChannel = 0;
        nextPulseStartUs = micros();
    }
    if ( ppmState == PPM_ON && micros() > nextPulseStartUs ){    
        // get the value of the channel and map it into usec
        pulseWidthUs = fmap( sbusChannelsValue[nextPpmChannel]) ; 
        // start the pulse with pio 
        pio_sm_put_blocking(ppmPio, ppmSm, pulseWidthUs);
        // calculate when next pulse must start
        nextPulseStartUs = micros() + pulseWidthUs + PPM_PULSE_INTERVAL;
        nextPpmChannel++; // prepare for next channel
        if (nextPpmChannel >= MAX_PPM_CHANNELS) {
            nextPpmChannel = 0;
            nextPulseStartUs += PPM_SYNCHRO_INTERVAL; 
        }
    }     
    
}

    