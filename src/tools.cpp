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

#include "tools.h"
#include "pico/stdlib.h"
#include "stdio.h"  // used by printf


uint32_t millis(){
    return  to_ms_since_boot( get_absolute_time());
}

uint32_t micros() {
    return  to_us_since_boot(get_absolute_time ());
}

void printHexBuffer(uint8_t * buffer , uint8_t bufferLength){
    for ( uint8_t i = 0 ; i < bufferLength ; i++){
        printf(" %x ", buffer[i]);
    }
    printf("\n");
}
