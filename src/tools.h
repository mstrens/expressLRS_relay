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

#pragma once

#include "pico/stdlib.h"
//#include "stdio.h"


uint32_t millis() ;

uint32_t micros();

void printHexBuffer(uint8_t * buffer , uint8_t bufferLength);


//! Byte swap unsigned short
uint16_t swap_uint16( uint16_t val ); 


//! Byte swap short
int16_t swap_int16( int16_t val ) ;

//! Byte swap unsigned int
uint32_t swap_uint32( uint32_t val );

//! Byte swap int
int32_t swap_int32( int32_t val );
