#pragma once

#include "pico/stdlib.h"
//#include "stdio.h"


uint32_t millis() ;

uint32_t micros();

void printHexBuffer(uint8_t * buffer , uint8_t bufferLength);
