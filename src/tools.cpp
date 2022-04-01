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
