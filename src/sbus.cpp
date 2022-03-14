#include "arduino.h"

#include <sbus.h>

enum SBUS_STATE{
  NO_SBUS_FRAME = 0,
  RECEIVING_SBUS 
};

uint8_t sbusFrame[24];
extern uint8_t CRSFRcFrame[30];
bool CRSFRcFrameReady = false;

void setupSbus(){
  Serial.begin(100000, 8, 2 , UART_PARITY_ODD);
  // invert the signal
  gpio_set_inover(1, GPIO_OVERRIDE_INVERT);
}


void handleSbusIn(){
  static SBUS_STATE sbusState = NO_SBUS_FRAME ;
  static uint8_t sbusCounter = 0;
  static uint32_t lastSbusMillis = 0;
  uint8_t c;
  while (Serial.available() ){
    if ( (millis() - lastSbusMillis ) > 2 ){
      sbusState = NO_SBUS_FRAME ;
    }
    lastSbusMillis = millis();
    c= Serial.read();
    //printf(" %X",c);
    switch (sbusState) {
      case NO_SBUS_FRAME :
        if (c == 0x0F) {
          sbusCounter = 1;
          sbusState = RECEIVING_SBUS ;
        }
      break;
      case RECEIVING_SBUS :
        sbusFrame[sbusCounter++] = c;
        if (sbusCounter == 24 ) {
          if ( (c != 0x00) && (c != 0x04) && (c != 0x14) && (c != 0x24) && (c != 0x34) ) {
            sbusState = NO_SBUS_FRAME;
          } else {
            storeSbusFrame();
            sbusState = NO_SBUS_FRAME;
          }
        }
      break;      
    } 
  }
}


void storeSbusFrame(){
    memcpy(&CRSFRcFrame[3], &sbusFrame[1], 22);
    CRSFRcFrameReady = true;
}  
