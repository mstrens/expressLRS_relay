#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include <config_basic.h>
#include <crsf.h>
#include <sbus.h>
#include <sport.h>

//     CRSF uses pin 4 UART1 TX = Serial1
//     CRSF uses pin 5 UART1 RX
//     SBUS uses pin 1 UART0 RX = Serial  
//     SPORT (TX and RX) uses PIO and pin 8; for safety, insert a 1K resistor in serie on the wire to Frsky Sport

void setup() {
  // setup UART for Sbus (100 kbaud, 8E2)...) (inverted)
  setupSbus(); 
  // setup UART1 for CRSF at 400000 baud
  setupCRSF(); 
  // setup Sport uart (using pio, 2 sm, dma, irq, queue)
  setupSport();
}

void loop() {
//- use arduino serial.read  to read the Sbus; store the data in CRSF frame
//- at regular interval send the CRSF frame on Serial1
//- use arduino Serial1.read to read the CRSF telemetry; discard when it is not a telemetry frame;
//       store each receive data in a array (value and flag available)
//- Sport uart (pio) is normally in receive mode.
//     When a byte is received and fit the device ID, look for the next telemetry field to be sent
//     If any, make a sport frame, put the pio (sport UART) in transmit mode, put the telemetry frame on DMA a dma channel
//     after some timelaps, put the pio back in receive mode  

  handleSbusIn();
  sendCRSFRcFrame();
  handleTlmIn();
  handleSportRxTx();
}

