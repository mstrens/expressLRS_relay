//#include <Arduino.h>
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include <config_basic.h>
#include <crsf.h>
#include <sbus.h>
#include <sport.h>
#include "hardware/watchdog.h"
#include <tusb.h>
#include <inttypes.h>

//     CRSF uses pin 4 UART1 TX = Serial1
//     CRSF uses pin 5 UART1 RX
//     SBUS uses pin 1 UART0 RX = Serial  
//     SPORT (TX and RX) uses PIO and pin 8; for safety, insert a 1K resistor in serie on the wire to Frsky Sport

void setup() {
  stdio_init_all();
  uint16_t counter = 50; 
  if ( watchdog_caused_reboot() ) counter = 0; // avoid the UDC wait time when reboot is caused by the watchdog   
  while ( (!tud_cdc_connected()) && (counter--)) { sleep_ms(100);  }
  
  // setup UART for Sbus (100 kbaud, 8E2)...) (inverted)
  setupSbus(); 
  // setup UART1 for CRSF at 400000 baud
  setupCRSF(); 
  // setup Sport uart (using pio, 2 sm, dma, irq, queue)
  setupSport();
  watchdog_enable(500, 1); // require an update once every 500 msec
}

void loop() {
//- use hardware uart0 to read the Sbus; get the data from a queue and store the data in a sbus buffer
//- pio1 (2 state machines) is used for half duplex uart to communicate with CRSF
//     at regular interval convert the Sbus buffer into a CRSF frame , switch pio in sending mode and send using a DMA
//     When pio TXFIFO becomes empty, it call an IRQ that switch the UART in receiving mode.
//     An IRQ send each received character from ELRS to a queue
//     The queue is read in main loop and handle the process the char
//     Discard the data when it is not a valid telemetry frame;
//     When valid, store each receive data in a array (value and flag available)
//- Sport uart (pio0) is inverted half duplex and is normally in receive mode.
//     When a byte is received and fit the device ID, look for the next telemetry field to be sent
//     If any, make a sport frame, put the pio (sport UART) in transmit mode, put the telemetry frame on DMA a dma channel
//     after some timelaps, put the pio back in receive mode; this could be changed to use an interrupt like for CRSF.
  watchdog_update();
  handleSbusIn();
  sendCrsfRcFrame();
  handleTlmIn();
  handleSportRxTx();
}


int main(){
  setup();
  while(1) loop();
}