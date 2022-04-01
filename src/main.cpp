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

//     CRSF (Tx and RX) uses PIO1 and pin 7 ; for safety, insert a 1K resistor in serie on the wire to Frsky Sport
//     SBUS uses pin 1 UART0 RX = Serial  
//     SPORT (TX and RX) uses PIO0 and pin 8; for safety, insert a 1K resistor in serie on the wire to Frsky Sport
//  the pinout of the ELRS Tx module is the folowing:
//      - ???? not used
//      - ???? not used 
//      - battery VCC (5V-10V)
//      - gnd
//      - CRSF signal (is usually used by frsky for sport signal)

void setup() {
  stdio_init_all();
  uint16_t counter = 100; 
  //if ( watchdog_caused_reboot() ) counter = 0; // avoid the UDC wait time when reboot is caused by the watchdog   
  while ( (!tud_cdc_connected()) && (counter--)) { sleep_ms(100);  }
  
  printf("started\n");
  // setup UART for Sbus (100 kbaud, 8E2)...) (inverted)
  setupSbus(); 

  // setup crsf uart for CRSF at 400000 baud; use pio1, 2 sm (one for tx and one for rx), dma for sending , irq and queue for reading)
  // the relay is master of crsf uart; it sent rc frame once every 4ms and between 2 frames, it read the telemetry on the same pin
  setupCRSF();

  // setup Sport uart (using pio0, 2 sm, dma, irq, queue)
  // FRSKY is master; it sent a code for polling the different devices once pet 11 msec; only one device can reply 
  setupSport();
  watchdog_enable(500, 1); // require an update once every 500 msec; othsewise, it forces a reboot
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
