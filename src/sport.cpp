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
#include <stdio.h>

#include "pico/stdlib.h"
//#include "pico/multicore.h"
#include "hardware/pio.h"
//#include "hardware/uart.h"
#include "sport_uart_tx_rx.pio.h"
#include "pico/util/queue.h"
#include "hardware/dma.h"
#include "sport.h"

// to do: 
// if we can reply to several device id, we should keep a table with the last field index used for this deviceid
// when we get a polling for one device id, we should use this table to search from this last field index 


// one pio and 2 state machines are used to manage the sport in halfduplex
// one state machine (sm) handle the TX and the second the RX
// to receive data, the sm is initialised and use an IRQ handler when rx fifo is not empty
//    in irq, this byte is store in a Rx queue
//    This queue is read in main loop
//    When a byte is received after a 7E, then we stop the sm that receive (it means frsky made a polling)
//    We fill a buffer with the data
//    We set up a dma to transfer the data to the TX fifo of the Tx state machine
//    We also set up a timestamp to stop after some msec the Tx state machine and start again the Rx one   


#define SPORT_PIO_RX_PIN 5  // pin being used by the UART pio
#define SPORTSYNCREQUEST 0x7E
#define SPORTDEVICEID    0xE4

queue_t sportRxQueue ;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for Sport
PIO sportPio = pio0;
uint sportSmRx = 1; // to get the request from sport
uint sportSmTx = 0; // to send the telemetry to sport
uint sportOffsetTx ; 
uint sportOffsetRx ; 

// dma channel is used to send Sport telemetry without blocking
int sport_dma_chan;
dma_channel_config sportDmaConfig;

uint8_t sportTxBuffer[50];

uint32_t restoreSportPioToReceiveMillis = 0; // when 0, the pio is normally in receive mode,
                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode


field fields[SPORT_TYPES_MAX];  // list of all telemetry fields and parameters used by Sport


void setupSport() {
    // list of fileds being used
    //latitude , longitude , groundspeed , heading , altitude ,  numSat
    //mVolt , current , capacity ,  remain 
    //vSpeed, pitch , roll , yaw
    //uplink_RSSI_1 , uplink_RSSI_2 , uplink_Link_quality , uplink_SNR , active_antenna , rf_Mode ,
    //uplink_TX_Power , downlink_RSSI , downlink_Link_quality , downlink_SNR
    
    uint16_t listFieldsID[] = {GPS_LONG_LATI_FIRST_ID , GPS_LONG_LATI_FIRST_ID ,GPS_SPEED_FIRST_ID, GPS_COURS_FIRST_ID , ALT_FIRST_ID, T1_FIRST_ID  ,\
                          VFAS_FIRST_ID , CURR_FIRST_ID , T2_FIRST_ID , FUEL_FIRST_ID  ,\
                          VARIO_FIRST_ID, ACCX_FIRST_ID , ACCY_FIRST_ID , ACCZ_FIRST_ID ,\
                          UPLINK_RSSI_1_ID , UPLINK_RSSI_2_ID , UPLINK_LINK_QUALITY_ID , UPLINK_SNR_ID , ACTIVE_ANTENNA_ID, RF_MODE_ID ,\
                          UPLINK_TX_POWER_ID , DOWNLINK_RSSI_ID , DOWNLINK_LINK_QUALITY_ID , DOWNLINK_SNR_ID} ;
    uint8_t listdeviceID[] = {DATA_ID_GPS, DATA_ID_GPS, DATA_ID_GPS, DATA_ID_GPS, DATA_ID_VARIO , DATA_ID_RPM ,\
                            DATA_ID_FAS , DATA_ID_FAS , DATA_ID_RPM , DATA_ID_FAS ,\
                            DATA_ID_VARIO , DATA_ID_ACC , DATA_ID_ACC , DATA_ID_ACC ,\
                            DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM,\
                            DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM , DATA_ID_RPM};
    uint16_t listInterval[] = { 500, 500 , 500 , 500 , 500 ,500,\
                            500 , 500 , 500 , 500,\
                            300 , 300 , 300 , 300,\
                            500 , 500 , 500 , 500 , 500, 500,\
                            500 , 500 , 500 , 500};
    for (uint8_t i = 0 ;  i<sizeof(listdeviceID); i++){
        fields[i].value= 0;
        fields[i].available= false;
        fields[i].nextMillis= 0;
        fields[i].interval= listInterval[i];
        fields[i].deviceId= listdeviceID[i];
        fields[i].fieldId= listFieldsID[i];
    }                                                
// configure the queue to get the data from Sport in the irq handle
    queue_init (&sportRxQueue, sizeof(uint8_t), 250);

// set up the DMA but do not yet start it to send data to Sport
// Configure a channel to write the same byte (8 bits) repeatedly to PIO0
// SM0's TX FIFO, placed by the data request signal from that peripheral.
    sport_dma_chan = dma_claim_unused_channel(true);
    sportDmaConfig = dma_channel_get_default_config(sport_dma_chan);
    channel_config_set_read_increment(&sportDmaConfig, true);
    channel_config_set_write_increment(&sportDmaConfig, false);
    channel_config_set_dreq(&sportDmaConfig, DREQ_PIO0_TX0);  // use state machine 0 
    channel_config_set_transfer_data_size(&sportDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        sport_dma_chan,
        &sportDmaConfig,
        &pio0_hw->txf[0], // Write address (only need to set this once)
        &sportTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit but do not yet start it (it starts only when a request from receiver is received)
    sportOffsetTx = pio_add_program(sportPio, &sport_uart_tx_program);
    sport_uart_tx_program_init(sportPio, sportSmTx, sportOffsetTx, SPORT_PIO_RX_PIN, 57600 , true); // we use the same pin and baud rate for tx and rx, true means thet UART is inverted 

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( PIO0_IRQ_0 , sportPioRxHandlerIrq) ;
    irq_set_enabled (PIO0_IRQ_0 , true) ;

// Set up the state machine we're going to use to receive them.
    sportOffsetRx = pio_add_program(sportPio, &sport_uart_rx_program);
    sport_uart_rx_program_init(sportPio, sportSmRx, sportOffsetRx, SPORT_PIO_RX_PIN, 57600 , true);  
}


void sportPioRxHandlerIrq(){    // when a byte is received on the Sport, read the pio Sport fifo and push the data to a queue (to be processed in the main loop)
  // clear the irq flag
  irq_clear (PIO0_IRQ_0 );
  while (  ! pio_sm_is_rx_fifo_empty (sportPio ,sportSmRx)){ // when some data have been received
     uint8_t c = pio_sm_get (sportPio , sportSmRx) >> 24;         // read the data
     queue_try_add (&sportRxQueue, &c);          // push to the queue
    //sportRxMillis = millis();                    // save the timestamp.
  }
}


//void processNextSportRxByte( uint8_t c){
//  static uint8_t previous = 0;
//  printf(" %X",c);
//  if ( ( previous ==  SPORTSYNCREQUEST)  && (c == SPORTDEVICEID) ) sendNextSportFrame();
//  previous = c;
//} 

void handleSportRxTx(void){   // main loop : restore receiving mode , wait for tlm request, prepare frame, start pio and dma to transmit it
    static uint8_t previous = 0;
    uint8_t data;
    if ( restoreSportPioToReceiveMillis) {            // put sm back in recive mode after some delay
        if (millis() > restoreSportPioToReceiveMillis){
            sport_uart_tx_program_stop(sportPio, sportSmTx, SPORT_PIO_RX_PIN );
            sport_uart_rx_program_restart(sportPio, sportSmRx, SPORT_PIO_RX_PIN, true);  // true = inverted
            restoreSportPioToReceiveMillis = 0 ;
        }
    } else {                             // when we are in receive mode
        if (! queue_is_empty(&sportRxQueue)) {
            queue_try_remove (&sportRxQueue,&data);
            if ( ( previous ==  SPORTSYNCREQUEST)  &&  (data == SPORTDEVICEID)   ) sendNextSportFrame(data);
            // we could use several device id ti increase the telemetry rate
            //if ( ( previous ==  SPORTSYNCREQUEST)  && ( (data == DATA_ID_GPS) || 
            //(data == DATA_ID_VARIO) || (data == DATA_ID_RPM) || (data == DATA_ID_FAS) || (data == DATA_ID_ACC) ) ) sendNextSportFrame(data);
            previous = data; 
            }
    }           
}


void sendNextSportFrame(uint8_t data_id){ // search for the next data to be sent for this device ID
    static uint8_t last_sport_idx = 0 ;
    if ( dma_channel_is_busy(sport_dma_chan) )return ; // skip if the DMA is still sending data
    uint32_t _millis = millis();
    for (uint8_t i = 0 ; i< SPORT_TYPES_MAX ; i++ ){
         last_sport_idx++;
         if (last_sport_idx >= SPORT_TYPES_MAX) last_sport_idx = 0 ;
         if ( (_millis >= fields[last_sport_idx].nextMillis) && (fields[last_sport_idx].available)  ) {
             sendOneSport(last_sport_idx);
             fields[last_sport_idx].available = false; // flag as sent
             fields[last_sport_idx].nextMillis = millis() + fields[last_sport_idx].interval;
             break;
         }
    }
}

void sendOneSport(uint8_t idx){  // fill one frame and send it
    // the frame contains 1 byte = type, 2 bytes = value id, 4 byte( or more) for value, 1 byte CRC
    uint8_t counter = 0;
    uint8_t value[4] = { 0, 1, 2, 3};
    uint16_t crc ;
    uint8_t tempBuffer[10];
    tempBuffer[counter++] = 0X10 ; // type of packet : data
    tempBuffer[counter++] = fields[idx].deviceId    ; // 0x0110 = Id for vario data
    tempBuffer[counter++] = fields[idx].deviceId >> 8 ; // 0x0110 = Id for vario data
    tempBuffer[counter++] = fields[idx].value >> 24; // value
    tempBuffer[counter++] = fields[idx].value >> 16 ;  
    tempBuffer[counter++] = fields[idx].value >> 8 ; // 
    tempBuffer[counter++] = fields[idx].value >> 0 ; //
    crc = tempBuffer[0] ;
    for (uint8_t i = 1; i<=6;i++){
      crc +=  tempBuffer[i]; //0-1FF
      crc += crc >> 8 ; //0-100
      crc &= 0x00ff ;
    }
    tempBuffer[counter] = 0xFF-crc ;  // CRC in buffer
    // copy and convert bytes
    // Byte in frame has value 0x7E is changed into 2 bytes: 0x7D 0x5E
    // Byte in frame has value 0x7D is changed into 2 bytes: 0x7D 0x5D
    uint8_t sportLength = 0;
    for (uint8_t i = 0 ; i<=7 ; i++){
      if (tempBuffer[i] == 0x7E) {
        sportTxBuffer[sportLength++] = 0x7D;
        sportTxBuffer[sportLength++] = 0x5E;
      } else if (tempBuffer[i] == 0x7D) {
        sportTxBuffer[sportLength++] = 0x7D;
        sportTxBuffer[sportLength++] = 0x5D;
      } else {
      sportTxBuffer[sportLength++]= tempBuffer[i];
      }
    }
    sport_uart_rx_program_stop(sportPio, sportSmRx, SPORT_PIO_RX_PIN); // stop receiving
    sport_uart_tx_program_start(sportPio, sportSmTx, SPORT_PIO_RX_PIN, true); // prepare to transmit
    // start the DMA channel with the data to transmit
    dma_channel_set_read_addr (sport_dma_chan, &sportTxBuffer[0], false);
    dma_channel_set_trans_count (sport_dma_chan, sportLength, true) ;
    // we need a way to set the pio back in receive mode when all bytes are sent 
    // this will be done in the main loop after some ms (here 6ms)
    restoreSportPioToReceiveMillis = millis() + 6;   
}

