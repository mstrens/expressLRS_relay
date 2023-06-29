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
#include "hardware/pio.h"
#include "hardware/dma.h"
//#include "hardware/irq.h"
//#include "uart_tx.pio.h"
#include "pico/util/queue.h"
#include "crsf_uart_tx_rx.pio.h"
#include "crsf.h"
#include "config.h"
#include "crc.h"
#include "sport.h"
#include "stdio.h"  // used by printf
#include "tools.h"

//#define DEBUG_WITH_FIXED_RC_FRAME
#define DEBUG_RECEIVED_FRAME // print the content of valid received frames



#define CRSF_PIO_PIN_TX 7  // pin being used by the UART pio for ELRS
#define CRSF_PIO_PIN_RX 8  // pin being used by the UART pio for ELRS

// When we look at the data exchanged with openTX with a logic analyser, it seems that:
// openTX send a frame with Rc channel once every 4msec (24 bytes at 400000 bauds = 24*25micro sec)
// the eLRS module sent some frame back about 40/45 usec later with  an adress = EA
// When the RX has no telemetry, there are at least 2 types of frame to openTx
// one has a length = 0X0D and a type = 3A; it is sent every 0.2msec
// the other has a length = 0X0C and a type = 0C (= link statistic); it is sent once every 0.32 sec
// I expect that there are some others type (gps, vario, ...)
// Still at least when ELRS TX was configured with 500 frames/sec, the connection occurs only when the frames where generates every 2msec instead of 4msec
// Probably this is related to the frame rate selected in ELRS TX  

queue_t crsfRxQueue ; // queue to get the telemetry data from crsf (from an irq handler)


//// This is the same as the default UART baud rate on Pico
const uint32_t CRSF_SERIAL_BAUD = 400000;

// one pio with 2 state machine is used to manage the inverted hal duplex uart for crsf
PIO crsfPio = pio1;
uint crsfSmRx = 1; // to get the telemetry from crsf
uint crsfSmTx = 0; // to send the channels to crsf
uint crsfOffsetTx ; 
uint crsfOffsetRx ; 

// dma channel is used to send Sport telemetry without blocking
int crsf_dma_chan;
dma_channel_config crsfDmaConfig;

uint8_t crsfTxBuffer[50]; // buffer to be used by dma to send the RC channels
uint8_t crsfTxBufferLength;


uint32_t restoreCrsfPioToReceiveMillis = 0; // when 0, the pio is normally in receive mode,
                                        // otherwise, it is the timestamp when pio transmit has to be restore to receive mode

//#define CRSF_RC_FRAME_INTERVAL 4 // msec replaced by an interval in micro sec
#define CRSF_RC_FRAME_INTERVAL_MICROS 4000  // microsec
#define CRSF_IRQ_TXFIFO_EMPTY PIO1_IRQ_1  // irq when txfifo is empty
#define CRSF_IRQ_RX_RECEIVED PIO1_IRQ_0   // irq when a byte is received

uint32_t crsfRcFrameIntervalMicros = CRSF_RC_FRAME_INTERVAL_MICROS ;

volatile CRSF_MODE crsfMode = RECEIVING;

crsf_tlm_state tlmState;  // store the state of handling receiving tlm bytes 

#define FRAME_TYPES_MAX 5
uint32_t crsfFrameNextMillis[FRAME_TYPES_MAX] = {0} ; 
uint8_t crsf_last_frame_idx = 0 ;  

voltageFrameStruct voltageFrame;
varioFrameStruct varioFrame;
attitudeFrameStruct attitudeFrame;
gpsFrameStruct gpsFrame;

//uint8_t CRSFTlmFrame[50];
//uint8_t CRSFTlmFrameLength;

rcFrameStruct crsfRcFrame ;
bool crsfRcFrameReady = false;

bool oneFrameFromElrsReceived = false; // become true when a valid frame is received from ELRS

GENERIC_CRC8 crsf_crc(CRSF_CRC_POLY);

#define TEST_PIN 3 

void setupCRSF(){
    //Serial1.begin(SERIAL_BAUD_CRSF);
    // configure the queue to get the telemetry data from crsf in the irq handle
    queue_init (&crsfRxQueue, sizeof(uint8_t), 250);

// set up the DMA but do not yet start it to send rc data to crsf
// Configure a channel to write the same byte (8 bits) repeatedly to PIO1
// SM0's TX FIFO, paced by the data request signal from that peripheral.
    crsf_dma_chan = dma_claim_unused_channel(true);
    crsfDmaConfig = dma_channel_get_default_config(crsf_dma_chan);
    channel_config_set_read_increment(&crsfDmaConfig, true);
    channel_config_set_write_increment(&crsfDmaConfig, false);
    channel_config_set_dreq(&crsfDmaConfig, DREQ_PIO1_TX0);  // use state machine 0 on PIO1 
    channel_config_set_transfer_data_size(&crsfDmaConfig, DMA_SIZE_8);
    dma_channel_configure(
        crsf_dma_chan,
        &crsfDmaConfig,
        &pio1_hw->txf[0], // Write address (only need to set this once)
        &crsfTxBuffer[0],   // we use always the same buffer             
        0 , // do not yet provide the number of bytes (DMA cycles)
        false             // Don't start yet
    );
// Set up the state machine for transmit and start it (but dma is not yet started) 
    crsfOffsetTx = pio_add_program(crsfPio, &crsf_uart_tx_program);
    crsf_uart_tx_program_init(crsfPio, crsfSmTx, crsfOffsetTx, CRSF_PIO_PIN_TX, CRSF_SERIAL_BAUD , true); // we use the same pin and baud rate for tx and rx, true means that UART is inverted 

// Set up an irq on pio to handle when Tx fifo is empty. We use PIO1_IRQ_1 for this
    //irq_set_exclusive_handler( CRSF_IRQ_TXFIFO_EMPTY , crsfPioTxEmptyHandlerIrq) ;
    //irq_set_enabled (CRSF_IRQ_TXFIFO_EMPTY , false) ;

// set an irq on pio to handle a received byte
    irq_set_exclusive_handler( CRSF_IRQ_RX_RECEIVED , crsfPioRxHandlerIrq) ;
    irq_set_enabled (CRSF_IRQ_RX_RECEIVED , false) ;

// Set up the state machine we're going to use to receive them. the sm is started
    crsfOffsetRx = pio_add_program(crsfPio, &crsf_uart_rx_program);
    crsf_uart_rx_program_init(crsfPio, crsfSmRx, crsfOffsetRx, CRSF_PIO_PIN_RX, CRSF_SERIAL_BAUD , true);

    irq_set_enabled (CRSF_IRQ_RX_RECEIVED , true) ; // enable the IRQ to handle the received charaters

}

void crsfPioRxHandlerIrq(){    // when a byte is received on the CRSF, read the pio CRSF fifo and push the data to a queue (to be processed in the main loop)
  irq_clear (CRSF_IRQ_RX_RECEIVED ); // clear the irq flag
  while (  ! pio_sm_is_rx_fifo_empty (crsfPio ,crsfSmRx)){ // when some data have been received 
    uint8_t c = pio_sm_get (crsfPio , crsfSmRx) >> 24;         // read the data
     queue_try_add (&crsfRxQueue, &c);          // push to the queue

    //sportRxMillis = millis();                    // save the timestamp.
  }
}

//  when crsf state machine fires the IRQ because TXFIFO is empty 
//  - clear the IRQ flag for TXFIFO and disabled it
//  - clear the IRQ flag for receiving
//  _ clear the Rx queue
//  - enable the receiving IRQ
//  - switch PIO receiving mode
//  - set state to receiving mode
/*void crsfPioTxEmptyHandlerIrq(){
    //printf("in irq\n");
    pio_interrupt_clear(crsfPio, 0);    // clear the irq flag in the pio that was set when Txfifo is empty
    irq_set_enabled (CRSF_IRQ_TXFIFO_EMPTY , false) ; // disable the IRQ to handle when TXFIFO is empty
    irq_clear (CRSF_IRQ_TXFIFO_EMPTY ); // clear the irq flag in NVIC
    irq_clear (CRSF_IRQ_RX_RECEIVED ); // clear the irq flag for receiving
    clearCrsfRxQueue();  // clear the queue and reset the process state  
    irq_set_enabled (CRSF_IRQ_RX_RECEIVED , true) ; // enable the IRQ to handle the received charaters
    crsf_uart_tx_program_stop(crsfPio ,crsfSmTx , CRSF_PIO_PIN);
    crsf_uart_rx_program_restart(crsfPio ,crsfSmTx , CRSF_PIO_PIN , true); // false = no invert     
    crsfMode = RECEIVING ;
}
*/
/*
int64_t alarm_callback_switchToReceiveMode(alarm_id_t id, void *user_data) {
    irq_clear (CRSF_IRQ_RX_RECEIVED ); // clear the irq flag for receiving
    clearCrsfRxQueue();  // clear the queue and reset the process state  
    //irq_set_enabled (CRSF_IRQ_RX_RECEIVED , true) ; // enable the IRQ to handle the received charaters
    crsf_uart_tx_program_stop(crsfPio ,crsfSmTx , CRSF_PIO_PIN);
    crsf_uart_rx_program_restart(crsfPio ,crsfSmRx , CRSF_PIO_PIN , true); // true = invert     
    irq_set_enabled (CRSF_IRQ_RX_RECEIVED , true) ; // enable the IRQ to handle the received charaters
    
    crsfMode = RECEIVING ;
    return 0;  // return 0 to avoid that the callback is called automatically in the future; we use a new add at each time.
}
*/

//#ifdef DEBUG_WITH_FIXED_RC_FRAME
uint8_t testBuffer[]={ 0xEE , 0x18, 0x16, 0xF2, 0x83, 0x1E, 0xFC, 0x00, 0x08, 0x3E , 0XF0, 0X81,
                        0X0F, 0x7C , 0xE0, 0x03, 0x1F, 0XF8, 0xC0, 0x07, 0x3E, 0XF0, 0X81, 0x0F, 0x7C, 0X63};
uint8_t testBufferLength = 0x1A;                        
//#endif

#define MODULE_ADDRESS  0xEE

void sendCrsfRcFrame(void){   //  called by main loop : 
//  when crsf state is receiving mode , if last transmit is more than x msec and if Rc channels are available,
//  - process once more the Rx queue
//  - clear the Rx queue from crsf (just to be sure because all bytes should have been processed)
//  - fill the TX buffer with RC channels ,
//  - switch the pio to transmit mode,
//  - start the dma to send the RC channel
//  - set state to transmit and save the timestamp.
//  - clear the irq on PIO1_IRQ1 and enable it (it is fired by the PIO when txfifo becomes emtpty)
//  when crsf state machine fires the IRQ because TXFIFO is empty, it call crsfPioTxEmptyHandlerIrq() 
//  - clear the IRQ flag for TXFIFO and disabled it
//  - clear the IRQ flag for receiving
//  _ clear the Rx queue
//  - enable the receiving IRQ
//  - switch PIO receiving mode
//  - set state to receiving mode
//  when a character is received in PIO, it fires an irq that is handled in crsfPioRxHandlerIrq()
//    - the irq push the data to a queue
//  The main loop read the queue and process the received data in handleTlmIn()  
    static uint32_t lastRcFrameSendMillis = 0;
    static uint32_t lastRcFrameSendMicros = 0;
    static bool waitFirstFrameFromElrs = true;
    ////if ( (crsfMode == RECEIVING) && ( (millis() - lastRcFrameSendMillis ) > CRSF_RC_FRAME_INTERVAL ) && ( crsfRcFrameReady) ) {
    //if ( ( (millis() - lastRcFrameSendMillis ) >= CRSF_RC_FRAME_INTERVAL ) && ( crsfRcFrameReady) ) {
    //if ( ( (millis() - lastRcFrameSendMillis ) >= CRSF_RC_FRAME_INTERVAL )  ) {    
    if ( ( (micros() - lastRcFrameSendMicros ) >= crsfRcFrameIntervalMicros )  ) {    
        if (waitFirstFrameFromElrs && oneFrameFromElrsReceived){
            waitFirstFrameFromElrs = false;
            createRadioModelIdFrame();
        } else {
            createRcChannelsFrame();
        }    
        dma_channel_set_read_addr (crsf_dma_chan, &crsfTxBuffer[0], false);
        dma_channel_set_trans_count (crsf_dma_chan, crsfTxBufferLength, true) ; // start the dma
//        add_alarm_in_us( (crsfTxBufferLength *24)+40 , alarm_callback_switchToReceiveMode , NULL , false); // 400000 baud = 2.5 usec/bit = 25 usec/byte
        crsfMode = SENDING ;
        lastRcFrameSendMillis = millis();
        lastRcFrameSendMicros = micros();
    }
}


void clearCrsfRxQueue(){
    uint8_t data;
    while ( !queue_is_empty(&crsfRxQueue)) queue_try_remove (&crsfRxQueue, &data); // clear the queue
    tlmState = CRSF_TLM_NOT_RECEIVING; // reset the state of processing data
}

        
void fillCrsfTxBuffer(uint8_t c){
    crsfTxBuffer[crsfTxBufferLength++] = c;
}

void fillCrsfTxBuffer(uint8_t * bufferFrom , uint8_t length){
    uint8_t count = 0;
    while (count < length){
        crsfTxBuffer[crsfTxBufferLength] = bufferFrom[count++];
        crsfTxBufferLength++;
    }
}

void createRcChannelsFrame(){
        crsfTxBufferLength = 0;
        fillCrsfTxBuffer(MODULE_ADDRESS);
        fillCrsfTxBuffer(CRSF_FRAME_RC_PAYLOAD_SIZE + 2);
        fillCrsfTxBuffer(CRSF_FRAMETYPE_RC_CHANNELS);
        #ifdef DEBUG_WITH_FIXED_RC_FRAME
        fillCrsfTxBuffer(  &testBuffer[3] , CRSF_FRAME_RC_PAYLOAD_SIZE ); // payload size include type and CRC
        #else
        fillCrsfTxBuffer( ( (uint8_t *) &crsfRcFrame) + 3 , CRSF_FRAME_RC_PAYLOAD_SIZE );
        #endif
        fillCrsfTxBuffer(crsf_crc.calc(&crsfTxBuffer[2], CRSF_FRAME_RC_PAYLOAD_SIZE + 1));  // crc includes frame type and so is 1 byte more than the payload
}        

#define MODEL_ID 0
void createRadioModelIdFrame(){
        crsfTxBufferLength = 0;
        fillCrsfTxBuffer(CRSF_ADDRESS_FLIGHT_CONTROLLER);
        fillCrsfTxBuffer(8); // size of payload
        fillCrsfTxBuffer(CRSF_FRAMETYPE_COMMAND);
        fillCrsfTxBuffer(CRSF_ADDRESS_CRSF_TRANSMITTER) ; 
        fillCrsfTxBuffer(CRSF_ADDRESS_RADIO_TRANSMITTER) ;
        fillCrsfTxBuffer(SUBCOMMAND_CRSF);
        fillCrsfTxBuffer(COMMAND_MODEL_SELECT_ID) ;
        fillCrsfTxBuffer(MODEL_ID);   // sequence of the model ID (in range 0...63)        
        fillCrsfTxBuffer(command_crc8(&crsfTxBuffer[2], 6));
        fillCrsfTxBuffer(crc8(&crsfTxBuffer[2], 7));
}

/*
void sendCRSFRcFrame(){
    static uint32_t lastRcFrameSendMillis ;
    if ( ( (millis() - lastRcFrameSendMillis ) > CRSF_RC_FRAME_INTERVAL ) && ( CRSFRcFrameReady) ) {
        //Serial1.write(MODULE_ADDRESS);
        //Serial1.write(CRSF_FRAME_RC_PAYLOAD_SIZE + 2);
        //Serial1.write(CRSF_FRAMETYPE_RC_CHANNELS);
        //Serial1.write(& CRSFRcFrame[3], CRSF_FRAME_RC_PAYLOAD_SIZE);
        //Serial1.write(crsf_crc.calc(&CRSFRcFrame[2], 22));
        crsfTxBufferLength = 0;
        fillCrsfBuffer(MODULE_ADDRESS);
        fillCrsfBuffer(CRSF_FRAME_RC_PAYLOAD_SIZE + 2);
        fillCrsfBuffer(CRSF_FRAMETYPE_RC_CHANNELS);
        fillCrsfBuffer(&CRSFRcFrame[3], CRSF_FRAME_RC_PAYLOAD_SIZE);
        fillCrsfBuffer(crsf_crc.calc(&crsfTXBuffer[2], 22));    
    }
}
*/

// read the rx queue and process each byte (= tlm data)
// when a frame has been totally received, check it, and store the tlm data in separated fields (to be sent later by sport process)
void handleTlmIn(){
   //static uint8_t tlmBuffer[50];
    static uint8_t tlmCounter;
    static uint8_t tlmLength ; // Length in the second byte of the message (is equal to the payload+2 because type and crc are included)
    static uint8_t tlmType; 
    uint8_t c;
    while(! queue_is_empty(&crsfRxQueue)) {
        queue_try_remove (&crsfRxQueue,&c);        
        //printf("%x\n", c);
        switch (tlmState) {
            case CRSF_TLM_NOT_RECEIVING :
                if (c == CRSF_ADDRESS_TLM) {
                    tlmCounter=0;
                    tlmFrame.tlmBuffer[tlmCounter++] = c;
                    tlmState = CRSF_TLM_RECEIVING_SIZE;
                }
            break;
            case CRSF_TLM_RECEIVING_SIZE :
                tlmFrame.tlmBuffer[tlmCounter++] = c;
                tlmLength = c;
                tlmState = CRSF_TLM_RECEIVING_TYPE;
            break;
            case CRSF_TLM_RECEIVING_TYPE :
                if ( ( ( c == CRSF_FRAMETYPE_GPS) && (tlmLength == (CRSF_FRAME_GPS_PAYLOAD_SIZE+2) ) )
                    || ( ( c == CRSF_FRAMETYPE_VARIO) && (tlmLength == (CRSF_FRAME_VARIO_PAYLOAD_SIZE+2) ) )
                    || ( ( c == CRSF_FRAMETYPE_BATTERY_SENSOR) && (tlmLength == (CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE+2) ) )
                    || ( ( c == CRSF_FRAMETYPE_ATTITUDE) && (tlmLength == (CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE+2) ) ) 
                    || ( ( c == CRSF_FRAMETYPE_LINK_STATISTICS) && (tlmLength == (CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE+2) ) ) 
                    || ( ( c == CRSF_FRAMETYPE_RADIO_ID ) && (tlmLength == (CRSF_FRAME_OPENTX_SYNC_PAYLOAD_SIZE+2) ) )     ){
                    tlmType = c;
                    tlmState = CRSF_TLM_RECEIVING_PAYLOAD;
                    tlmFrame.tlmBuffer[tlmCounter++] = c;
                } else {
                    tlmState = CRSF_TLM_NOT_RECEIVING;
                }   
            break;
            case CRSF_TLM_RECEIVING_PAYLOAD :
                tlmFrame.tlmBuffer[tlmCounter++] = c;
                if (tlmCounter >  tlmLength  ) { // e.g. if tlmLength = 3, CRC is at idx 4 (because there is an adress in pos 0)
                    tlmState = CRSF_TLM_RECEIVING_CRC;
                }
            break;
            case CRSF_TLM_RECEIVING_CRC :
                if ( c != crsf_crc.calc(&tlmFrame.tlmBuffer[2], tlmLength - 1)) {  
                    tlmState = CRSF_TLM_NOT_RECEIVING;
                } else {  // we received a full telemetry packet; we can save it.
                    storeTlmFrame();
                //printf("frame received\n");
                }
            break;
            
        }
    }
}

extern field fields[SPORT_TYPES_MAX];

void storeTlmFrame(){
#ifdef DEBUG_RECEIVED_FRAME    
    printf("CRSF In : ");
    for (uint8_t i = 0 ; i<(tlmFrame.tlmBuffer[1] +1); i++){
        printf(" %2X ",tlmFrame.tlmBuffer[i] );
    }
    printf("\n");
#endif    
    int32_t temp;
    oneFrameFromElrsReceived = true ;
    switch (tlmFrame.tlmBuffer[2]) { // byte 2 = type
        case CRSF_FRAMETYPE_GPS:
            fields[LATITUDE].available = getCrossfireTelemetryValue(3, 4 , temp) ;        
            //temp = tlmFrame.gpsFrame.latitude;
            fields[LATITUDE].value = (( ((((uint32_t)( temp < 0 ? -temp : temp)) /10 ) * 6 ) / 10 ) & 0x3FFFFFFF)  ;  
            if(temp < 0) fields[LATITUDE].value |= 0x40000000;
            
            fields[LONGITUDE].available = getCrossfireTelemetryValue(7, 4 , temp) ;        
            //temp = tlmFrame.gpsFrame.longitude; // degree with 7 decimals
            fields[LONGITUDE].value = (( ((((uint32_t)( temp < 0 ? -temp : temp)) /10 ) * 6 ) / 10 ) & 0x3FFFFFFF)  | 0x80000000;  
            if(temp < 0) fields[LONGITUDE].value |= 0x40000000;
            
            fields[GROUNDSPEED].available = getCrossfireTelemetryValue(11, 2 , temp) ;
            fields[GROUNDSPEED].value = (int32_t)(((float) temp ) * 5.4); //( from 0.1 km/h  to 0.01 kts = * 5.4)
            
            fields[HEADING].available = getCrossfireTelemetryValue(13, 2 , temp) ;
            fields[HEADING].value = temp ; //( from 0.01 degree to 0.01 degree )
            

            fields[ALTITUDE].available = getCrossfireTelemetryValue(15, 2 , temp) ;
            fields[ALTITUDE].value = (temp - 1000) * 100;  //(from m with offset 1000m to cm)
            
            //fields[NUMSAT].value = tlmFrame.gpsFrame.numSat;
            fields[NUMSAT].available = getCrossfireTelemetryValue(17, 1 , fields[NUMSAT].value) ;
            break;
        case CRSF_FRAMETYPE_VARIO:
            //fields[VSPEED].value = tlmFrame.varioFrame.vSpeed;
            fields[VSPEED].available = getCrossfireTelemetryValue(3, 2 , fields[VSPEED].value);
            break;
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            fields[MVOLT].available = getCrossfireTelemetryValue(3, 2 , temp) ;
            fields[MVOLT].value = temp * 10; // from 0.1V to 0.01V
            
            fields[CURRENT].available = getCrossfireTelemetryValue(5, 2 , fields[CURRENT].value);
            fields[CURRENT].value = temp * 100; // from 0.1A to 0.001A
            
            //fields[CAPACITY].value = tlmFrame.voltageFrame.capacity;
            fields[CAPACITY].available = getCrossfireTelemetryValue(7, 3 , fields[CAPACITY].value);
            //fields[REMAIN].value = tlmFrame.voltageFrame.remain;
            fields[REMAIN].available = getCrossfireTelemetryValue(10, 1 , fields[REMAIN].value);
            break;
        case CRSF_FRAMETYPE_ATTITUDE:
            fields[PITCH].available = getCrossfireTelemetryValue(3, 2 , temp);
            fields[PITCH].value = (int32_t)(((float) temp ) * 0.00573) ; // from 1/10000 Rad to deg ; so * 0.00573
            
            
            fields[ROLL].available = getCrossfireTelemetryValue(5, 2 , temp);
            fields[ROLL].value =  (int32_t)(((float) temp ) * 0.00573) ; // from 1/10000 Rad to deg ; so * 0.00573
            
            fields[YAW].available = getCrossfireTelemetryValue(7, 2 , temp);            
            fields[YAW].value = (int32_t)(((float) temp ) * 0.00573) ; // from 1/10000 Rad to deg ; so * 0.00573
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            //fields[UPLINK_RSSI_1].value = tlmFrame.linkstatisticsFrame.uplink_RSSI_1;
            fields[UPLINK_RSSI_1].available = getCrossfireTelemetryValue(3, 1 , fields[UPLINK_RSSI_1].value);
            //fields[UPLINK_RSSI_2].value = tlmFrame.linkstatisticsFrame.uplink_RSSI_2;
            fields[UPLINK_RSSI_2].available = getCrossfireTelemetryValue(4, 1 , fields[UPLINK_RSSI_2].value);
            //fields[UPLINK_LINK_QUALITY].value = tlmFrame.linkstatisticsFrame.uplink_Link_quality;
            fields[UPLINK_LINK_QUALITY].available = getCrossfireTelemetryValue(5, 1 , fields[UPLINK_LINK_QUALITY].value);
            //fields[UPLINK_SNR].value = tlmFrame.linkstatisticsFrame.uplink_SNR;
            fields[UPLINK_SNR].available = getCrossfireTelemetryValue(6, 1 , fields[UPLINK_SNR].value);
            //fields[ACTIVE_ANTENNA].value = tlmFrame.linkstatisticsFrame.active_antenna;
            fields[ACTIVE_ANTENNA].available = getCrossfireTelemetryValue(7, 1 , fields[ACTIVE_ANTENNA].value);
            //fields[RF_MODE].value = tlmFrame.linkstatisticsFrame.rf_Mode;
            fields[RF_MODE].available = getCrossfireTelemetryValue(8, 1 , fields[RF_MODE].value);
            //fields[UPLINK_TX_POWER].value = tlmFrame.linkstatisticsFrame.uplink_TX_Power;
            fields[UPLINK_TX_POWER].available = getCrossfireTelemetryValue(9, 1 , fields[UPLINK_TX_POWER].value);
            //fields[DOWNLINK_RSSI].value = tlmFrame.linkstatisticsFrame.downlink_RSSI;
            fields[DOWNLINK_RSSI].available = getCrossfireTelemetryValue(10, 1 , fields[DOWNLINK_RSSI].value);
            //fields[DOWNLINK_LINK_QUALITY].value = tlmFrame.linkstatisticsFrame.downlink_Link_quality;
            fields[DOWNLINK_LINK_QUALITY].available = getCrossfireTelemetryValue(11, 1 , fields[DOWNLINK_LINK_QUALITY].value);
            //fields[DOWNLINK_SNR].value = tlmFrame.linkstatisticsFrame.downlink_SNR;
            fields[DOWNLINK_SNR].available = getCrossfireTelemetryValue(12, 1 , fields[DOWNLINK_SNR].value); 
            //printf(" Rf mode %x", tlmFrame.linkstatisticsFrame.rf_Mode);
            break;
        case CRSF_FRAMETYPE_RADIO_ID:
            int rate ;
            int offset;
            if ( ( tlmFrame.opentxSyncFrame.dest_addr == 0xEA ) && (tlmFrame.opentxSyncFrame.synchrType == 0x10) ) {
                rate = swap_int32(tlmFrame.opentxSyncFrame.rate) / 10 ; // save rate in usec
                offset = swap_int32(tlmFrame.opentxSyncFrame.offset) / 10 ;// save offset in usec
            }
            #ifdef DEBUG_FRAME_RADIO_ID
            printf( "tlm=");
            for ( uint8_t i = 0; i < (0X0D+2) ; i++){
                printf("%X ",tlmFrame.tlmBuffer[i]);
            }
            printf("rate=%i   offset=%i\n", rate , offset );
            #endif
            break;
    }
}

bool getCrossfireTelemetryValue(uint8_t index, uint8_t len , int32_t & value)
{
  bool result = false;
  uint8_t * pbyte = & tlmFrame.tlmBuffer[index];
  value = (*pbyte & 0x80) ? -1 : 0;
  for (uint8_t i=0; i<len; i++) {
    value <<= 8;
    if (*pbyte != 0xff) {
      result = true;
    }
    value += *pbyte++;
  }
  return result;
}