#include <Arduino.h>
#include "hardware/pio.h"
#include "hardware/dma.h"
//#include "hardware/irq.h"
//#include "uart_tx.pio.h"

#include "crsf.h"
#include "config_basic.h"
#include "crc.h"
#include "sport.h"


//// This is the same as the default UART baud rate on Pico
const uint SERIAL_BAUD_CRSF = 400000;

#define CRSF_RC_FRAME_INTERVAL 50 // msec

#define FRAME_TYPES_MAX 5
uint32_t crsfFrameNextMillis[FRAME_TYPES_MAX] = {0} ; 
uint8_t crsf_last_frame_idx = 0 ;  

voltageFrameStruct voltageFrame;
varioFrameStruct varioFrame;
attitudeFrameStruct attitudeFrame;
gpsFrameStruct gpsFrame;

uint8_t CRSFRcFrame[50]; // buffer that contains the frame to be sent (cvia dma)
uint8_t CRSFRcFrameLength;

//uint8_t CRSFTlmFrame[50];
//uint8_t CRSFTlmFrameLength;

extern bool CRSFRcFrameReady;

GENERIC_CRC8 crsf_crc(CRSF_CRC_POLY);

void setupCRSF(){
    Serial1.begin(SERIAL_BAUD_CRSF);
}

#define MODULE_ADDRESS  0xEE
void sendCRSFRcFrame(){
    static uint32_t lastRcFrameSendMillis ;
    if ( ( (millis() - lastRcFrameSendMillis ) > CRSF_RC_FRAME_INTERVAL ) && ( CRSFRcFrameReady) ) {
        Serial1.write(MODULE_ADDRESS);
        Serial1.write(CRSF_FRAME_RC_PAYLOAD_SIZE + 2);
        Serial1.write(CRSF_FRAMETYPE_RC_CHANNELS);
        Serial1.write(& CRSFRcFrame[3], CRSF_FRAME_RC_PAYLOAD_SIZE);
        Serial1.write(crsf_crc.calc(&CRSFRcFrame[2], 22));
    }
}

void handleTlmIn(){
    //static uint8_t tlmBuffer[50];

    static uint8_t tlmCounter;
    static uint8_t tlmPayloadSize ;
    static uint8_t tlmType; 
    crsf_tlm_state tlmState;
    tlmState = CRSF_TLM_NOT_RECEIVING;
    while ( Serial1.available() ) {
        uint8_t c = Serial1.read();
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
                tlmPayloadSize = c;
                tlmState = CRSF_TLM_RECEIVING_TYPE;
            break;
            case CRSF_TLM_RECEIVING_TYPE :
                if ( ( ( c == CRSF_FRAMETYPE_GPS) && (tlmPayloadSize == CRSF_FRAME_GPS_PAYLOAD_SIZE ) )
                    || ( ( c == CRSF_FRAMETYPE_VARIO) && (tlmPayloadSize == CRSF_FRAME_VARIO_PAYLOAD_SIZE ) )
                    || ( ( c == CRSF_FRAMETYPE_BATTERY_SENSOR) && (tlmPayloadSize == CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE ) )
                    || ( ( c == CRSF_FRAMETYPE_ATTITUDE) && (tlmPayloadSize == CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE ) ) ){
                    
                    tlmType = c;
                    tlmState = CRSF_TLM_RECEIVING_PAYLOAD;
                    tlmFrame.tlmBuffer[tlmCounter++] = c;
                } else {
                    tlmState = CRSF_TLM_NOT_RECEIVING;
                }   
            break;
            case CRSF_TLM_RECEIVING_PAYLOAD :
                tlmFrame.tlmBuffer[tlmCounter++] = c;
                if (tlmCounter == tlmPayloadSize) {
                    tlmState = CRSF_TLM_RECEIVING_CRC;
                }
            break;
            case CRSF_TLM_RECEIVING_CRC :
                if ( c != crsf_crc.calc(&tlmFrame.tlmBuffer[2], tlmPayloadSize + 1)) {  // to check !!!!!!!!!!!!!!!!
                    tlmState = CRSF_TLM_NOT_RECEIVING;
                } else {  // we received a full telemetry packet; we can save it.
                    storeTlmFrame();
                }

            break;
            
        }
    }
}

extern field fields[14];

void storeTlmFrame(){
    int32_t temp;
    switch (tlmFrame.tlmBuffer[2]) { // byte 2 = type
        case CRSF_FRAMETYPE_GPS:
            temp = tlmFrame.gpsFrame.longitude; // degree with 7 decimals
            fields[LONGITUDE].value = (( ((((uint32_t)( temp < 0 ? -temp : temp)) /10 ) * 6 ) / 10 ) & 0x3FFFFFFF)  | 0x80000000;  
            if(temp < 0) fields[LONGITUDE].value |= 0x40000000;
            fields[LONGITUDE].available = true ;        
            temp = tlmFrame.gpsFrame.latitude;
            fields[LATITUDE].value = (( ((((uint32_t)( temp < 0 ? -temp : temp)) /10 ) * 6 ) / 10 ) & 0x3FFFFFFF)  ;  
            if(temp < 0) fields[LATITUDE].value |= 0x40000000;
            fields[LATITUDE].available = true ;
            fields[GROUNDSPEED].value = tlmFrame.gpsFrame.groundspeed; //( km/h / 10 )
            fields[GROUNDSPEED].available = true;
            fields[HEADING].value = tlmFrame.gpsFrame.heading; //( degree / 100 )
            fields[HEADING].available = true;
            fields[ALTITUDE].value = tlmFrame.gpsFrame.altitude;
            fields[ALTITUDE].available = true;
            fields[NUMSAT].value = tlmFrame.gpsFrame.numSat;
            fields[NUMSAT].available = true;
            break;
        case CRSF_FRAMETYPE_VARIO:
            fields[VSPEED].value = tlmFrame.varioFrame.vSpeed;
            fields[VSPEED].available = true;
            break;
        case CRSF_FRAMETYPE_BATTERY_SENSOR:
            fields[MVOLT].value = tlmFrame.voltageFrame.mVolt;
            fields[MVOLT].available = true;
            fields[CURRENT].value = tlmFrame.voltageFrame.current;
            fields[CURRENT].available = true;
            fields[CAPACITY].value = tlmFrame.voltageFrame.capacity;
            fields[CAPACITY].available = true;
            fields[REMAIN].value = tlmFrame.voltageFrame.remain;
            fields[REMAIN].available = true;
            break;
        case CRSF_FRAMETYPE_ATTITUDE:
            fields[PITCH].value = tlmFrame.attitudeFrame.pitch;
            fields[PITCH].available = true;
            fields[ROLL].value = tlmFrame.attitudeFrame.roll;
            fields[ROLL].available = true;
            fields[YAW].value = tlmFrame.attitudeFrame.yaw;
            fields[YAW].available = true;            
            break;
    }
}

