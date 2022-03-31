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

#include <inttypes.h>

#define CRSF_ADDRESS_CRSF_RECEIVER 0xEC // address of the receiver (used for telemetry)

typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,

    //CRSF_FRAMETYPE_ELRS_STATUS = 0x2E, ELRS good/bad packet count and status flags

    CRSF_FRAMETYPE_COMMAND = 0x32,
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ  = 0x78,
    CRSF_FRAMETYPE_KISS_RESP = 0x79,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
} crsf_frame_type_e;


#define CRSF_FRAME_RC_PAYLOAD_SIZE 22  
#define CRSF_FRAME_GPS_PAYLOAD_SIZE 15 
#define CRSF_FRAME_VARIO_PAYLOAD_SIZE 2 
#define CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE 8
#define CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE 6 
#define CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE 8  // to change?
#define CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE 10

#define CRSF_ADDRESS_TLM 0xEA // this value has been seen using the logic analyser

enum CRSF_MODE {
    RECEIVING = 0,
    SENDING
};

typedef enum
{
    CRSF_FRAMEIDX_GPS = 0x00,
    CRSF_FRAMEIDX_VARIO = 0x01,
    CRSF_FRAMEIDX_BATTERY_SENSOR = 0x02,
    CRSF_FRAMEIDX_ATTITUDE = 0x03,
    CRSF_FRAMEIDX_FLIGHT_MODE = 0x04,
} crsf_frame_idx_e;

typedef enum {
    CRSF_TLM_NOT_RECEIVING = 0x00,
    CRSF_TLM_RECEIVING_SIZE ,
    CRSF_TLM_RECEIVING_TYPE ,
    CRSF_TLM_RECEIVING_PAYLOAD ,
    CRSF_TLM_RECEIVING_CRC,
} crsf_tlm_state;

struct gpsFrameStruct
{
    uint8_t  device_addr; // should be 0xEC (=receiver)
    uint8_t  frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t  type;        // from crsf_frame_type_e
    int32_t  latitude;     //( degree / 10`000`000 )
    int32_t  longitude;    // (degree / 10`000`000 )
    uint16_t groundspeed;  // ( km/h / 10 )
    uint16_t heading;      //( degree / 100 )
    uint16_t altitude;     //( meter ­1000m offset )
    uint8_t  numSat;       //( counter )
    uint8_t crc;
} __attribute__((packed)) ;

struct voltageFrameStruct
{
    uint8_t device_addr; // should be 0xEC (=receiver)
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
    uint16_t mVolt ;
    uint16_t current ;
    uint32_t capacity : 24; // there is only one uint32 (splitted in 24 and 8)
    uint32_t remain : 8 ;
    uint8_t crc;
} __attribute__((packed)) ;

struct varioFrameStruct
{
    uint8_t device_addr; // should be 0xEC (=receiver)
    uint8_t frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t type;        // from crsf_frame_type_e
    int16_t vSpeed ;     // in 0.1 m/sec
    uint8_t crc;
} __attribute__((packed)) ;


struct attitudeFrameStruct
{
    uint8_t  device_addr; // should be 0xEC (=receiver)
    uint8_t  frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t  type;        // from crsf_frame_type_e
    int16_t  pitch;     //( rad/1000 )
    int16_t  roll;    // (rad / 1000 )
    int16_t  yaw;     // ( rad / 1000 )
    uint8_t crc;
} __attribute__((packed)) ;

/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */

struct linkstatisticsFrameStruct
{
    uint8_t  device_addr; // should be 0xEC (=receiver)
    uint8_t  frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t  type;        // from crsf_frame_type_e
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
    uint8_t crc;
}__attribute__((packed)) ;


union {
 gpsFrameStruct gpsFrame;
 voltageFrameStruct voltageFrame;
 varioFrameStruct varioFrame;
 attitudeFrameStruct attitudeFrame;
 linkstatisticsFrameStruct linkstatisticsFrame;
 uint8_t tlmBuffer[50];
} tlmFrame ;

struct rcFrameStruct
{
    uint8_t  device_addr; // should be 0xEC (=receiver)
    uint8_t  frame_size;  // counts size after this byte, so it must be the payload size + 2 (type and crc)
    uint8_t  type;        // from crsf_frame_type_e
    uint8_t  rcChannels[22];     //( rad/1000 )
    uint8_t crc;
} __attribute__((packed)) ;

//void setup_DMA_PIO(); 
void setupCRSF();
void sendCrsfRcFrame();
void handleTlmIn();
void storeTlmFrame();
void crsfPioRxHandlerIrq();
//void processCrsfRxQueue();
//void fillCrsfBufferWithRcChannels();
void fillCrsfTxBuffer(uint8_t c);
void fillCrsfTxBuffer(uint8_t * bufferFrom , uint8_t length);
void crsfPioTxEmptyHandlerIrq();
void clearCrsfRxQueue();

        
