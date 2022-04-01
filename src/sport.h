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

#include "Arduino.h"

#define SPORT_TYPES_MAX 24
struct field {
    int32_t value;
    bool available;
    uint32_t nextMillis;
    uint16_t interval; // msec
    uint8_t deviceId;
    uint16_t fieldId;
} ;


// default SPORT_SENSOR_ID
#define DATA_ID_VARIO  0x00  // = sensor 0 used for Alt and Vspeed
#define DATA_ID_FLVSS  0xA1  //          1
#define DATA_ID_FAS    0x22  //          2
#define DATA_ID_GPS    0x83  //          3 used for all GPS data
#define DATA_ID_RPM    0xE4  //          4
#define DATA_ID_ACC    0x67  //          7
//list of 28 device ID codes is (in sequence)
// 0x00,0xA1,0x22,0x83,0xE4,0x45,0xC6,0x67,0x48,0xE9,0x6A,0xCB,0xAC,0x0D,0x8E,0x2F,0xD0,0x71,0xF2,0x53,0x34,0x95,0x16,0xB7,0x98,0x39,0xBA,0x1B


// FrSky new DATA IDs (2 bytes) (copied from openTX telemetry/frsky_sport.cpp on 11 jul 2014) // those values are not used directly but bits 4 up to 11 are stored in an array in oXs_out_frsky.cpp 
#define ALT_FIRST_ID            0x0100
#define ALT_LAST_ID             0x010f
#define VARIO_FIRST_ID          0x0110
#define VARIO_LAST_ID           0x011f
#define CURR_FIRST_ID           0x0200
#define CURR_LAST_ID            0x020f
#define VFAS_FIRST_ID           0x0210
#define VFAS_LAST_ID            0x021f
#define CELLS_FIRST_ID          0x0300
#define CELLS_SECOND_ID         0x0301
#define CELLS_THIRD_ID          0x0302
#define CELLS_LAST_ID           0x030f
#define T1_FIRST_ID             0x0400
#define T1_LAST_ID              0x040f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define RPM_FIRST_ID            0x0500
#define RPM_LAST_ID             0x050f
#define FUEL_FIRST_ID           0x0600
#define FUEL_LAST_ID            0x060f
#define ACCX_FIRST_ID           0x0700
#define ACCX_LAST_ID            0x070f
#define ACCY_FIRST_ID           0x0710      
#define ACCY_LAST_ID            0x071f
#define ACCZ_FIRST_ID           0x0720
#define ACCZ_LAST_ID            0x072f
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850
#define GPS_TIME_DATE_LAST_ID   0x085f
#define A3_FIRST_ID             0x0900
#define A3_LAST_ID              0x090f
#define A4_FIRST_ID             0x0910
#define A4_LAST_ID              0x091f
#define AIR_SPEED_FIRST_ID      0x0a00
#define AIR_SPEED_LAST_ID       0x0a0f
#define RSSI_ID                 0xf101  // please do not use this code because it is already used by the receiver
#define ADC1_ID                 0xf102  // please do not use this code because it is already used by the receiver
#define ADC2_ID                 0xf103
#define BATT_ID                 0xf104
#define SWR_ID                  0xf105  // please do not use this code because it is already used by the receiver
#define UPLINK_RSSI_1_ID        0x0c00  // to check if this range is valid
#define UPLINK_RSSI_2_ID        0x0c01
#define UPLINK_LINK_QUALITY_ID  0x0c02
#define UPLINK_SNR_ID           0x0c03
#define ACTIVE_ANTENNA_ID       0x0c04
#define RF_MODE_ID              0x0c05
#define UPLINK_TX_POWER_ID      0x0c06
#define DOWNLINK_RSSI_ID        0x0c07
#define DOWNLINK_LINK_QUALITY_ID  0x0c08
#define DOWNLINK_SNR_ID           0x0c09



enum fieldIdx {
      LATITUDE =0,
      LONGITUDE,
      GROUNDSPEED ,
      HEADING, 
      ALTITUDE ,
      NUMSAT ,
      MVOLT, 
      CURRENT,
      CAPACITY,
      REMAIN,
      VSPEED,
      PITCH,
      ROLL,
      YAW ,
    UPLINK_RSSI_1 , UPLINK_RSSI_2 , UPLINK_LINK_QUALITY , UPLINK_SNR , ACTIVE_ANTENNA, RF_MODE ,
    UPLINK_TX_POWER , DOWNLINK_RSSI , DOWNLINK_LINK_QUALITY , DOWNLINK_SNR
};

void setupSport(void);

void sportPioRxHandlerIrq() ;   // when a byte is received on the Sport, read the pio Sport fifo and push the data to a queue (to be processed in the main loop)

void handleSportRxTx(void);
void sendNextSportFrame(uint8_t data_id) ; // search for the next data to be sent
void sendOneSport(uint8_t idx); 

