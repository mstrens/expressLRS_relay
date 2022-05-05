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
#include <stdint.h>
//#include "targets.h"

#define crclen 256
#define CRSF_CRC_POLY 0xd5

class GENERIC_CRC8
{
private:
    uint8_t crc8tab[crclen];
    uint8_t crcpoly;

public:
    GENERIC_CRC8(uint8_t poly);
    uint8_t calc(const uint8_t data);
    uint8_t calc(const uint8_t *data, uint8_t len, uint8_t crc = 0);
};

class GENERIC_CRC14
{
private:
    uint16_t crc14tab[crclen];
    uint16_t crcpoly;

public:
    GENERIC_CRC14(uint16_t poly);
    uint16_t calc(uint8_t *data, uint8_t len, uint16_t crc);
    uint16_t calc(volatile uint8_t *data, uint8_t len, uint16_t crc);
};
