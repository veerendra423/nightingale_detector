/*  
 *  Copyright (C) 2021
 *  All Rights Reserved. 
 *
 *  The contents of this software are proprietary and confidential, 
 *  LLC.  No part of this program may be photocopied, reproduced or translated 
 *  into another programming language without prior written consent of LLC
 */

#ifndef __MISB_0601_H__
#define __MISB_0601_H__

#ifndef H_KLVCODE
#define H_KLVCODE

#ifdef __cplusplus
extern "C" {
#endif
const int PACKET_LENGTH = 69;
unsigned char packetBuffer[70];

void setLatitude(float Dlatitude);
void setLongitude(float Dlongitude);
void setAltitude(float Daltitude);
void setRol(float Drol);
void setPitch(float Dpitch);
void setYaw(float Dyaw);
void setHdg(float Dhdg);
void makePacket(unsigned char *buff);

#ifdef __cplusplus
}
#endif

#endif /* H_KLVCODE */

enum misbST0601Tag{
	TIMESTAMP_TAG = 0x02,
        TIMESTAMP_LENGTH = 0x08,
	LATITUDE_TAG = 0x0D,
	LATITUDE_LENGTH = 0x04,
	LONGITUDE_TAG = 0x0E,
	LONGITUDE_LENGTH = 0x04,
	ALTITUDE_TAG = 0x0F,
	ALTITUDE_LENGTH = 0x02,
	CHECKSUM_TAG = 0x01,
	CHECKSUM_LENGTH = 0x02,
	ROLL_TAG = 0x14,
	ROLL_LENGTH = 0x04,
	PITCH_TAG = 0x13,
	PITCH_LENGTH = 0x04,
	YAW_TAG = 0x12,
	YAW_LENGTH = 0x04,
	HDG_TAG = 0x05,
	HDG_LENGTH = 0x02
};

#endif                          // __MISB_0601_H__
