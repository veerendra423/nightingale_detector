/*  
 *  Copyright (C) 2021
 *  All Rights Reserved. 
 *
 *  The contents of this software are proprietary and confidential, 
 *  LLC.  No part of this program may be photocopied, reproduced or translated 
 *  into another programming language without prior written consent of LLC
 */

#include "misbst0601.h"
#include <inttypes.h>
#include <netinet/in.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

//============================================================================

uint16_t altitude; // Map 0..(2^16-1) to -900..19000 meters.
uint32_t roll; // Map (0)..(2^32-1) to (0 to 360)
int32_t pitch; // Map -(2^31-1)..(2^31-1) to +/-180.
uint32_t yaw; // Map (0)..(2^32-1) to (0 to 360)
uint16_t hdg; // Map (0)..(2^16-1) to (0 to 360)
uint32_t checksum;
int32_t latitude; // map -(2^31-1)..(2^31-1) to +/- 90, Error Indicator: -(2^31) From MISB 601.2
int32_t longitude; //Map -(2^31-1)..(2^31-1) to +/-180. Error Indicator: -(2^31)
uint64_t timestamp;
int DEBUG = 1;

const unsigned char ldsVersion = 0x02;  //ldsVersion and uasLdsKey from MISB 601.2 spec
const unsigned char uasLdsKey[] = {0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01, 0x0E, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00};

unsigned char msgLength = 0x34;
// Entries in the form {Tag, Length}, Tag is specified in MISB 601.2, Length is BER short form
unsigned char timestampTagLen[] = {TIMESTAMP_TAG, TIMESTAMP_LENGTH};
unsigned char latitudeTagLen[] = {LATITUDE_TAG, LATITUDE_LENGTH};
unsigned char longitudeTagLen[] = {LONGITUDE_TAG, LONGITUDE_LENGTH};
unsigned char altitudeTagLen[] = {ALTITUDE_TAG, ALTITUDE_LENGTH};
unsigned char checksumTagLen[] = {CHECKSUM_TAG, CHECKSUM_LENGTH};
unsigned char rollTagLen[] = {ROLL_TAG, ROLL_LENGTH};
unsigned char pitchTagLen[] = {PITCH_TAG, PITCH_LENGTH};
unsigned char yawTagLen[] = {YAW_TAG, YAW_LENGTH};
unsigned char hdgTagLen[] = {HDG_TAG, HDG_LENGTH};
uint64_t updateTimestamp(void);

//============================================================================
// FUNCTIONS
//--------------------------------------------------
// Maps a value in an input range to a scaled value in the output range
int32_t mapValue(float val, float inStart, float inEnd, float outStart, float outEnd) {
        return (int32_t)(outStart + (((outEnd - outStart) / (inEnd - inStart)) * (val - inStart)));
}

// Maps(roll and yaw) a value in an input range to a scaled value in the output range
uint32_t mapryValue(float val, float inStart, float inEnd, float outStart, float outEnd) {
        return (uint32_t)(outStart + (((outEnd - outStart) / (inEnd - inStart)) * (val - inStart)));
}

//--------------------------------------------------
// Checksum algorithm from MISB 601.2, pg. 12
uint16_t makeChecksum(unsigned char *buff, unsigned short len) {
        uint16_t bcc = 0, i;
        for ( i = 0 ; i < len; i++)
    bcc += buff[i] << (8 * ((i + 1) % 2));
  return bcc;
}
//--------------------------------------------------
// Check if the system is big endian or not.
int sysIsBigEndian(void) {
        union {
                uint32_t i;
                char ch[4];
        } tmp = {0x01020304};

        return tmp.ch[0] == 1;
}

//--------------------------------------------------
// Convert ints of type uint64_t to network order, checks if conversion is needed.
uint64_t htonll(uint64_t num) {
        if (sysIsBigEndian()) return num;
        else return (((num & 0xFFULL) << 56) | ((num & 0xFF00000000000000ULL) >> 56) |
                                                                ((num & 0xFF00ULL) << 40) | ((num & 0x00FF000000000000ULL) >> 40) |
                                                                ((num & 0xFF0000ULL) << 24) | ((num & 0x0000FF0000000000ULL) >> 24) |
                                                                ((num & 0xFF000000ULL) << 8) | ((num & 0x000000FF00000000ULL) >> 8));
}

// Map -(2^31-1)..(2^31-1) to +/-90.
//API to set the latitude
void setLatitude(float Dlatitude)
{
       	latitude = (int32_t)mapValue(Dlatitude, -90.0, 90.0, -2147483647.0, 2147483647.0); // 2147483647 = (2^31 - 1)
    	latitude = (uint32_t)htonl(latitude);
    	if(DEBUG){
    		printf("setlattitude:%x\n", latitude);
    	}
}

// Map -(2^31-1)..(2^31-1) to +/-180.
//API to set the longitude
void setLongitude(float Dlongitude)
{
	longitude = (int32_t)mapValue(Dlongitude, -180.0, 180.0, -2147483647, 2147483647);
    	longitude = (uint32_t)htonl(longitude);
    	if(DEBUG){
    		printf("setlongitude:%x\n", longitude);
    	}
}

// Map 0..(2^16-1) to -900..19000 meters.
//API to set the Altitude
void setAltitude(float Daltitude)
{
	altitude = (uint16_t)mapValue(Daltitude, -900, 19000, 0, 65535);
    	altitude = (uint16_t)htons(altitude);
    	if(DEBUG){
    		printf("setaltitude:%x\n", altitude);
    	}
}

// Map (0)..(2^32-1) to (0 to 360)
//API to set the Roll
void setRol(float Drol)
{
	roll = (uint32_t)mapryValue(Drol, 0, 360, 0, 4294967295);
    	roll = (uint32_t)htonl(roll);
    	if(DEBUG){
    		printf("setRoll:%x\n", roll);
    	}
}

// Map -(2^31-1)..(2^31-1) to +/-180.
//API to set Pitch
void setPitch(float Dpitch)
{
	pitch = (int32_t)mapValue(Dpitch, -180.0, 180.0, -2147483647.0, 2147483647.0);
    	pitch = (int32_t)htonl(pitch);
    	if(DEBUG){
    		printf("setPitch:%x\n", pitch);    
    	}
}

// Map (0)..(2^32-1) to (0 to 360)
//API to set yaw
void setYaw(float Dyaw)
{
	yaw = (uint32_t)mapryValue(Dyaw, 0, 360, 0, 4294967295);
    	yaw = (uint32_t)htonl(yaw);
    	if(DEBUG){
    		printf("setYaw:%x\n",yaw);
    	}
}

// Map (0)..(2^16-1) to (0 to 360)
//API to set hdg
void setHdg(float Dhdg)
{
	hdg = (uint16_t)mapValue(Dhdg, 0, 360, 0, 65535);
    	hdg = (uint16_t)htons(hdg);
    	if(DEBUG){
    		printf("sethdg:%x\n",hdg);
    	}
}

//Timestamp update
void timestampupdate()
{
	timestamp = htonll(updateTimestamp());
        if(DEBUG){
		printf("timestamp:");
		printf("%" PRId64 "\n", timestamp);
	}
}
//--------------------------------------------------
// Assemble packet in the given buffer
void makePacket(unsigned char *buff) {
        // Copy all fields into the packet buffer
	timestampupdate();
	memcpy(&buff[0], &uasLdsKey, 16);
        memcpy(&buff[16], &msgLength, 1);
        memcpy(&buff[17], &timestampTagLen, 2);
        memcpy(&buff[19], &timestamp, 8);
        memcpy(&buff[27], &latitudeTagLen, 2);
        memcpy(&buff[29], &latitude, 4);
        memcpy(&buff[33], &longitudeTagLen, 2);
        memcpy(&buff[35], &longitude, 4);
        memcpy(&buff[39], &altitudeTagLen, 2);
        memcpy(&buff[41], &altitude, 2);
        memcpy(&buff[43], &rollTagLen, 2);
        memcpy(&buff[45], &roll, 4);
        memcpy(&buff[49], &pitchTagLen, 2);
        memcpy(&buff[51], &pitch, 4);
        memcpy(&buff[55], &yawTagLen, 2);
        memcpy(&buff[57], &yaw, 4);
	memcpy(&buff[61], &hdgTagLen, 2);
        memcpy(&buff[63], &hdg, 2);
	memcpy(&buff[65], &checksumTagLen, 2);
        //calculate checksum on buffer
        checksum = makeChecksum(buff, 67);
        memcpy(&buff[67], &checksum, 2);
	if(DEBUG){
		printf("makePacket\n");
		int i;
		for (i = 0; i < PACKET_LENGTH; ++i) {
        	   printf("0x%X ", buff[i]);
        	}
		printf("\n");
	}
        return;
}
//--------------------------------------------------
// Returns the current UNIX timestamp in microseconds
uint64_t updateTimestamp(void) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        return ((((uint64_t)ts.tv_sec * 1000000)) + (((uint64_t)ts.tv_nsec / 1000)));
}
