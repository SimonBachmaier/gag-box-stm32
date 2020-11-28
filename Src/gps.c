/*
 * gps.c
 *
 *  Created on: 18.07.2020
 *      Author: danie
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "gps.h"
#include "usart.h"
#include <stdio.h>

// Global variables
char gps_MsgBuffer[GPS_MSGCOUNT][GPS_MAXMSGLEN] = { { '\0' } };	//buffer for received GPS messages
char* info = NULL;
char gps_ReceiveMsg[GPS_MAXMSGLEN] = { '\0' };
char gps_CharBuffer = '\0';

/**
 * Initializes the message buffer
 */

void InitMsgBuffer() {
	for(int i = 0; i < GPS_MSGCOUNT; i++)
	{
		memset(gps_MsgBuffer[i], 0, GPS_MAXMSGLEN);
	}
}

/**
 * Parses the received GPS message string and returns part of the message specified by the dataId and dataPosition
 */
char* GetGpsData(char msgStorage[][GPS_MAXMSGLEN], enum GPS dataID, uint8_t dataPosition)
{
  char msgSplit[GPS_MAXMSGLEN] = { '\0' };
  memset(msgSplit, 0, sizeof(msgSplit));
  strcpy(msgSplit, msgStorage[dataID]);

  char splitAt[] = ",*";
  char* msgSplitPointer = &msgSplit[0];
  info = strsep(&msgSplitPointer, splitAt);

  uint8_t positionCounter = 0;
  while(info != NULL)
  {
    if(positionCounter == dataPosition)
      break;

    positionCounter++;

    info = strsep(&msgSplitPointer, splitAt);
  }

  return(info);
}

/**
 * Stores the GPS data packages at right buffer position.
 */
void SortGpsData(char msgStorage[][GPS_MAXMSGLEN], char* msg)
{
  enum GPS id = err;
  char currentID[6];	//ID of message in gpsReceiveMsg

  strncpy(currentID,msg,6);

  if(!strcmp(currentID,"$GPGGA"))
  {
      id = GGA;
  }
  else if(!strcmp(currentID,"$GPGSA"))
  {
      id = GSA;
  }
  else if(!strcmp(currentID,"$GPGSV"))
  {
    char gsvMsgCount[10];

    strncpy(gsvMsgCount,msg,10);

    if(!strcmp(gsvMsgCount,"$GPGSV,3,3"))
    {
	      id = GSV3;
    }
    else if(!strcmp(gsvMsgCount,"$GPGSV,3,2")||!strcmp(gsvMsgCount,"$GPGSV,2,2"))
    {
	      id = GSV2;
    }
    else
    {
	      id = GSV1;
    }
  }
  else if(!strcmp(currentID,"$GPGLL"))
  {
      id = GLL;
  }
  else if(!strcmp(currentID,"$GPRMC"))
  {
      id = RMC;
  }
  else if(!strcmp(currentID,"$GPVTG"))
  {
      id = VTG;
  }
  else if(!strcmp(currentID,"$GPZDA"))
  {
      id = ZDA;
  }
  else
  {
      id = err;
  }
  if (strlen(msg) != 0)
  {
	  memset(msgStorage[id], 0, GPS_MAXMSGLEN);
	  strncpy(msgStorage[id], msg, sizeof msgStorage[id]-1);
  }
}

/**
 * Returns lat or long values and copy degrees, minutes and (seconds*1000) to pointer latLonValues
 */
void getLatOrLon(fakeBoolean GetLonInsteadOfLat, uint16_t latLonValues[])
{
	char *end;
	char delimiter[] = ".";
	char* data;
	char* ptr;
	if (GetLonInsteadOfLat)
	{
		data = GetGpsData(gps_MsgBuffer,GPS_GGA_LONGITUDE);

		char degreesString[3] = { data[0], data[1], data[1] };
		uint32_t degrees = strtoul(degreesString, &end, 10);
		latLonValues[0] = (uint16_t) degrees;
		ptr = &data[3];
	} else
	{
		data = GetGpsData(gps_MsgBuffer,GPS_GGA_LATITUDE);

		char degreesString[2] = { data[0], data[1] };
		uint32_t degrees = strtoul(degreesString, &end, 10);
		latLonValues[0] = (uint16_t) degrees;

		ptr = &data[2];
	}

	ptr = strtok(ptr, delimiter);
	uint32_t minutes = strtoul(ptr, &end, 10);
	latLonValues[1] = (uint16_t) minutes;

	ptr = strtok(NULL, delimiter);
	uint32_t minutesTimes10000 = strtoul(ptr, &end, 10);
	latLonValues[2] = (uint16_t) (minutesTimes10000 * 6); // --> Sekunden / 1000
}

/**
 * Get current speed
 */
uint32_t getSpeed()
{
    char* speedPtr = GetGpsData(gps_MsgBuffer,GPS_RMC_SPEED);
    char speedNullTerminated[] = {speedPtr[0], speedPtr[1], speedPtr[2], speedPtr[3], speedPtr[4],  '\0'};
    double speed = atof(speedNullTerminated);
    speed = speed * 1.852; // convert from knots to kmh

    return (int32_t)(speed * 10000); // --> * 10000 to convert everything to int32t
}

/**
 * Get current altitude
 */
int32_t getAltitude()
{
    char* altitudePtr = GetGpsData(gps_MsgBuffer,GPS_GGA_ALTITUDE);
    char altitudeNullTerminated[] = {altitudePtr[0], altitudePtr[1], altitudePtr[2], altitudePtr[3], altitudePtr[4], altitudePtr[5], altitudePtr[6],  '\0'};
    double altitude = atof(altitudeNullTerminated);

    return (int32_t)(altitude * 10); // --> * 10 to convert everything to int32t
}

/**
 * Get current number of satellites
 */
uint8_t getSatellites()
{
    char* altitudePtr = GetGpsData(gps_MsgBuffer, GPS_GGA_SATELLLITES);
    char altitudeNullTerminated[] = {altitudePtr[0], altitudePtr[1],  '\0'};
    uint32_t altitude = atoi(altitudeNullTerminated);

    return (uint8_t) altitude; // --> * 10 to convert everything to int32t
}
