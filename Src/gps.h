/*
 * gps.h
 *
 *  Created on: 19.07.2020
 *      Author: danie
 */

#ifndef GPS_H_
#define GPS_H_

/**
 * Contains methods to parse and store the GPS data that is received via UART
 */

//status flags
#define FLAG_GPS_STATUS		0x8000	//Status		0: no connection	1: connected
#define FLAG_GPS_DATA		0x4000	//data Status		0: data invalid		1: data valid
#define FLAG_GPS_MSG_RECEIVE	0x2000	//message receive	0: no reception		1: receive ongoing
#define FLAG_GPS_MSG_PENDING	0x1000	//message pending	0: buffer empty		1: message in buffer

/*
$GPRMC message defines
$GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh

1    = UTC of position fix
2    = Data status (V = navigation receiver warning)
3    = Latitude of fix
4    = N or S
5    = Longitude of fix
6    = E or W
7    = Speed over ground in knots
8    = Track made good in degrees True
9    = UT date
10   = Magnetic variation degrees (Easterly var. subtracts from true course)
11   = E or W
12   = Checksum
*/
#define GPS_RMC_TIME		RMC,1
#define GPS_RMC_STATUS		RMC,2
#define GPS_RMC_LATITUDE	RMC,3
#define GPS_RMC_NORTH_SOUTH_L	RMC,4
#define GPS_RMC_LONGITUDE	RMC,5
#define GPS_RMC_EAST_WEST_L	RMC,6
#define GPS_RMC_SPEED		RMC,7
#define GPS_RMC_TRUE		RMC,8
#define GPS_RMC_DATE		RMC,9
#define GPS_RMC_MAGNETIC	RMC,10
#define GPS_RMC_EAST_WEST_M	RMC,11
#define GPS_RMC_CHECK		RMC,12

/*
$GPGSA message defines
$GPGSA,M,x,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,xx,x.x,x.x,x.x*hh

1    = Mode:
       M=Manual, forced to operate in 2D or 3D
       A=Automatic, 3D/2D
2    = Mode:
       1=Fix not available
       2=2D
       3=3D
3-14 = IDs of SVs used in position fix (null for unused fields)
15   = PDOP Position dilution of precision
16   = HDOP Horizontal dilution of precision
17   = VDOP Vertical dilution of precision
18   = Checksum
*/
#define GPS_GSA_MODE_MA		GSA,1
#define GPS_GSA_MODE_123	GSA,2
#define GPS_GSA_SVID_1		GSA,3
#define GPS_GSA_SVID_2		GSA,4
#define GPS_GSA_SVID_3		GSA,5
#define GPS_GSA_SVID_4		GSA,6
#define GPS_GSA_SVID_5		GSA,7
#define GPS_GSA_SVID_6		GSA,8
#define GPS_GSA_SVID_7		GSA,9
#define GPS_GSA_SVID_8		GSA,10
#define GPS_GSA_SVID_9		GSA,11
#define GPS_GSA_SVID_10		GSA,12
#define GPS_GSA_SVID_11		GSA,13
#define GPS_GSA_SVID_12		GSA,14
#define GPS_GSA_PDOP		GSA,15
#define GPS_GSA_HDOP		GSA,16
#define GPS_GSA_VDOP		GSA,17
#define GPS_GSA_CHECK		GSA,18

/*
$GPGGA message defines
$GPGGA,hhmmss.sss,ddmm.mmmm,a,dddmm.mmmm,a,x,xx,x.x,x.x,M,,,,xxxx*hh

1   = UTC of position in hhmmss.sss format, (000000.000 ~ 235959.999)
2   = Latitude in ddmm.mmmm format
      Leading zeros transmitted
3	= Latitude hemisphere indicator, ‘N’ = North, ‘S’ = South
4	= Longitude in dddmm.mmmm format
	  Leading zeros transmitted
5	= Longitude hemisphere indicator, 'E' = East, 'W' = West
6	= GPS quality indicator
	  0: position fix unavailable
	  1: valid position fix, SPS mode
	  2: valid position fix, differential GPS mode
	  3: GPS PPS Mode, fix valid
	  4: Real Time Kinematic. System used in RTK mode with fixed integers
	  5: Float RTK. Satellite system used in RTK mode. Floating integers
	  6: Estimated (dead reckoning) Mode
	  7: Manual Input Mode
	  8: Simulator Mode
7	= Number of satellites in use, (00 ~ 12)
8	= Horizontal dilution of precision, (00.0 ~ 99.9)
9	= mean sea level (geoid), (-9999.9 ~ 17999.9)
10	= Differential reference station ID, 0000 ~ 1023
	  NULL when DGPS not used
11   = Checksum
*/
#define GPS_GGA_TIME		GGA,1
#define GPS_GGA_LATITUDE	GGA,2
#define GPS_GGA_NSINDICATOR	GGA,3
#define GPS_GGA_LONGITUDE	GGA,4
#define GPS_GGA_EWINDICATOR	GGA,5
#define GPS_GGA_GPSQUALITY	GGA,6
#define GPS_GGA_SATELLLITES	GGA,7
#define GPS_GGA_HDOP		GGA,8
#define GPS_GGA_ALTITUDE	GGA,9
#define GPS_GGA_DGPSID		GGA,10
#define GPS_GGA_CHECK		GSA,11

#define GPS_VTG_SPEED		VTG,3

#define GPS_MAXMSGLEN 			80	//maximum length of one GPS message
#define GPS_MSGCOUNT 			10	//maximum amount of messages to be received
#define GPS_RINGBUFFERCOUNT 	30	//maximum amount of messages to be buffered
#define GPS_CHARBUFFERSIZE 		100	//maximum amount of messages to be buffered

typedef enum
{
  FALSE = 0U,
  TRUE = !FALSE
} fakeBoolean;

extern char gps_MsgBuffer[GPS_MSGCOUNT][GPS_MAXMSGLEN];	//buffer for received GPS messages
extern char gps_ReceiveMsg[GPS_MAXMSGLEN];		//single received GPS message
//static uint8_t charCount;
extern char gps_CharBuffer;	//buffer for character to be received
extern char* info;

enum GPS{RMC,GSA,GSV1,GSV2,GSV3,GLL,GGA,VTG,ZDA,err};	//position in gpsMsgBuffer

char* GetGpsData(char msgStorage[][GPS_MAXMSGLEN], enum GPS, uint8_t);
void SortGpsData(char[][GPS_MAXMSGLEN], char*);
void InitMsgBuffer();
void getLatOrLon(fakeBoolean, uint16_t[]);
uint32_t getSpeed();
int32_t getAltitude();
uint8_t getSatellites();
uint8_t getSatellites();

#endif /* GPS_H_ */
