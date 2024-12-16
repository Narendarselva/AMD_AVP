#ifndef _VEHICLE_INFO_H
#define _VEHICLE_INFO_H
/*
* Copyright (c) 2022-2023,Motovis Intelligent Technologies (Shanghai) Co.,Ltd
* Motovis Intelligent Confidential Proprietary
* FileName：vehicle_info.h
* summary:
* version: V1.0.0.0
* Author: motovis
* Finish data: 2022.22.3
* Notes：UTF-8
* Revision history：
*/

#ifndef _DirectStatus_
#define _DirectStatus_
typedef enum _DirectStatus
{
	StandStill = 0,
	Forward,
	Backward,
	Invalid
}DirectStatus;
#endif

#ifndef _GearStatus_
#define _GearStatus_
typedef enum _GearStatus
{
	None = 0,
	Parking,
	Reverse,
	Neutral,
	Drive
}GearStatus;
#endif

#ifndef _LightStatus_
#define _LightStatus_
typedef enum _LightStatus
{
	LightOff = 0,
	LLight,
	RLight,
	BothLight,
	ValidLight
}LightStatus;
#endif


typedef struct _CarInfo
{
    unsigned long long lTimeMsec;//timestamp,millsecond
    int nBrake;// 1，brake  0，not
    int nLLight;//left turn light 1、on 0、off
    int nRLight;//right turn light 1、on 0、off
    float fVelocity;//speed, km/h
    float fYawRate;//yaw angle rate
    int nTransmissionStatus;// Transmission 0 Invalid 1P 2R 3N 4D
    float fAlpha;//angle of wheel
    float fSteeingWheelAngle;// steelwheel angle，clockwise is +，unit is angle
    unsigned short  nFLWheelSpeedRC;//front leftwheel speed pulse 
    unsigned short  nFRWheelSpeedRC;//front right wheel speed pulse  
    unsigned short  nRLWheelSpeedRC;//rear left wheel speed pulse  
    unsigned short  nRRWheelSpeedRC;//rear right wheel speed pulse  
    int nPulseDirection;//wheel speed pulse direction
	int nGear;//gear 1-P 2-R 3-N 4-D
	float fWheelSpeed;//wheel speed,unit km/h
}CarInfo;

#endif