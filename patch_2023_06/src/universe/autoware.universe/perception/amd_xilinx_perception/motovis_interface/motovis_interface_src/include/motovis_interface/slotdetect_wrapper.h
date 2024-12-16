#include "vehicle_info.h"

#ifndef _ArmPointFloat_
#define _ArmPointFloat_
typedef struct _exArmPointFloat_
{
	float x;
	float y;	
}exArmPointFloat;
#endif

#ifndef _ArmPointInt32_
#define _ArmPointInt32_
typedef struct _exArmPointInt32_
{
	unsigned int x;
	unsigned int y;	
}exArmPointInt32;
#endif

#ifndef _MvSlotPoint_
#define _MvSlotPoint_
typedef struct _exSlotPoint			
{
	exArmPointInt32 tImagePoint;		//slot image position
	exArmPointFloat tWorldPoint;		//slot world position
} exSlotPoint;
#endif

typedef struct _exSlotData			//slot data
{
	exSlotPoint  	tPoint0;		//slot point1 .image and woild point
	exSlotPoint  	tPoint1;		//slot point2 .image and woild point
	exSlotPoint  	tPoint2;		//slot point3 .image and woild point
	exSlotPoint  	tPoint3;		//slot point4 .image and woild point
	unsigned int	nSlotId;		//slot id
	unsigned int	nDirection;		//slot direction 0:left 1:right 2:front 3:rear		
	unsigned int 	nType;			//slot type
	unsigned int 	nAvailableState;//slot is suitable for parking?1=could use 0=not	
	float			fAngle;			//angle between x 
	float			fSlotTheta;		//degree of confidence
    unsigned char	aReserved[68]; 
} exSlotData;

typedef struct _exSlotResult
{
	unsigned int 		nStructLen;		//
	unsigned int      	nFrameId;		//frame id
	unsigned long long	lTimeMsec;		//timestamp
	unsigned int 		nDetectSlotNum;	//number of detect slot
    unsigned int      	nManageSlotNum;	//number of manage slot
	exSlotData    		tDetectSlot[16];//slot data of detect slot
	exSlotData    		tManageSlot[20];//slot data of manage slot
	unsigned char 		chRerseved[32];
} exSlotResult;

#ifndef _exFreeSpacePoint_
#define _exFreeSpacePoint_
typedef struct _exFreeSpacePoint_
{	
	exArmPointInt32 tImagePoint;	//image point 
	exArmPointFloat tWorldPoint;  //world point
	char	uType;				//obstacle type  3=ground pin 10=curb 12=people 13=bike 14=tricycle 15=pillar 16=obstacle 
	char   chReserved[7];					
}exARFreeSpacePoint;
#endif

#ifndef _exFreeSpaceRegionInfo_
#define _exFreeSpaceRegionInfo_
typedef struct _exFreeSpaceRegionInfo_
{
	unsigned long long  lTimeMsec;		//timestamp unit ms
	unsigned int		nFrameIndex;	//frame index
	unsigned int 		nPointNum;		//valid point number
	exARFreeSpacePoint tPoint[128];		//freespace point array	
}exFreeSpaceRegionInfo;
#endif

#ifndef _exImageAddr_
#define _exImageAddr_
typedef struct _exImageAddr_
{
	unsigned long long ulPhyAddr[4];
	unsigned long long ulVirAddr[4];
	unsigned int	   uFrameId;
	unsigned int	   uReserved;	
}exImageAddr;
#endif
/*
* Function Name: slotDetect_Init 
* Function: slot detect system initial
* Input Param: null
* Output Param: null
* Return value: true:success，false:failure
*/
int slotDetect_Init(); //system init,true-sucess,false-failure

/*
* Function Name: slotDetect_UnInit 
* Function: slot detect system uninitial
* Input Param: null
* Output Param: null
* Return value: true:success，false:failure
*/
int slotDetect_UnInit(); //system uninit

/*
* Function Name: SetTouchPoint 
* Function: input the touchpoint value
* Input Param: x:x value;y:y value;type:0-press,1-release,only response when release;
* The screen should be 1920*1080;
* Output Param: null
* Return value: true:success，false:failure
*/
int SetTouchPoint(int x,int y,int type);

/*
* Function Name: SetParkingStatu
* Function: input the parking statu
* Input Param: 
		iParkingType:0：no avail 1：park in 2：park out;3:searching slot;
		iParkingFlag:0：parking in/parking out 1：sucess 2：slot position not meet 3：plan failure 
			4：slot too small 5：meet obstacle stop;
		iParkingStepSize:parking in/out total steps,value = 5;
		iParkingStep:parking in/out current,value <= 5;
* Output Param: null
* Return value: true:success，false:failure
*/
int SetParkingStatus(int iParkingType,int iParkingFlag,int iParkingStepSize,int iParkingStep);

/*
* Function Name: PushCanInfo 
* Function: push can parse result
* Input Param: tInfo:can parse result
* Output Param: null
* Return value: true:success，false:failure
*/
int PushCanInfo(CarInfo &tInfo);

/*
* Function Name: GetSlotResult 
* Function: get slot result,include detect slot and manage slot
* Input Param: null
* Output Param: tSlot
* Return value: true:success，false:failure
*/
int GetSlotResult(exSlotResult &tSlot);

/*
* Function Name: GetFreespaceResult 
* Function: get freespace result
* Input Param: null
* Output Param: tFreespace
* Return value: true:success，false:failure
*/
int GetFreespaceResult(exFreeSpaceRegionInfo &tFreespace);

/*
* Function Name: MvStartCnn 
* Function: Start cnn caculation
* Input Param: null
* Output Param: null
* Return value: true:success，false:failure
*/
int MvStartCnn();

/*
* Function Name: GetImageBufferAddr 
* Function: Get the shared memory address of image
* Input Param: null
* Output Param: ulLeft,ulRight,ulRear,ulFront are four camera's address
* Return value: 0:success，false:other value
*/
int GetImageBufferAddr(exImageAddr &tAddr);

/*
* Function Name: GetBv2dImageAddr 
* Function: Get the shared memory address of birdview image
* Input Param: null
* Output Param: null
* Return value: image addr in inner space
*/
unsigned long long GetBv2dImageAddr();

/*
* Function Name: GetCnnFinishFlag 
* Function: Get cnn finish flag
* Input Param: null
* Output Param: null
* Return value: false:processing;true:finished
*/
int GetCnnFinishFlag();

//Disable slot handler function
void DisableSlot();

//Disable freespace handler function
void DisableFreespace();

//Set select slot id
void SetParkSlotId(int nId);

void SetWorkMode(int iMode);
