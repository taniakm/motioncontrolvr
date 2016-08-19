/// class		InsertionDevice
/// brief		Encapsulates the functionality of the insertion stage. 
///				Inherits functionality from MotionController class.
///
///	author		Troy Adebar
///	date		March 25, 2013


#pragma once

#include "MotionController.h"
#include "DeviceParameters.h"

class InsertionDevice :	public MotionController
{
public: 
	//Methods
	//InsertionDevice(void);
	InsertionDevice(DeviceParams params);
	~InsertionDevice(void);
	void Init(std::string comPort, DeviceParams params);
	//void Init(QString comPort, DeviceParams params);
	void ReEnable(DeviceParams params);
	int ConvertMMToPosition(float angle, DeviceParams params);
	float ConvertPositionToMM(long position, DeviceParams params);
	float GetMM(DeviceParams params);	
	void SetDefaultVelocity(DeviceParams params);
	//float GetPitch(void);

	DeviceParams devParams;
	//MotionController *motionContInsert;
	
private:	//Methods
	void SetLimits(DeviceParams params);

private:	//Attributes
	static bool m_created;
	float m_min;
	float m_max;
	float m_incr;
	float m_currentMM;
	bool m_isInsertionLocked;
};
