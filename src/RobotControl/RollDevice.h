/// class:		RollDevice
/// brief:		Encapsulates the functionality of the roll device. 
///				Inherits functionality from MotionController class.
///
///	author:		Troy Adebar
///	date:		March 25, 2013


#pragma once

#include "MotionController.h"
#include "DeviceParameters.h"

class RollDevice :	public MotionController
{
public: 
	//Methods
	//RollDevice(void);
	RollDevice(DeviceParams params);
	~RollDevice(void);
	void Init(std::string comPort, DeviceParams params);
	//void Init(QString comPort, DeviceParams params);
	void ReEnable(DeviceParams params);
	int ConvertAngleToPosition(float angle, DeviceParams params);
	float ConvertPositionToAngle(long position, DeviceParams params);
	float GetAngle(DeviceParams params);	
	void SetDefaultVelocity(DeviceParams params);
	void setVibration(bool);

	DeviceParams devParams;
	//MotionController *motionContDev;

private:	//Methods
	void SetLimits(DeviceParams params);

private:	//Attributes
	static bool m_created;
	float m_min;
	float m_max;
	float m_incr;
	float m_currentAngle;
	bool m_isRollerLocked;
};
