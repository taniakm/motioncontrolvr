#include "RollDevice.h"
//#include "Utilities.h"
#include  <math.h>
#include "qstring.h"
#include "qdebug.h"


bool RollDevice::m_created = false;

/// \brief		Constructor: Initializes roll device ID and sets current and position limits.	
RollDevice::RollDevice(DeviceParams params) : 
MotionController(params.deviceID),
m_currentAngle(NULL),
m_isRollerLocked(false)
{
	if(m_created)
	{
		qDebug() << "ERROR: Only one instance of RollDevice is allowed";
	}

	m_created = true;
	
	devParams = params;
}

/// \brief		Destructor: Stops motion if there happens to be any	
RollDevice::~RollDevice(void)
{
	MotionController::Stop();	
	m_isRollerLocked = false;
	m_created = false;
}

/// \brief		Initialize MCDC with comPort param and set limits of MCDC
///	\pre		comPort is a valid port number and the roll device is connected to that port
///	\post		The com port is initialized and limits are set
void RollDevice::Init(std::string comPort, DeviceParams params)
//void RollDevice::Init(QString comPort, DeviceParams params)
{
	/*motionContDev = new MotionController(params.deviceID);
	motionContDev->InitializeDevice(comPort);*/
	
	MotionController::InitializeDevice(comPort);
	MotionController::SetCorridorValue(params.deviceCorridor);
	//MotionController::ClearExistingData();

}

/// \brief		Re-enable roll device motor 
/// \pre		Roll device is still connected
///	\post		Limits are set again and the MCDC is re-enabled
void RollDevice::ReEnable(DeviceParams params)
{
	SetLimits(params);						//Set limits again just to be safe
	MotionController::EnableDevice();	//"EN" command
	MotionController::ResetEncoder();	//Reset encoder ticks to zero
}

///	\brief		Set Limits on Device
/// \pre		MCDC is still connected
/// \post		Current and Position Limits are Set
void RollDevice::SetLimits(DeviceParams params)
{
	SetPeakCurrentLimit(params.currentLimit);
	//SetPositionLimits( ConvertAngleToPosition(params.minLimit, params),
	//	ConvertAngleToPosition(params.maxLimit, params) );
	//EnablePositionLimits(true);
}

/// \brief		Change max travelling velocity back to default value
void RollDevice::SetDefaultVelocity(DeviceParams params)
{
	MotionController::ChangeMaxMotorVelocity(params.maxVelocity);
}

/// \brief		Convert angle (in degrees) into encoder ticks for the motion controller
/// \pre		Angle is a valid angle for the roll device
/// \post		The postion (in encoder ticks) is returned 
int RollDevice::ConvertAngleToPosition(float angle, DeviceParams params)
{
	float pos = angle * (float)params.micromoTicksPerRev * (float)params.spurGearRatio / 360.0;
	return (int)(pos+0.5);	//Return rounded-up int value of position
}

/// \brief		Convert position, in encoder ticks, to its associated angle (in degrees)
/// \pre		Position is in encoder ticks and is a valid number from the MCDC
/// \post		The angle (in degrees) is returned 
float RollDevice::ConvertPositionToAngle(long position, DeviceParams params)
{
	float angle = ( (float)position * 360.0 ) / ( params.micromoTicksPerRev * params.spurGearRatio);
	return angle;
}

/// \brief		Get the current angle of the roll motor
///	\details	Get the encoder position and then covert it to an angle
/// \pre		MCDC is still connected
///	\post		Current angle of the roll motor is returned
float RollDevice::GetAngle(DeviceParams params)
{
	m_currentAngle = ConvertPositionToAngle( this->GetPosition(), params);
	return m_currentAngle;
}

/// \brief		Set the digital output of the MCDC
///	\details	Sets the FAULT pin to either high-impedance or GND
/// \pre		MCDC DIGOUT is set to true
///	\post		FAULT output pin is set based on OnOff
void RollDevice::setVibration(bool OnOff)
{
	MotionController::setDigitalOutput(OnOff);
}