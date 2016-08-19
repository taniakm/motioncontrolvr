#include "InsertionDevice.h"
#include "qstring.h"
#include "qdebug.h"
#include  <math.h>


bool InsertionDevice::m_created = false;

/// \brief		Constructor: Initializes roll device ID and sets current and position limits.	
//InsertionDevice::InsertionDevice(void) : 
InsertionDevice::InsertionDevice(DeviceParams params) : 
MotionController(params.deviceID),
	m_currentMM(NULL),
	m_isInsertionLocked(false)
{
	if(m_created)
	{
		qDebug() << "ERROR: Only one instance of InsertionDevice is allowed";
	}

	m_created = true;

	devParams = params;
}

/// \brief		Destructor: Stops motion if there happens to be any	
InsertionDevice::~InsertionDevice(void)
{
	MotionController::Stop();
	m_isInsertionLocked = false;
	m_created = false;
}

/// \brief		Initialize MCDC with comPort param and set limits of MCDC
///	\pre		comPort is a valid port number and the roll device is connected to that port
///	\post		The com port is initialized and limits are set
void InsertionDevice::Init(std::string comPort, DeviceParams params)
//void InsertionDevice::Init(QString comPort, DeviceParams params)
{
	/*motionContInsert = new MotionController(params.deviceID);
	motionContInsert->InitializeDevice(comPort);*/
	MotionController::InitializeDevice(comPort);
	MotionController::SetCorridorValue(params.deviceCorridor);
}

/// \brief		Re-enable insertion device motor 
/// \pre		Insertion device is still connected
///	\post		Limits are set again and the MCDC is re-enabled
void InsertionDevice::ReEnable(DeviceParams params)
{
	SetLimits(params);						//Set limits again just to be safe
	MotionController::EnableDevice();	//"EN" command
	MotionController::ResetEncoder();	//Reset encoder ticks to zero
}

///	\brief		Set Limits on Device
/// \pre		MCDC is still connected
/// \post		Current and Position Limits are Set
void InsertionDevice::SetLimits(DeviceParams params)
{
	SetPeakCurrentLimit(params.currentLimit);
	//SetPositionLimits(MIN_ENCODER_LIMIT, MAX_ENCODER_LIMIT);
	//EnablePositionLimits(true);
}

/// \brief		Change max travelling velocity back to default value
void InsertionDevice::SetDefaultVelocity(DeviceParams params)
{
	MotionController::ChangeMaxMotorVelocity(params.maxVelocity);
}

/// \brief		Convert angle (in degrees) into encoder ticks for the motion controller
/// \pre		MM is a valid linear slide position in mm
/// \post		The postion (in encoder ticks) is returned 
int InsertionDevice::ConvertMMToPosition(float MM, DeviceParams params)
{
	//float pos = MM * (float)micromoTicksPerRev / slidePitch * spurGearRatio;
	float pos = MM * (float)(params.micromoTicksPerRev / params.mmPerRev);
	return (int)(pos+0.5);	//Return rounded-up int value of position
}

/// \brief		Convert position, in encoder ticks, to its associated angle (in degrees)
/// \pre		Position is in encoder ticks and is a valid number from the MCDC
/// \post		The linear slide position (in mm) is returned 
float InsertionDevice::ConvertPositionToMM(long position, DeviceParams params)
{
	//float MM = ( (float)position / micromoTicksPerRev ) * slidePitch / spurGearRatio;
	float MM = (float)position / (params.micromoTicksPerRev / params.mmPerRev);
	return MM;
}

/// \brief		Get the current angle of the roll motor
///	\details	Get the encoder position and then covert it to an angle
/// \pre		MCDC is still connected
///	\post		Current angle of the roll motor is returned
float InsertionDevice::GetMM(DeviceParams params)
{
	long currentPos = this->GetPosition();
	m_currentMM = ConvertPositionToMM( currentPos, params);
	return m_currentMM;
}

/// \brief		Get the lead screw pitch
///	\details	Gives the number of mm travelled during one rotation of the insertion motor
/// \pre		None
///	\post		Leadscrew pitch is returned
//float InsertionDevice::GetPitch()
//{
//	return slidePitch/gearRatio;
//}

