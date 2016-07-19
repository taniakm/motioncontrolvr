#pragma once
#include "devices/CGenericHapticDevice.h"
#include "devices/CPhantomDevices.h"

namespace chai3d {

	enum ClutchMode
	{
		CM_NORMAL = 0,
		CM_PRESSED,
	};

	class cPhantomDeviceWithClutch : public cPhantomDevice
	{
	protected:
		cTransform m_clutchOffset;
		ClutchMode m_clutchMode;

	public:
		cPhantomDeviceWithClutch(unsigned int a_deviceNumber = 0);

		//! Read position and orientation of haptic device. Results is passed through a transformation matrix (4x4).
		virtual bool getTransform(cTransform& a_transform);

		//! Read the position of the device. Units are meters [m].
		virtual bool getPosition(cVector3d& a_position);

		//! Read orientation frame (3x3 matrix) of the haptic device end-effector.
		virtual bool getRotation(cMatrix3d& a_rotation);

		virtual void clutchPressed();
		
		virtual void releaseClutch();
	};
}

