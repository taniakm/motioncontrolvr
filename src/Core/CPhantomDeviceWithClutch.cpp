#include "CPhantomDeviceWithClutch.h"

namespace chai3d {

	cPhantomDeviceWithClutch::cPhantomDeviceWithClutch(unsigned int a_deviceNumber)
		: cPhantomDevice(a_deviceNumber)
		, m_clutchMode(CM_NORMAL)
	{
		m_clutchOffset.identity();
	}



	//! Read position and orientation of haptic device. Results is passed through a transformation matrix (4x4).
	bool cPhantomDeviceWithClutch::getTransform(cTransform& a_transform)
	{
		bool res;
		switch(m_clutchMode) {
		case CM_NORMAL:
			res = cPhantomDevice::getTransform(a_transform);
			a_transform = m_clutchOffset*a_transform;
			break;
		case CM_PRESSED:
			a_transform = m_clutchOffset;
			res = true;
			break;
		}

		return res;
	}

	//! Read the position of the device. Units are meters [m].
	bool cPhantomDeviceWithClutch::getPosition(cVector3d& a_position)
	{
		bool res;
		switch(m_clutchMode) {
		case CM_NORMAL:
			res = cPhantomDevice::getPosition(a_position);
			a_position = m_clutchOffset.getLocalPos() + a_position;
			break;
		case CM_PRESSED:
			a_position = m_clutchOffset.getLocalPos();
			res = true;
			break;
		}

		return res;
	}

	//! Read orientation frame (3x3 matrix) of the haptic device end-effector.
	bool cPhantomDeviceWithClutch::getRotation(cMatrix3d& a_rotation)
	{
		bool res;
		switch(m_clutchMode) {
		case CM_NORMAL:
			res = cPhantomDevice::getRotation(a_rotation);
			a_rotation = m_clutchOffset.getLocalRot()*a_rotation;
			break;
		case CM_PRESSED:
			a_rotation = m_clutchOffset.getLocalRot();
			res = true;
			break;
		}

		return res;
	}

	

	void cPhantomDeviceWithClutch::clutchPressed() 
	{ 
		switch(m_clutchMode) {
		case CM_NORMAL: 
			{
				cTransform cOffset; getTransform(cOffset); 
				cVector3d posOffset; getPosition(posOffset);
				cOffset.setLocalPos(posOffset);
				m_clutchOffset = cOffset;
				m_clutchMode = CM_PRESSED;
				break;
			}
		case CM_PRESSED:
			break;
		}
	}

	void cPhantomDeviceWithClutch::releaseClutch()
	{
		switch(m_clutchMode) {
		case CM_NORMAL:
			break;
		case CM_PRESSED:
			cTransform current; cPhantomDevice::getTransform(current);
			cVector3d currPos; cPhantomDevice::getPosition(currPos);
			current.setLocalPos(currPos);
			cTransform currInv = current; currInv.invert();
			cTransform delta = m_clutchOffset*currInv;
			m_clutchOffset = delta;
			m_clutchMode = CM_NORMAL;
			break;
		}
	}
}