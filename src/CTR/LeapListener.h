#pragma once
//#include "c:\users\tania\documents\motioncontrol\chai3d-3.0.0\external\leapsdk\include\leap.h"
#include "Leap.h"
#include "chai3d.h"

using namespace chai3d;
using namespace Leap;

class LeapListener :
	public Listener
{
public:
	LeapListener(void);
	~LeapListener(void);

	virtual void onInit(const Controller&);
	virtual void onConnect(const Controller&);
	virtual void onFrame(const Controller&);

	Frame			frame;
	HandList		pHandList;
	Hand			pHand;
	int			    handID;
	int				handCount;
	Vector			handCenterTemp;
	cVector3d		handCenter;
	FingerList		pFingerList;
	Finger			pFinger;
	int				fingerCount;

	cMatrix3d		camToOculusRot;
	//cShapeSphere*	handCenterSphere;						// a small sphere representing the center of the hand
	
};

