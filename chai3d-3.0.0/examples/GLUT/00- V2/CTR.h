#include "chai3d.h"
#include "Leap.h"
#include "ConcentricTubeSet.h"
#include "SerialClass.h"
#include "gsl/gsl_linalg.h"

//------------------------------------------------------------------------------
using namespace chai3d;
using namespace Leap;
using namespace std;

//------------------------------------------------------------------------------

struct LeftSkeletonFinger {
	cVector3d		leftBoneStart[4];
	cVector3d		leftBoneEnd[4];
};

struct RightSkeletonFinger {
	cVector3d		rightBoneStart[4];
	cVector3d		rightBoneEnd[4];
};

struct LeftSkeletonHand {
	LeftSkeletonFinger		leftSkeletonFinger[5];
};

struct RightSkeletonHand {
	RightSkeletonFinger		rightSkeletonFinger[5];
};

struct LeftJointSpheres {
	cShapeSphere*       leftJointSphere[25];
};

struct RightJointSpheres {
	cShapeSphere*       rightJointSphere[25];
};

struct LeftFingerLines {
	cShapeLine*		leftFingerLine[25];
};

struct RightFingerLines {
	cShapeLine*		rightFingerLine[25];
};

struct Tube {
	cShapeSphere*		tubeSphere[100000];
};

struct dataStruct {							// struct to store data at a given instant
	double				time;				// current time in program	
	cVector3d			devicePos;			// position of the device
	cMatrix3d			deviceRot;			// rotation of the device
	cVector3d			CTRPos;				// position of base of CTR
	cMatrix3d			CTRRot;				// rotation of CTR
	cVector3d			modelPos;			// position of model
	cMatrix3d			modelRot;			// rotation of model
	int					pedalPressed;		// (1=clutch, 2=camera, 3=zoom in, 4= zoom out, 5=orient, 6=sim)
	int					button0Pressed;		// 1 if pressed, 0 if not
	int					button1Pressed;		// 1 if pressed, 0 if not
	
	float				OD0;
	float				ID0;
	float				kappa0;
	float				alpha0;
	float				Beta0;
	float				Lc0;
	float				Ls0;
	float				OD1;
	float				ID1;
	float				kappa1;
	float				alpha1;
	float				Beta1;
	float				Lc1;
	float				Ls1;
	float				OD2;
	float				ID2;
	float				kappa2;
	float				alpha2;
	float				Beta2;
	float				Lc2;
	float				Ls2;

};

vector<dataStruct> data;					// variable to store data in each time step