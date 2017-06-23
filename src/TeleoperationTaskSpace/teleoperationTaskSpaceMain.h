#include "chai3d.h"
#include "GL/glut.h"
#include "ConcentricTubeSet.h"
#include "CPhantomDeviceWithClutch.h"
#include "CTRControl.h"

int tubeNum;
int useDefaultSetBool;

CTRControl robot;

struct teleopDataStruct {
	double				time;				// current time in program
	float				alpha0;
	float				alpha1;
	float				alpha2;	
	float				Beta0;
	float				Beta1;
	float				Beta2;
};

vector<teleopDataStruct> data;					// variable to store data in each time step