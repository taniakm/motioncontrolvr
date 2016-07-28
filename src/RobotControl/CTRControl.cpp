#include "CTRControl.h"


// ------------- Hardware specific parameters -------------
static DeviceParams g_deviceParams[] = {
	//deviceID		//deviceCorridor	//micromoTicksPerRev	//spurGearRatio		//mmPerRev		//minLimit		//maxLimit		//currentLimit		//maxVelocity
	{1,				20,					11836.92,				1,					37.7,			-300000,		300000,			2,					2},
	{2,				20,					11836.92,				1,					1,				-300000,		300000,			2,					2},
	{3,				20,					11836.92,				1,					66.0,			-300000,		300000,			2,					2},
	{4,				20,					11836.92,				1,					1,				-300000,		300000,			2,					2},
	{5,				20,					11836.92,				0.88,				77.85,			-300000,		300000,			2,					2},
	{6,				20,					11836.92,				1,					1,				-300000,		300000,			2,					2},
};

static QString g_comPorts[] = {1,2,3,4,5,6};
// -------------------------------------------------------


CTRControl::CTRControl(void)
{
}


CTRControl::~CTRControl(void)
{
}


void Init() {
	CTRControl robot;
	for(int i=0; i<NUM_TUBES; i++) {
		robot.m_tubeControllers[i].Init(g_comPorts[2*i],g_deviceParams[2*i],g_comPorts[2*i+1],g_deviceParams[2*i+1]);
	}
}

