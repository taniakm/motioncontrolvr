#include "CTRControl.h"


// ------------- Hardware specific parameters -------------
static DeviceParams g_deviceParams[] = {
	//deviceID		//deviceCorridor	//micromoTicksPerRev	//spurGearRatio		//mmPerRev		//minLimit		//maxLimit		//currentLimit		//maxVelocity
	{0,				20,					11836.92,				1,					37.7,			-300000,		300000,			0.2,					10}, // max velocity was 2000 for all
	{0,				20,					11836.92,				1,					1.0,			-300000,		300000,			0.2,					10},
	{0,				20,					11836.92,				1,					66.0,			-300000,		300000,			0.2,					10},
	{0,				20,					11836.92,				1,					1,				-300000,		300000,			0.2,					10},
	{0,				20,					11836.92,				0.88,				77.85,			-300000,		300000,			0.2,					10},
	{0,				20,					11836.92,				0.88,				1,				-300000,		300000,			0.2,					10},
};

static std::string g_comPorts[] = {"\\\\.\\COM44", "\\\\.\\COM45", "\\\\.\\COM46", "\\\\.\\COM47", "\\\\.\\COM48", "\\\\.\\COM49"};
// -------------------------------------------------------


CTRControl::CTRControl(void)
{
}


CTRControl::~CTRControl(void)
{
}


void CTRControl::Init() {
	for(int i=0; i<NUM_TUBES; i++) {
		this->m_tubeControllers[i].Init(g_comPorts[2*i],g_deviceParams[2*i],g_comPorts[2*i+1],g_deviceParams[2*i+1]);
	}
	//this->m_tubeControllers[0].Init(g_comPorts[0],g_deviceParams[0],g_comPorts[1],g_deviceParams[1]);
	//this->m_tubeControllers[0].Init(g_comPorts[1],g_deviceParams[1],g_comPorts[0],g_deviceParams[0]);
}

