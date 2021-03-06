#include "TubeControl.h"


TubeControl::TubeControl(void)
{
	
}


TubeControl::~TubeControl(void)
{

}


void TubeControl::Init(std::string comPort_insertDev, DeviceParams params_insertDev, std::string comPort_rollDev, DeviceParams params_rollDev) 
//void TubeControl::Init(QString comPort_insertDev, DeviceParams params_insertDev, QString comPort_rollDev, DeviceParams params_rollDev) 
{
	transController = new InsertionDevice(params_insertDev);
	transController->Init(comPort_insertDev, params_insertDev);

	rotController = new RollDevice(params_rollDev);
	rotController->Init(comPort_rollDev, params_rollDev);

	/*transController2 = new InsertionDevice(params_rollDev);
	transController2->Init(comPort_rollDev, params_rollDev);*/

}