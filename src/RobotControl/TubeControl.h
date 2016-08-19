#pragma once

#include "InsertionDevice.h"
#include "RollDevice.h"

class TubeControl
{
public:
	TubeControl(void);
	~TubeControl(void);

	InsertionDevice *transController;
	RollDevice *rotController;
	InsertionDevice *transController2;

	//void Init(QString comPort_insertDev, DeviceParams params_insertDev, QString comPort_rollDev, DeviceParams params_rollDev);
	void Init(std::string comPort_insertDev, DeviceParams params_insertDev, std::string comPort_rollDev, DeviceParams params_rollDev);

};

