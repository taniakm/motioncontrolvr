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

	void Init(QString comPort_insertDev, DeviceParams params_insertDev, QString comPort_rollDev, DeviceParams params_rollDev);

};

