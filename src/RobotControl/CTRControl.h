#pragma once

#include "TubeControl.h"


#define NUM_TUBES 3

using namespace std;

class CTRControl
{
public:
	CTRControl(void);
	~CTRControl(void);

	TubeControl m_tubeControllers[NUM_TUBES];

	void Init();
};

