#pragma once


struct DeviceParams {
	int deviceID;							// Network ID of the insertion MCDC	
	int deviceCorridor;						// Default corridor value
	float micromoTicksPerRev;				// # of encoder ticks in one full rotation (without factoring in the gear ratio) 
	float spurGearRatio;					// Gear ratio for micromo motors
	float mmPerRev;							// MM travelled for one revolution of motor (***FOR INSERTION DEVICE ONLY)
	float minLimit;							// Minimum encoder limit that the motor will not be able to travel past
	float maxLimit;							// Maximum encoder limit that the motor will not be able to travel past
	float currentLimit;						// motor peak current limit in mA
	float maxVelocity;						// Insertion device velocity when it is not moving in no increments mode
};