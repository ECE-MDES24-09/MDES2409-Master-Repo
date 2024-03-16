#ifndef Lidar_h
#define Lidar_h

#include <TFMPlus.h>
#include "printf.h"
#include <globalVariables.h>
//enum SerialSelect {
//	S0,
//	S1,
//	S2,
//	S3
//};

class Lidar {

public:
	init(SerialSelect SS)
	int16_t getDist()
private:
	//// Initialize variables
	TFMPlus tfmP;
	int16_t tfDist = 0;    // Distance to object in centimeters
	int16_t tfFlux = 0;    // Strength or quality of return signal
	int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
};