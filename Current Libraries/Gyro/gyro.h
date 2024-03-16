#ifndef Gyro_h
#define Gyro_h

#include "ICM_20948.h"
#include "globalVariables.h"

#ifdef USE_SPI
extern ICM_20948_SPI myICM; // Declare as extern to avoid multiple definitions
#else
extern ICM_20948_I2C myICM; // Declare as extern
#endif

class Gyro {
public:
	void gyro_init();
	float getGyroAng();
};
#endif
