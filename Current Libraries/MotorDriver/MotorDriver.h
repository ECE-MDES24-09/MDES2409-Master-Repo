#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "Arduino.h"
#include <globalVariables.h>
class MotorDriver {

public:

	void setSpeed(int targetLeftSpeed, int targetRightSpeed);
	void startMove();
	void FollowLine(int right, int left, double angle, double offset);

private:

};

#endif

