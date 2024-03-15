#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

// motor pins
#define RIGHT_FORWARD_PIN 5
#define RIGHT_BACKWARD_PIN 4
#define LEFT_FORWARD_PIN 3
#define LEFT_BACKWARD_PIN 2

#define RAMP_RATE 2 

#include "Arduino.h"

class MotorDriver {

public:

	void setSpeed(int targetLeftSpeed, int targetRightSpeed) {
		this->targetLeftSpeed = targetLeftSpeed;
		this->targetRightSpeed = targetRightSpeed;
	}

	void startMove() {
		
		// ramping
		if (targetRightSpeed > this->rightSpeed)
			this->rightSpeed += RAMP_RATE;
		else if (targetRightSpeed < this->rightSpeed)
			this->rightSpeed -= RAMP_RATE;

		if (targetLeftSpeed > this->leftSpeed)
			this->leftSpeed += RAMP_RATE;
		else if (targetLeftSpeed < this->leftSpeed)
			this->leftSpeed -= RAMP_RATE;

		// output PWMs
		if (rightSpeed >= 0){
			analogWrite(RIGHT_FORWARD_PIN, rightSpeed);
			analogWrite(RIGHT_BACKWARD_PIN, 0);
		}
		else{
			analogWrite(RIGHT_FORWARD_PIN, 0);
			analogWrite(RIGHT_BACKWARD_PIN, -rightSpeed);
		}

		if (leftSpeed >= 0){
			analogWrite(LEFT_FORWARD_PIN, leftSpeed);
			analogWrite(LEFT_BACKWARD_PIN, 0);
		}
		else{
			analogWrite(LEFT_FORWARD_PIN, 0);
			analogWrite(LEFT_BACKWARD_PIN, -leftSpeed);
		}

		delay(1);  // I don't like this. don't use delay, possible solution: timer, multi-threading (unlikely)

	}
	

	
		void FollowLine(int right, int left, double angle, double offset) {
		
		double ramp = .01;
		if (offset < 5) {
			offset = offset * 2;
		}
		OS = 100 * (angle) / 90 - (offset);
		double Right = right + OS;
		double Left = left - OS;
		
		if(Right > 240){
			Right = 240;
		}
		if(Left > 240){
			Left = 240;
		}
		
		analogWrite(RIGHT_FORWARD_PIN, Right);
		analogWrite(LEFT_FORWARD_PIN, Left);

	}

private:

	int OS = 0;
	
	int leftSpeed = 0;
	int rightSpeed = 0;

	int targetLeftSpeed = 0;
	int targetRightSpeed = 0;

};

#endif

