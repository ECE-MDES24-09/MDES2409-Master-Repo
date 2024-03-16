#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "Arduino.h"
#include <Servo.h>
#include <MotorDriver.h>
#include <pixy_line_detection.h>
#include <serial_communication.h>
//#include <gyro_blue.h>
//#include <TMP.h>
#include "Manipulator_Control.h"
#include <globalVariables.h>

class RobotControl {
public:
	MotorDriver motorDriver;
	//TMP TMPFront;
	//Gyro Gyro;
	
	void init();
	
	int ColorSensor();
	int GetDist();
	int GetAngle();
	void test();
	void turn(float Turn);
	void followHeading(float Direction);
	void lineFollow(int robotSpeed, double targetOffset);
	void crossGap();


	void connectionCheck();
	void updateSerialInput();
	void rollersIntake();
	void rollersDispense();
	void rollersStop();
	void rocketDrop();
	void cruisin();

	/**

		Code for Manual Control.

	**/
	/**
	void manualControl();
	// InTake
	void HandleSquareButtonPress();

	// OutTake
	void HandleCrossButtonPress();


	// Storage tilting control
	void HandleTriangleButtonPress();


	void printStates();
	**/
	void servo_write();

private:
	PixyLineDetect lineDetect;
    Manipulator_Control manipulatorControl;

	const int RC_STORAGE_SERVO_PIN = STORAGE_SERVO_PIN;

	Servo myservo;
};


#endif