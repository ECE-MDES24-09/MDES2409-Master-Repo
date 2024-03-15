#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#define MAX_ANGLE 70.0
#define SPEED_MULTIPLIER 1.0
#define ROLLER_MULTIPLIER 1.0
#define STORAGE_SERVO_PIN 12

#include "Arduino.h"
#include <Servo.h>
#include <motor_driver.h>
#include <pixy_line_detection.h>
#include <serial_communication.h>

#include <gyro_blue.h>
#include <TMP.h>



// Counter and compare values
const uint16_t t1_load = 0;
const uint16_t t1_comp = 250;

enum class RollerState
{
	InTake,
	OutTake,
	Stop,
};

enum class StorageState
{
	Start,
	PickUp,
	BigCubeDrop,
	SmallCubeDrop,
	FuelDrop,
	Cruise,
};

class RobotControl {
	

public:

	MotorDriver motorDriver;
	TMP TMPFront;
	Gyro Gyro;

	const int sensorPin = A12; 
	
	void init();
	
	int ColorSensor();
	int GetDist();
	int GetAngle();
	void test();
	void Turn(float Turn)
	void FollowHeading(float Direction);
	void lineFollow(int robotSpeed, double targetOffset);

	void manualControl();

	void connectionCheck();
	void updateSerialInput();

	// InTake
	void HandleSquareButtonPress();

	// OutTake
	void HandleCrossButtonPress();


	// Storage tilting control
	void HandleTriangleButtonPress();


	void printStates();

	void servo_write();

private:

	PixyLineDetect lineDetect;

	int8_t dataRecieved[DATA_LENGTH];

	RollerState rollersState = RollerState::Stop;
	StorageState storageState = StorageState::Start;

	float targetStorageAng = 90.0f;
	float storageAng = 0.0f;
	float storageRotateSpeed = 0.1f; // degree per loop
	float bigCubeDropAng = 115.0f;
	float smallCubeDropAng = 115.0f;
	float pickUpAng = 115.0f;
	float crusingAng = 90.0f;
	float fuelDropAng = 75.0f;
	float startAng = 90.0f;

	int8_t lastDataReceived2 = -1; // To track the last state of dataRecieved[2]
	int8_t lastDataReceived3 = -1; // To track the last state of dataRecieved[3]
	int8_t lastDataReceived4 = -1; // To track the last state of dataRecieved[4]
	
	Servo myservo;

	int storageCycleStep = 0;

};


#endif