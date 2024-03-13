#include "Manipulator_Control.h"

// Initialize the manipulator control (attach servos)
void Manipulator_Control::init() 
{
    storageServo.attach(STORAGE_SERVO_PIN);
    leftRollerServo.attach(LEFT_ROLLER_SERVO_PIN);
    rightRollerServo.attach(RIGHT_ROLLER_SERVO_PIN);
}

// Move the storage servo to pick up position
void Manipulator_Control::pickUp() 
{
    storageServo.write(pickUpAng);
}

// Move the storage servo to drop big cube position
void Manipulator_Control::dropBigCube() 
{
    storageServo.write(bigCubeDropAng);
}

// Move the storage servo to drop small cube position
void Manipulator_Control::dropSmallCube() 
{
    storageServo.write(smallCubeDropAng);
}

// Move the storage servo to drop fuel position
void Manipulator_Control::dropFuel() 
{
    storageServo.write(fuelDropAng);
}

// Move the storage servo to cruising position
void Manipulator_Control::cruise() 
{
    storageServo.write(crusingAng);
}

// Set speed for the left roller servo
void Manipulator_Control::setLeftRollerSpeed(int speed) 
{
    leftRollerServo.writeMicroseconds(map(speed, -100, 100, 1000, 2000)); // Assuming speed range is -100 to 100
}

// Set speed for the right roller servo
void Manipulator_Control::setRightRollerSpeed(int speed) 
{
    rightRollerServo.writeMicroseconds(map(speed, -100, 100, 2000, 1000)); // Assuming speed range is -100 to 100
}
