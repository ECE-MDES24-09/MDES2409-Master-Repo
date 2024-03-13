#ifndef MANIPULATOR_CONTROL_H
#define MANIPULATOR_CONTROL_H

#include "Arduino.h"
#include <Servo.h>

class Manipulator_Control {
public:
    void init();
    void setStorageAngle(float angle, float speedFactor); // Function to set storage angle with speed control
    void setLeftRollerSpeed(int speed); // Function to set left roller speed
    void setRightRollerSpeed(int speed); // Function to set right roller speed

private:
    const int STORAGE_SERVO_PIN = 12;
    const int LEFT_ROLLER_SERVO_PIN = 10; // Example pin for left roller servo
    const int RIGHT_ROLLER_SERVO_PIN = 11; // Example pin for right roller servo
    Servo storageServo;
    Servo leftRollerServo;
    Servo rightRollerServo;

    float currentStorageAngle; // Current angle of the storage servo
    float targetStorageAngle; // Target angle for the storage servo

    const float bigCubeDropAng = 115.0f;
    const float smallCubeDropAng = 115.0f;
    const float pickUpAng = 115.0f;
    const float crusingAng = 90.0f;
    const float fuelDropAng = 75.0f;
    const float startAng = 90.0f;

    // Function to update the storage servo angle gradually
    void updateStorageAngle(float speedFactor);
};

#endif
