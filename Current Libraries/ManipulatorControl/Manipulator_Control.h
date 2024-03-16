#ifndef MANIPULATOR_CONTROL_H
#define MANIPULATOR_CONTROL_H

#include "Arduino.h"
#include <Servo.h>
#include <globalVariables.h>
class Manipulator_Control {
public:
    void init();
    void setStorageAngle(float angle, float speedFactor); // Function to set storage angle with speed control
    void setLeftRollerSpeed(int speed); // Function to set left roller speed
    void setRightRollerSpeed(int speed); // Function to set right roller speed

private:
    Servo storageServo;
    Servo leftRollerServo;
    Servo rightRollerServo;

    // Function to update the storage servo angle gradually
    void updateStorageAngle(float speedFactor);
};

#endif
