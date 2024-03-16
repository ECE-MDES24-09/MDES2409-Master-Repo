//
// Created by jorda on 03/15/2024.
//
#include "MotorDriver.h"

<<<<<<< HEAD
void MotorDriver::setSpeed(int targetLS, int targetRS) {
    targetLeftSpeed = targetLS;
    targetRightSpeed = targetRS;
=======
void MotorDriver::setSpeed(int targetLeftSpeed, int targetRightSpeed) {
    this->targetLeftSpeed = targetLeftSpeed;
    this->targetRightSpeed = targetRightSpeed;
>>>>>>> 1b510070ff16c63498ab9274048c425d8414488c
}

void MotorDriver::startMove() {

    // ramping
<<<<<<< HEAD
    if (targetRightSpeed > rightSpeed)
        rightSpeed += RAMP_RATE;
    else if (targetRightSpeed < rightSpeed)
        rightSpeed -= RAMP_RATE;

    if (targetLeftSpeed > leftSpeed)
        leftSpeed += RAMP_RATE;
    else if (targetLeftSpeed < leftSpeed)
        leftSpeed -= RAMP_RATE;
=======
    if (targetRightSpeed > this->rightSpeed)
        this->rightSpeed += RAMP_RATE;
    else if (targetRightSpeed < this->rightSpeed)
        this->rightSpeed -= RAMP_RATE;

    if (targetLeftSpeed > this->leftSpeed)
        this->leftSpeed += RAMP_RATE;
    else if (targetLeftSpeed < this->leftSpeed)
        this->leftSpeed -= RAMP_RATE;
>>>>>>> 1b510070ff16c63498ab9274048c425d8414488c

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

void MotorDriver::FollowLine(int right, int left, double angle, double offset) {
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