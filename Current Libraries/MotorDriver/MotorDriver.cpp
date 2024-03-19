//
// Created by jorda on 03/15/2024.
//
#include "MotorDriver.h"

void MotorDriver::init() {
    pinMode(RIGHT_FORWARD_PIN, INPUT);
    pinMode(RIGHT_BACKWARD_PIN, INPUT);
    pinMode(LEFT_FORWARD_PIN, INPUT);
    pinMode(LEFT_BACKWARD_PIN, INPUT);

}

void MotorDriver::setSpeed(int targetLeftSpeed, int targetRightSpeed) {
    this->targetLeftSpeed = targetLeftSpeed;
    this->targetRightSpeed = targetRightSpeed;
}

void MotorDriver::startMove() {

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