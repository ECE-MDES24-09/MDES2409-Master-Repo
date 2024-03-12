#ifndef MANIPULATOR_CONTROL_H
#define MANIPULATOR_CONTROL_H

#include "Arduino.h"
#include <Servo.h>

enum class RollerState {
    InTake,
    OutTake,
    Stop,
};

enum class StorageState {
    Start,
    PickUp,
    BigCubePickUp, // New state for picking up big cubes
    BigCubeDrop,   // New state for dropping big cubes
    SmallCubeDrop,
    FuelDrop,
    Cruise,
};

class Manipulator_Control {
public:
    void init();
    void setStorageAngle(float angle);
    void updateState(); // Function to update the state machine

private:
    const int STORAGE_SERVO_PIN = 12;
    Servo storageServo;

    RollerState rollersState = RollerState::Stop;
    StorageState storageState = StorageState::Start;

    const float bigCubeDropAng = 115.0f;
    const float smallCubeDropAng = 115.0f;
    const float pickUpAng = 115.0f;
    const float crusingAng = 90.0f;
    const float fuelDropAng = 75.0f;
    const float startAng = 90.0f;

    void handleState(); // Function to handle the current state
};

#endif
