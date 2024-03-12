#include "Manipulator_Control.h"

void Manipulator_Control::init() {
    storageServo.attach(STORAGE_SERVO_PIN);
}

void Manipulator_Control::setStorageAngle(float angle) {
    storageServo.write(angle);
}

void Manipulator_Control::updateState() {
    handleState();
}

void Manipulator_Control::handleState() {
    switch (storageState) {
        case StorageState::BigCubePickUp:
            // Perform actions for picking up big cube
            setStorageAngle(pickUpAng);
            // Transition to next state
            storageState = StorageState::BigCubeDrop;
            break;
        case StorageState::BigCubeDrop:
            // Perform actions for dropping big cube
            setStorageAngle(bigCubeDropAng);
            // Transition to next state
            storageState = StorageState::Cruise;
            break;
        // Add other cases for handling different states as needed
        default:
            // Handle default case or unknown states
            break;
    }
}
