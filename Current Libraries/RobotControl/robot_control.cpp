//
// Created by jorda on 03/15/2024.
//
#include "robot_control.h"

void RobotControl::init() {
    //Gyro.init();
    //TMPFront.init(SerialSelect::S0);
<<<<<<< HEAD
    motorDriver.init();
=======
	//MPU6050 mpu(Wire);
>>>>>>> 7c04040004ee4aac350c34e4e1a5f7da160fb546
    lineDetect.init();
    manipulatorControl.init();
    myservo.attach(RC_STORAGE_SERVO_PIN);
    pinMode(sensorPin, INPUT);
}


/**

    Sensor Control

**/

int RobotControl::ColorSensor() {
    return analogRead(sensorPin);
}

int RobotControl::GetDist() {
    //return TMPFront.getDist();
}

int RobotControl::GetPixyAngle() {
    return lineDetect.getAng(MAX_ANGLE);
}

double RobotControl::USDistance(){
	float distance_cm;
	digitalWrite(TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);

	// measure duration of pulse from ECHO pin
	distance_cm = pulseIn(ECHO_PIN, HIGH)*.017;


	return distance_cm;
}

/**

    Test Function

**/

void RobotControl::test() {
    //moves roller motors
    rollersIntake();

    //color sensor
    Serial.println(ColorSensor());

    //lidar
    //Serial.print(TMPFront.getDist());

    //moves block picking up thing
    myservo.write(100); //88 high 130 low

    //test line following
    lineFollow(130, 0);

    //test gyro code
    //Serial.println(Gyro.getYaw());

    //tests drive train
    motorDriver.setSpeed(-150,150);
    for(int i = 0; i<200; i++){
        motorDriver.startMove();
    }
    motorDriver.setSpeed(0,0);
    for(int i = 0; i<200; i++){
        motorDriver.startMove();
    }
}

/**

    Drive Control

**/

void RobotControl::turn(float Turn) {
    float Angle = 0;
    float FinalAngle = 0;
    float right = 0, left = 0, TurnSpeed = 0;

    while(Angle > Turn+3 || Angle < Turn-3){

        //Angle = getGyroAng(); // gyro_red
        //Angle = Gyro.getYaw(); // gyro_blue
        TurnSpeed = 2*(Turn - Angle);
        if(TurnSpeed > 200) TurnSpeed = 200;
        if(TurnSpeed < -200) TurnSpeed = -200;
        if(TurnSpeed > 0 && TurnSpeed < 55) TurnSpeed = 55;
        if(TurnSpeed > -55 && TurnSpeed < 0) TurnSpeed = -55;

        Serial.print(Angle);
        Serial.print(" ");
        Serial.print(Turn);
        Serial.print(" ");
        Serial.println(TurnSpeed);

        motorDriver.setSpeed(TurnSpeed, -TurnSpeed);
        motorDriver.startMove();


    }
    motorDriver.setSpeed(0, 0);
    for(int i = 255;i>0;i--){
        motorDriver.startMove();
    }
}

double RobotControl::USDistance(){
    float distance_cm;
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // measure duration of pulse from ECHO pin
    distance_cm = pulseIn(ECHO_PIN, HIGH)*.017;


    return distance_cm;
}



void RobotControl::followHeading(float Direction) {
    //float Angle = getGyroAng(); // gyro_red
    //float Angle = Gyro.getYaw(); // gyro_blue
    //if(Angle > Direction){

    //}
    //if(Angle < Direction){

    //}
}

void RobotControl::lineFollow(int robotSpeed, double targetOffset) {
    lineDetect.update();

    double angle = lineDetect.getAng(MAX_ANGLE);
    double lineOffset = lineDetect.getOffset();

    Serial.println(angle);

    // compute target angle and speed offset
    //++
    double sensitivity = 1 / 300.0;
    double ossen = lineOffset * sensitivity;
    if (ossen > 1.0) ossen = 1.0;
    else if (ossen < -1.0) ossen = -1.0;
    double targetAng = asin(ossen) * 180 / M_PI;
    double angleDiff = angle - targetAng;
    int speedOffset = static_cast<int>(100.0 * (angleDiff) / 90.0);

    // another way
    //int speedOffset = 100 * (angle) / 90 - lineOffset;  // double to int
    //speedOffset *= 5;

    //speedOffset *= 1;
    // Serial.print(" ");
    // Serial.println(speedOffset);
    // output to the motors
    int leftSpeed = robotSpeed + speedOffset;
    int rightSpeed = robotSpeed - speedOffset;

    if (leftSpeed > 255)
        leftSpeed = 255;
    else if (leftSpeed < -255)
        leftSpeed = -255;

    if (rightSpeed > 255)
        rightSpeed = 255;
    else if (rightSpeed < -255)
        rightSpeed = -255;

    // Serial.print(leftSpeed);
    // Serial.print(rightSpeed);

    motorDriver.setSpeed(leftSpeed, rightSpeed);
}
// TODO: Implement crossGap
void RobotControl::crossGap() {

}

/**

    Manipulator Control

**/
void RobotControl::rollersIntake() {
    manipulatorControl.setStorageAngle(pickUpAng, 0.3);
    manipulatorControl.setLeftRollerSpeed(50);
    manipulatorControl.setRightRollerSpeed(50);
}

void RobotControl::rollersDispense() {
    manipulatorControl.setStorageAngle(CubeDropAng, 0.3);
    manipulatorControl.setLeftRollerSpeed(-50);
    manipulatorControl.setRightRollerSpeed(-50);
}

void RobotControl::rollersStop() {
    manipulatorControl.setLeftRollerSpeed(0);
    manipulatorControl.setRightRollerSpeed(0);
}

void RobotControl::rocketDrop() {
    manipulatorControl.setStorageAngle(fuelDropAng, 0.3);
    rollersStop();

}

void RobotControl::cruisin() {
    manipulatorControl.setStorageAngle(crusinAng, 0.3);
    rollersStop();

}

void RobotControl::servo_write() {
    myservo.write(targetStorageAng);
}


/**

    Code for Manual Control.

**/
/**
void RobotControl::manualControl() {
    // Wheels control
    int leftSpeed = dataRecieved[0] * 2 + 1;
    int rightSpeed = dataRecieved[1] * 2 + 1;

    leftSpeed = (int)(leftSpeed * SPEED_MULTIPLIER + 0.5);
    rightSpeed = (int)(rightSpeed * SPEED_MULTIPLIER + 0.5);

    if (leftSpeed <= 20 && leftSpeed >= -20)
        leftSpeed = 0;

    if (rightSpeed <= 20 && rightSpeed >= -20)
        rightSpeed = 0;

    // Serial.print(leftSpeed);
    // Serial.print(rightSpeed);
    motorDriver.setSpeed(leftSpeed, rightSpeed);


    // Roller control
    if (dataRecieved[3] == 1 && lastDataReceived3 == 0)
        HandleSquareButtonPress();

    lastDataReceived3 = dataRecieved[3]; // Update lastDataReceived3 for the next cycle

    if (dataRecieved[4] == 1 && lastDataReceived4 == 0)
        HandleCrossButtonPress();

    lastDataReceived4 = dataRecieved[4]; // Update lastDataReceived3 for the next cycle

    // Storage control
    if (dataRecieved[2] == 1 && lastDataReceived2 == 0)
        HandleTriangleButtonPress();

    lastDataReceived2 = dataRecieved[2]; // Update lastDataReceived2 for the next cycle
}

void RobotControl::connectionCheck() {
    // stop the motors if controller is disconnected
    if (millis() - lastTimeRecevieData > 100) {
        for (byte i = 0; i < DATA_LENGTH; i++) {
            dataRecieved[i] = 0;
        }
    }
}

void RobotControl::updateSerialInput() {
    // write to data received

    int8_t* temp_data = getSerialData(); // Attempt to get data after "START"

    // If data is received and is not just zeros (check for valid data)
    if (temp_data[0] != 0 || temp_data[1] != 0) {
        for (byte i = 0; i < DATA_LENGTH; i++)
            dataRecieved[i] = temp_data[i];
    }
}

void RobotControl::HandleSquareButtonPress() {
    switch (rollersState)
    {
        case RollerState::Stop:
            rollersState = RollerState::InTake;
        //LeftRollerJoint.targetAngularVelocity = new Vector3(0, RollersSpeed, 0);
        //RightRollerJoint.targetAngularVelocity = new Vector3(0, -RollersSpeed, 0);
        break;
        case RollerState::OutTake:
            rollersState = RollerState::Stop;
        //LeftRollerJoint.targetAngularVelocity = new Vector3(0, 0, 0);
        //RightRollerJoint.targetAngularVelocity = new Vector3(0, 0, 0);
        break;
    }
}

void RobotControl::HandleCrossButtonPress() {
    switch (rollersState)
    {
        case RollerState::Stop:
            rollersState = RollerState::OutTake;
        //LeftRollerJoint.targetAngularVelocity = new Vector3(0, -RollersSpeed, 0);
        //RightRollerJoint.targetAngularVelocity = new Vector3(0, RollersSpeed, 0);
        break;
        case RollerState::InTake:
            rollersState = RollerState::Stop;
        //LeftRollerJoint.targetAngularVelocity = new Vector3(0, 0, 0);
        //RightRollerJoint.targetAngularVelocity = new Vector3(0, 0, 0);
        break;
    }
}

void RobotControl::HandleTriangleButtonPress() {
    // Based on the current step in the cycle, move to the next state
    switch (storageCycleStep) {
        case 0: // Currently in the first Cruise
            storageState = StorageState::PickUp;
            targetStorageAng = pickUpAng; // Set target angle to pick up position
            storageCycleStep++; // Move to the next step in the cycle
            break;
        case 1: // After PickUp, move back to Cruise
            storageState = StorageState::Cruise;
            targetStorageAng = crusinAng; // Set target angle for cruising
            storageCycleStep++; // Move to the next step in the cycle
            break;
        case 2: // In the second Cruise, move to FuelDrop
            storageState = StorageState::FuelDrop;
            targetStorageAng = fuelDropAng; // Set target angle for fuel drop
            storageCycleStep++; // Prepare for the final transition back to Cruise
            break;
        case 3: // After FuelDrop, return to Cruise and reset cycle
            storageState = StorageState::Cruise;
            targetStorageAng = crusinAng; // Set target angle for cruising
            storageCycleStep = 0; // Reset the cycle
            break;
        default:
            // Just in case, reset the cycle if it's out of expected range
            storageCycleStep = 0;
            break;
    }
}
void RobotControl::printStates() {
        String message = "Rollers State: ";

        // Append the current state of rollersState to the message
        switch (rollersState) {
            case RollerState::InTake:
                message += "InTake";
            break;
            case RollerState::OutTake:
                message += "OutTake";
            break;
            case RollerState::Stop:
                message += "Stop";
            break;
            default:
                message += "Unknown";
            break;
        }

        message += ", Storage State: ";

        // Append the current state of storageState to the message
        switch (storageState) {
            case StorageState::Start:
                message += "Start";
            break;
            case StorageState::PickUp:
                message += "PickUp";
            break;
            case StorageState::BigCubeDrop:
                message += "BigCubeDrop";
            break;
            case StorageState::SmallCubeDrop:
                message += "SmallCubeDrop";
            break;
            case StorageState::FuelDrop:
                message += "FuelDrop";
            break;
            case StorageState::Cruise:
                message += "Cruise";
            break;
            default:
                message += "Unknown";
            break;
        }

        // Print the entire message on a single line
        Serial.println(message);
}
**/
