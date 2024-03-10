#include "RoboDriver.h"

RoboDriver::RoboDriver() {
  // Constructor
}

void RoboDriver::begin() {
  // Initialize GPIO pins, sensors, and modules
  // Initialize GPIO pins
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);
  // Initialize sensors and modules
  gyro_init();
  lineDetect.init();  
}

bool RoboDriver::followLineUntilTurn {
  // Follow line until a 90-degree turn is detected
  while (true) {
    lineDetect.refresh();
    double angle = lineDetect.getAng();
    double offset = lineDetect.getOS();

    // Print sensor readings
    Serial.println(angle);
    Serial.println(offset);

    // Check for 90-degree turn
    if ((abs(angle) >= MaxAngle)) {
      break;
    }

    // Continue following line
    motorDriver.FollowLine(BaseSpeed, BaseSpeed, angle, offset);
  }

  // Stop the motors after detecting a turn
  stopMotors();
  return true;
}

void RoboDriver::turn(float turnAngle) {
  // Function to execute a turn
  float currentAngle = 0;
  float targetAngle = turnAngle + getGyroAng();
  float turnSpeed = 0;

  // Normalize target angle
  if (targetAngle > 360) {
    targetAngle -= 360;
  }

  // Perform turning movement
  while (currentAngle > targetAngle - 2 || currentAngle < targetAngle + 2) {
    currentAngle = getGyroAng();
    turnSpeed = targetAngle - currentAngle;
    constrainTurnSpeed(turnSpeed);

    // Set motor speed for turning
    motorDriver.setSpeed(turnSpeed, -turnSpeed);
    motorDriver.startMove();
  }
}

void RoboDriver::stopMotors() {
  // Function to stop the motors
  motorDriver.setSpeed(0, 0);
  for (int i = 0; i < MotorRunDuration; i++) {
    motorDriver.startMove();
  }
}


void RoboDriver::constrainTurnSpeed(float &speed) {
  // Constrain the turn speed to a maximum value
  const float MaxTurnSpeed = 200;
  if (speed > MaxTurnSpeed) speed = MaxTurnSpeed;
  if (speed < -MaxTurnSpeed) speed = -MaxTurnSpeed;
}
