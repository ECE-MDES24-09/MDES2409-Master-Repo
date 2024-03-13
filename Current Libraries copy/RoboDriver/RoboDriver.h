
#ifndef RoboDriver_h
#define RoboDriver_h

// Including necessary libraries and other header files
#include <Arduino.h>              // Standard Arduino library
#include <MotorDriver.h>          // Custom library for motor control
#include <pixy_line_detection.h>  // Custom library for line detection using Pixy camera
#include <gyro.h>                 // Custom library for gyroscopic sensor
#include <globalVariables.h>
// Definition of the RoboDriver class
class RoboDriver {
public:
  // Constructor declaration
  RoboDriver();

  // Initializes the RoboDriver
  void begin();

  // Follows a line until a turn is detected, returns true if turn is detected
  bool followLineUntilTurn();
  
  // Stops all motors
  bool stopTheMotors();
  
  // Starts all motors
  bool startTheMotors();
  
  bool doTheJig();
  
  bool takeItBack();
  
  bool circleLeft();
  
  bool circleRight();
	
  // Makes a turn with a specified angle
  void turn(float turnAngle);

private:
  // Objects for motor control and sensors
  MotorDriver motorDriver;        // Motor driver to control robot's movement
  pixyLineDetect lineDetect;      // Line detection object using Pixy camera
  //const int MaxAngle = 90;        // Maximum angle for turning
  //const int BaseSpeed = 125;      // Base speed for the motors
  //const int MotorRunDuration = 500; // Duration for running the motors in milliseconds

  // Stops all motors
  void stopMotors();

  // Constrains the turn speed to a safe range
  void constrainTurnSpeed(float &speed);
};

// End of include guard
#endif
