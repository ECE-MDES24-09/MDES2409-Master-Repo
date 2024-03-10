#ifndef RoboDriver_h
#define RoboDriver_h

#include <Arduino.h>
#include <MotorDriver.h>
#include <pixy_line_detection.h>
#include <gyro.h>

class RoboDriver {
public:
  RoboDriver();
  void begin();
  bool followLineUntilTurn();
  void turn(float turnAngle);

private:
  MotorDriver motorDriver;
  pixyLineDetect lineDetect;
  const int MaxAngle = 90;
  const int BaseSpeed = 125;
  const int MotorRunDuration = 500;

  void stopMotors();
  void constrainTurnSpeed(float &speed);
};

#endif
