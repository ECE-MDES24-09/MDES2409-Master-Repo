#include "MPU9250.h"
#include <MadgwickAHRS.h>

#define CALIBRATE false

class Gyro {
public:
    void init();

    void updateYawPitchRoll();

    float getYaw();

private:
    MPU9250 mpu;
    Madgwick filter;
};
