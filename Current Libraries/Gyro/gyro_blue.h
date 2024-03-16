#include "MPU9250.h"
#include <MadgwickAHRS.h>
<<<<<<< HEAD
#include <globalVariables.h>
=======
>>>>>>> 1b510070ff16c63498ab9274048c425d8414488c

#define CALIBRATE false

class Gyro {
public:
    void init();

    void updateYawPitchRoll();

    float getYaw();

private:
    MPU9250 mpu;
    Madgwick filter;
<<<<<<< HEAD
=======
    const unsigned long microsPerReading = 1000000 / 25;
    unsigned long microsPrevious = 0;
>>>>>>> 1b510070ff16c63498ab9274048c425d8414488c
};
