#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

void gyro_init(){



	while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {

      initialized = true;
    }
  }
}



float getGyroAng(){
    float test = 0;
    float x = 0;
    float y = 0;
    float x1 = 0;
    float y1 = 0;
    float x2 = 0;
    float y2 = 0;
    float x3 = 0;
    float y3 = 0;
    int16_t z = 0;
    float Angle = 0;
	

	if (myICM.dataReady()){
        myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
        //y = myICM.getAGMT().mag.axes.y;         // The values are only updated when you call 'getAGMT'
        x1 = myICM.magX() + 40;
        y1 = myICM.magY() - 70;
        myICM.getAGMT();
        x2 = myICM.magX() + 40;
        y2 = myICM.magY() - 70;
        myICM.getAGMT();
        x3 = myICM.magX() + 40;
        y3 = myICM.magY() - 70;

        x = (x1 + x2 + x3) / 3;
        y = (y1 + y2 + y3) / 3;

        Angle = atan2(x, y) * 180.0 / PI+180;
		Serial.print(x);
        Serial.print(" ");
        Serial.println(y);
	}
	
	return Angle;
}