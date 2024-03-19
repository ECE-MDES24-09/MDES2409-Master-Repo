#include <robot_control.h>
#include <Wire.h>

#define MOTOR_SPEED 125

RobotControl rc;
float angle = 0.0f;

void PickUpCubes();
void FollowLine();
void TurnLeft90();
void CrossGap();
  
void setup() { 
  Serial.begin(115200);
  Serial1.begin(115200);
  rc.init();
}

void loop() {
    //Delays start untill light turns on

    
    int start = rc.ColorSensor();
     while(start-20<rc.ColorSensor()){
     // Serial.println(rc.ColorSensor());
     //Serial.println(rc.USDistance());
    }

    PickUpCubes();
    TurnLeft90();
    FollowLine();

    //Spit out cubes
    rc.motorDriver.Infeed(-200);
    delay(2000);
    rc.motorDriver.Infeed(0);

    TurnLeft90();
    FollowLine();
    //PickUpThrusters();
    TurnLeft90();

  //DO NOT use linefollow function, you must use the code below.
  while(abs(rc.GetPixyAngle())<40){
  rc.lineFollow(150, 0);
  }
  rc.motorDriver.setSpeed(0, 0);
	for(int i = 0;i<256;i++){
		rc.motorDriver.startMove();
    delay(1);
	}

    CrossGap();
    
    //just presses button, need to code thruster drop off.
    rc.motorDriver.setSpeed(100,100);
    while(rc.USDistance()>17){
		rc.motorDriver.startMove();
    delay(1);
    }

    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<1000; i++){
		rc.motorDriver.startMove();
    delay(1);
		}

  	rc.motorDriver.setSpeed(-150,150);
		for(int i = 0; i<1400; i++){
		rc.motorDriver.startMove();
    delay(1);
		}

    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<1000; i++){
		rc.motorDriver.startMove();
    delay(1);
		}

    rc.motorDriver.setSpeed(-100,-100);
    while(rc.USDistance()<36){
		rc.motorDriver.startMove();
    delay(1);
    }

    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<1000; i++){
		rc.motorDriver.startMove();
    delay(1);
		}

    //Stops the robot
  int STOP = 0;
  while(STOP<1){}
}

void PickUpCubes(){

  rc.myservo.write(127);
  rc.motorDriver.Infeed(1);

  for(int i = 0; i<2; i++){
    rc.motorDriver.setSpeed(50,50);
    //Serial.println(rc.USDistance());
    while(rc.USDistance()>11){
		rc.motorDriver.startMove();
    Serial.println(rc.USDistance());
    delay(1);
    }

    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<500; i++){
		rc.motorDriver.startMove();
    delay(1);
		}

    rc.motorDriver.setSpeed(-50,-250);
		for(int i = 0; i<600; i++){
		rc.motorDriver.startMove();
    delay(1);
    }

    rc.motorDriver.setSpeed(-250,-50);
		for(int i = 0; i<600; i++){
		rc.motorDriver.startMove();
    delay(1);
    }

    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<500; i++){
		rc.motorDriver.startMove();
    delay(1);
		}
  }

    rc.motorDriver.setSpeed(50,50);
    //Serial.println(rc.USDistance());
    while(rc.USDistance()>11){
		rc.motorDriver.startMove();
    Serial.println(rc.USDistance());
    delay(1);
    }

    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<500; i++){
		rc.motorDriver.startMove();
    delay(1);
		}

    rc.motorDriver.setSpeed(-200,-200);
		for(int i = 0; i<500; i++){
		rc.motorDriver.startMove();
    delay(1);
    }

    rc.myservo.write(115);
    rc.motorDriver.Infeed(0);

    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<500; i++){
		rc.motorDriver.startMove();
    delay(1);
		}
}

void FollowLine(){

    while(abs(rc.GetPixyAngle())<40){
    rc.lineFollow(150, 0);
    }

    rc.motorDriver.setSpeed(200,200);
		for(int i = 0; i<1000; i++){
		rc.motorDriver.startMove();
    delay(1);
		}
    rc.motorDriver.setSpeed(0, 0);
	  for(int i = 255;i>0;i--){
		rc.motorDriver.startMove();
    delay(1);
	}

}

void TurnLeft90(){
  
    rc.motorDriver.setSpeed(-150,150);
		for(int i = 0; i<1400; i++){
		rc.motorDriver.startMove();
    delay(1);
		}
    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<500; i++){
		rc.motorDriver.startMove();
    delay(1);
		}
}

void CrossGap(){

  	rc.motorDriver.setSpeed(150,150);
		for(int i = 0; i<1000; i++){
		rc.motorDriver.startMove();
    delay(1);
		}
  	rc.motorDriver.setSpeed(50,50);
		for(int i = 0; i<7000; i++){
		rc.motorDriver.startMove();
    rc.myservo.write(93);
    delay(1);
		}
    rc.motorDriver.setSpeed(-50,-50);
		for(int i = 0; i<800; i++){
		rc.motorDriver.startMove();
    delay(1);
		}
    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<500; i++){
		rc.motorDriver.startMove();
    rc.myservo.write(120);
    delay(1);
		}

  	rc.motorDriver.setSpeed(255,255);
		for(int i = 0; i<1000; i++){
		rc.motorDriver.startMove();
    delay(1);
		}

    rc.motorDriver.setSpeed(0,0);
		for(int i = 0; i<1000; i++){
		rc.motorDriver.startMove();
    delay(1);
		}

}
