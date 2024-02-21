#include <MotorDriver.h>
#include <Servo.h>

Servo myservo;
MotorDriver motorDriver;

void setup() {
  // put your setup code here, to run once:
myservo.attach(12); 
myservo.write(90); //80 to 115
pinMode(5,OUTPUT);
pinMode(4,OUTPUT);
pinMode(3,OUTPUT);
pinMode(2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:


motorDriver.setSpeed(50, 50);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
delay(4000);
  motorDriver.setSpeed(0, 0);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
  motorDriver.setSpeed(254, 254);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
delay(5000);
  motorDriver.setSpeed(0, 0);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
int test = 0;
while(test<1){

}

}
