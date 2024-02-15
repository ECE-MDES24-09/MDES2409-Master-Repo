#include <MotorDriver.h>
#include <pixy_line_detection.h>

#define MaxAngle = 90

pixyLineDetect lineDetect;
MotorDriver motorDriver;

int Speed = 125;

void setup() {
pinMode(7,OUTPUT);
pinMode(6,OUTPUT);
pinMode(5,OUTPUT);
pinMode(4,OUTPUT);

// PWM pins 3 and 11 frequency = 31372.55Hz
// TCCR2B = TCCR2B & B11111000 | B00000001;
//9 and 10
// TCCR1B = TCCR1B & B11111000 | B00000001;
// //5 and 6
// TCCR0B = TCCR0B & B11111000 | B00000001;

Serial.begin(115200);
lineDetect.init();
  
}

void loop() {

  lineDetect.refresh();
  double ang = lineDetect.getAng(70);
  double offset = lineDetect.getOS();
  Serial.println(ang);

  double test = abs(lineDetect.getAng(90));


while (test < 70){
  lineDetect.refresh();
  ang = lineDetect.getAng(90);
  offset = lineDetect.getOS();
  Serial.println(ang);
  Serial.println(offset);
  test = abs(ang);
  motorDriver.FollowLine(200,200,ang,offset);
}
  motorDriver.setSpeed(0, 0);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
//   motorDriver.setSpeed(250, 250);
// for(int i=0; i<1200; i++){
//     motorDriver.startMove();
// }
//   motorDriver.setSpeed(-250, 250);
//   for(int i=0; i<1300; i++){
//     motorDriver.startMove();
// }
//   motorDriver.setSpeed(0, 0);
//   for(int i=0; i<3000; i++){
//     motorDriver.startMove();
// }
//   motorDriver.setSpeed(250, -250);
//   for(int i=0; i<1400; i++){
//     motorDriver.startMove();
// }
//   motorDriver.setSpeed(-250, -250);
//   for(int i=0; i<800; i++){
//     motorDriver.startMove();
// }
//   motorDriver.setSpeed(250, -250);
//   for(int i=0; i<1000; i++){
//     motorDriver.startMove();
// }
  motorDriver.setSpeed(250, 250);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
  motorDriver.setSpeed(0, 0);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}

  motorDriver.setSpeed(250, -250);
for(int i=0; i<1000; i++){
    motorDriver.startMove();
}
  motorDriver.setSpeed(0, 0);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
lineDetect.refresh();
test = abs(lineDetect.getAng(90));
while (test < 70){
  lineDetect.refresh();
  ang = lineDetect.getAng(90);
  offset = lineDetect.getOS();
  Serial.println(ang);
  Serial.println(offset);
  test = abs(ang);
  motorDriver.FollowLine(200,200,ang,offset);
}
  motorDriver.setSpeed(0, 0);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
  motorDriver.setSpeed(200, 200);
  for(int i=0; i<500; i++){
    motorDriver.startMove();
}
  motorDriver.setSpeed(250, -250);
  for(int i=0; i<1000; i++){
    motorDriver.startMove();
}
  motorDriver.setSpeed(0, 0);
for(int i=0; i<500; i++){
    motorDriver.startMove();
}
lineDetect.refresh();
test = abs(lineDetect.getAng(90));
while (test < 70){
  lineDetect.refresh();
  ang = lineDetect.getAng(90);
  offset = lineDetect.getOS();
  Serial.println(ang);
  Serial.println(offset);
  test = abs(ang);
  motorDriver.FollowLine(200,200,ang,offset);
}


  motorDriver.setSpeed(0, 0);
int i = 0;
while(i<1){
motorDriver.startMove();
}

//  {
//  lineDetect.refresh();
//  ang = lineDetect.getAng(60.0);
//  offset = lineDetect.getOS(60.0);
//  
//  //FollowLine(125, 125, ang, offset);
//  delay(1);
//  
//  }
}
