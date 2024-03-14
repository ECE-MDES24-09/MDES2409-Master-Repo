#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#define RIGHT_FORWARD  6
#define LEFT_FOWARD  9

#define RIGHT_BACKWARD  5
#define LEFT_BACKWARD  10


class MotorDriver {

public:


void StopMove(int left, int right){
  double increment = .99;

  while (increment > 0){
	  if(right > 0)
  analogWrite(RIGHT_FORWARD,increment*right);
	  if(right < 0)
  analogWrite(RIGHT_BACKWARD,increment*right*-1);
	  if(left > 0)
  analogWrite(LEFT_FORWARD,increment*left);
	  if(left < 0)
  analogWrite(LEFT_BACKWARD,increment*left*-1);
  delay(1);
  increment = increment - .01;
  }
}

void StartMove(int left, int right){
  double increment = .01;
  while (increment < 1){
	  if (right > 0)
		  analogWrite(RIGHT_FORWARD, increment * right);
	  if (right < 0)
		  analogWrite(RIGHT_BACKWARD, increment * right * -1);
	  if (left > 0)
		  analogWrite(LEFT_FORWARD, increment * left);
	  if (left < 0)
		  analogWrite(LEFT_BACKWARD, increment * left * -1);
  delay(1);
  increment = increment + .01;
  Serial.print(increment);
  }
}

void FollowLine(int left, int right, double angle, double offset){


  analogWrite(RIGHT_FORWARD, right - 100*angle/90);//right forward
  analogWrite(LEFT_FORWARD, left + 100*angle/90);//left forward

  Serial.println(angle);
  Serial.println(offset);

}

}

#endif