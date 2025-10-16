#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <ESP32Servo.h>

class ServoControl {
public:
  void begin();
  void centerSteering();
  void turnLeft();
  void turnRight();
  void smoothLeft();
  void smoothRight();
  
private:
  Servo steeringServo;
};

#endif