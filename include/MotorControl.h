#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Wire.h>

// Configuración I2C para comunicación con Arduino
#define ARDUINO_ADDR 0x08

#define I2C_SDA_PIN 41
#define I2C_SCL_PIN 42

#define CMD_SPEED_1    '1'
#define CMD_SPEED_2    '2'
#define CMD_SPEED_3    '3'

// Comandos I2C
#define CMD_STOP       0x00
#define CMD_FORWARD    0x01
#define CMD_BACKWARD   0x02
#define CMD_LEFT       0x03
#define CMD_RIGHT      0x04
#define CMD_SPEED      0x05

class MotorControl {
public:
  void begin();
  void moveForward(int speed);
  void moveBackward(int speed);
  void stopMotor();
  void turnLeft();
  void turnRight();
  void softStop(int delayTime);
  void rampSpeed(int targetSpeed, int rampTime);
  

private:
  void sendI2CCommand(uint8_t command, uint8_t value = 0);
  int currentSpeed;
  bool isMovingForward;
};

#endif



/*
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class MotorControl {
public:
  void begin();
  void moveForward(int speed);
  void moveBackward(int speed);
  void stopMotor();
  void softStop(int delayTime);
  void rampSpeed(int targetSpeed, int rampTime);
  void turnInPlace(bool clockwise, int speed);
  
  int currentSpeed;
  bool isMovingForward;

private:
  // Pines para L293D
  static const int MOTOR_IN1 = 39;
  static const int MOTOR_IN2 = 40;
  static const int MOTOR_ENA = 41;
  
  // Configuración PWM
  static const int MOTOR_PWM_CHANNEL = 1;
  static const int PWM_FREQ = 5000;
  static const int PWM_RESOLUTION = 8;
};

#endif
*/