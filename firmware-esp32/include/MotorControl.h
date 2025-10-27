// MotorControl.h
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#define I2C_SDA_PIN 41
#define I2C_SCL_PIN 42
#define ARDUINO_ADDR 0x08


class MotorControl {
private:
    //uint8_t i2cAddress;
    int currentManualSpeed = 150; // Velocidad por defecto
    
public:
    //MotorControl(uint8_t address);
    void begin(); 
    void sendI2CCommand(uint8_t command, uint8_t value = 0);
    
    // Comandos básicos de movimiento
    void moveForward(int speed = 150);
    void moveBackward(int speed = 150);
    void turnLeft();
    void turnRight();
    void stopMotor();
    
    // Modos automáticos
    void enableAutonomousMode();
    void sendAutonomousCommand(byte command);
    void enableObstacleAvoidance();
    void enableWallFollowing();
    void emergencyStop();
    
    // Configuración de parámetros
    void setManualSpeed(int speed);
    void setAvoidanceParameters(int minDistance, int speed, int turnSpeed);
    void setWallFollowParameters(int speed, int idealDistance);
    
    // Funciones de giro suave (opcionales)
    void softTurnLeft();
    void softTurnRight();
};

#endif

/*
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
*/
/*
class MotorControl {
public:
  void begin();
  void moveForward(int speed);
  void moveBackward(int speed);
  void stopMotor();
  void turnLeft();
  void turnRight();
  void softTurnLeft();
  void softTurnRight();
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