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
    int lastSonarDistance = -1;
    bool sonarEnabled = true;
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
    void setSonarState(bool enabled);
    int getSonarDistance();
    void requestSonarData();
    // Funciones de giro suave (opcionales)
    void softTurnLeft();
    void softTurnRight();
};

#endif
