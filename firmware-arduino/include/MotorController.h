// MotorController.h
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "Command.h" // Incluir comandos

// PINES L298N
const int ENA = 9;   // PWM motor izquierdo
const int IN1 = 7;   
const int IN2 = 6;   
const int ENB = 10;  // PWM motor derecho  
const int IN3 = 5;   
const int IN4 = 4;

// Pines sonar (si lo usas)
#define TRIG_PIN 8    
#define ECHO_PIN 11  

enum OperationMode {
  MODE_EMERGENCY_STOP,
  MODE_MANUAL,
  MODE_AUTONOMOUS,
  MODE_OBSTACLE_AVOIDANCE
};

class MotorController {
private:
// Pines para driver L298N
    int enA, in1, in2;  // Motor izquierdo
    int enB, in3, in4;  // Motor derecho
    // Estado del sistema
    bool sonarEnabled = true;  // Por defecto activado
    OperationMode currentMode = MODE_MANUAL;
    bool emergencyStopActive = false;
    unsigned long lastCommandTime = 0;
    const unsigned long SAFETY_TIMEOUT = 5000; // 1 segundo
    
    // Velocidades
    int manualSpeed = 150;
    int autoSpeed = 120;
    
    
public:
     MotorController(int enA_pin, int in1_pin, int in2_pin, 
                   int enB_pin, int in3_pin, int in4_pin);
    
    void begin(); 
    // Sistema de prioridades
    void handleCommand(byte command, byte data = 180);
    void updateSafety(); // Verificación de seguridad continua
    void emergencyStop();
    void stop();
    // Getters de estado
    bool isInEmergency() { return emergencyStopActive; }
    void setSonarState(bool enabled);
    bool isSonarEnabled() { return sonarEnabled; }
    OperationMode getCurrentMode() { return currentMode; }
    
private:
    // Implementación interna de movimientos
    void executeManualCommand(byte command);
    void executeAutonomousCommand(byte command);
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    
    
    // Verificaciones de seguridad
    bool isSafetyTimeout();
    void resetSafetyTimer();
};

#endif


/*
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>

enum Command {
  CMD_STOP = 0,
  CMD_FORWARD = 1,
  CMD_BACKWARD = 2,
  CMD_LEFT = 3,
  CMD_RIGHT = 4,
  CMD_OBSTACLE_AVOIDANCE = 5,
  CMD_WALL_FOLLOWING = 6,
  CMD_EMERGENCY_STOP = 7,
  CMD_SET_AVOIDANCE_PARAMS = 8,
  CMD_SET_WALL_FOLLOW_PARAMS = 9,
  CMD_SET_MANUAL_SPEED = 10
};

enum OperationMode {
  MODE_MANUAL,
  MODE_OBSTACLE_AVOIDANCE,
  MODE_WALL_FOLLOWING
};

class MotorController {
private:
private:
    OperationMode currentMode = MODE_MANUAL;
    bool emergencyStopActive = false;

    // Parámetros de velocidad, viejos
    int defaultSpeed = 150;
    int avoidanceSpeed = 120;
    int wallFollowSpeed = 100;
    int turnSpeed = 180;
    
    // Agregar variables para pines de motor viejos
    int motorLeft1, motorLeft2, motorRight1, motorRight2;
    
public:
void handleCommand(byte command, byte data = 0);
    void updateSafety(); // Verificación continua de seguridad
    void emergencyStop();
    bool isInEmergency() { return emergencyStopActive; }

    // Constructor para inicializar pines viejo
    MotorController(int ml1, int ml2, int mr1, int mr2) {
        motorLeft1 = ml1;
        motorLeft2 = ml2;
        motorRight1 = mr1;
        motorRight2 = mr2;
    }
    
    void begin() {
        pinMode(motorLeft1, OUTPUT);
        pinMode(motorLeft2, OUTPUT);
        pinMode(motorRight1, OUTPUT);
        pinMode(motorRight2, OUTPUT);
    }
    
    void setAvoidanceParams(int speed, int turnSpd) {
        avoidanceSpeed = speed;
        turnSpeed = turnSpd;
    }
    
    void setManualParams(int speed) {
        defaultSpeed = speed;
    }
    
    void setWallFollowParams(int speed) {
        wallFollowSpeed = speed;
    }
    
    void moveForward(OperationMode mode) {
        switch(mode) {
            case MODE_OBSTACLE_AVOIDANCE:
                analogWrite(motorLeft1, avoidanceSpeed);
                analogWrite(motorRight1, avoidanceSpeed);
                break;
            case MODE_WALL_FOLLOWING:
                analogWrite(motorLeft1, wallFollowSpeed);
                analogWrite(motorRight1, wallFollowSpeed);
                break;
            default:
                analogWrite(motorLeft1, defaultSpeed);
                analogWrite(motorRight1, defaultSpeed);
        }
    }
    
    void stop() {
        analogWrite(motorLeft1, 0);
        analogWrite(motorLeft2, 0);
        analogWrite(motorRight1, 0);
        analogWrite(motorRight2, 0);
    }
};

#endif
*/