// MotorController.h
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include "Command.h" // Incluir comandos
/*
// Comandos con prioridades integradas
enum Command {
  // PRIORIDAD 1: Comandos de seguridad/emergencia
  CMD_EMERGENCY_STOP = 0,
  
  // PRIORIDAD 2: Comandos manuales (desde BLE)
  CMD_MANUAL_FORWARD = 1,
  CMD_MANUAL_BACKWARD = 2,
  CMD_MANUAL_LEFT = 3,
  CMD_MANUAL_RIGHT = 4,
  
  // PRIORIDAD 3: Comandos autónomos (desde ESP32)
  CMD_AUTO_FORWARD = 5,
  CMD_AUTO_BACKWARD = 6,
  CMD_AUTO_LEFT = 7,
  CMD_AUTO_RIGHT = 8,
  CMD_AUTO_NAVIGATE = 9,
  
  // PRIORIDAD 4: Configuración
  CMD_SET_MANUAL_SPEED = 10,
  CMD_SET_AUTO_SPEED = 11,
  CMD_STOP = 12
};*/

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