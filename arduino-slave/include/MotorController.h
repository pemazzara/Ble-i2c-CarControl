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
//#define TRIG_PIN 8    
//#define ECHO_PIN 11  

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
