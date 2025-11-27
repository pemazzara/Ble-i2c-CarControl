// Esp32Slave - MotorControl.cpp
#include "MotorControl.h"

MotorControl::MotorControl() {}

void MotorControl::begin() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    stop();
    resetSafetyTimer();
    Serial.println("🚗 MotorControl inicializado.");
}

void MotorControl::handleCommand(byte command, byte data) {
    resetSafetyTimer();
    
    // PRIORIDAD 1: EMERGENCY STOP
    if (command == CMD_EMERGENCY_STOP) {
        emergencyStop();
        return;
    }
    
    if (emergencyStopActive && command != CMD_STOP) {
        Serial.println("⚠️  Ignorando comando - Sistema en emergencia");
        return;
    }

    if (command == CMD_STOP) {
        if (emergencyStopActive) emergencyStopActive = false;
        stop();
        Serial.println("🛑 Comando STOP recibido y emergencia reseteada.");
        return;
    }

    // COMANDOS DE CONFIGURACIÓN (usar data)
    switch(command) {
        case CMD_SET_MANUAL_SPEED:
            manualSpeed = data;
            Serial.printf("🔧 Velocidad manual: %d\n", manualSpeed);
            return;
        case CMD_SET_AUTO_SPEED:
            autoSpeed = data;
            Serial.printf("🔧 Velocidad auto: %d\n", autoSpeed);
            return;
    }
    
    // COMANDOS DE MOVIMIENTO
    if (command >= CMD_MANUAL_FORWARD && command <= CMD_MANUAL_RIGHT) {
        executeManualCommand(command);
        return;
    }
}

void MotorControl::updateSafety() {
    if (isSafetyTimeout() && !emergencyStopActive) {
        stop();
    }
}

void MotorControl::emergencyStop() {
    emergencyStopActive = true;
    stop();
    Serial.println("🛑 PARADA DE EMERGENCIA ACTIVADA (I2C o Timeout)");
}

void MotorControl::executeManualCommand(byte command) {
    switch(command) {
        case CMD_MANUAL_FORWARD:
        Serial.println("🎮 Comando MANUAL ADELANTE");
            moveForward(manualSpeed);
            break;
        case CMD_MANUAL_BACKWARD:
            moveBackward(manualSpeed);
            break;
        case CMD_MANUAL_LEFT:
            turnLeft(manualSpeed);
            break;
        case CMD_MANUAL_RIGHT:
            turnRight(manualSpeed);
            break;
    }
    Serial.printf("🎮 Comando %d ejecutado - Vel: %d\n", command, manualSpeed);
}

// Implementación de movimientos (omitiendo la función executeAutonomousCommand por simplicidad)
void MotorControl::moveForward(int speed) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed);
}
void MotorControl::moveBackward(int speed) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, speed);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, speed);
}
void MotorControl::turnLeft(int speed) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(ENA, speed); // Izq: Atrás
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed); // Der: Adelante
}
void MotorControl::turnRight(int speed) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed); // Izq: Adelante
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(ENB, speed); // Der: Atrás
}
void MotorControl::stop() {
    analogWrite(ENA, 0); analogWrite(ENB, 0);
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void MotorControl::setMotorSpeeds(int leftSpeed, int rightSpeed) {
        // Suavizado de velocidad
        currentLeftSpeed = smoothSpeed(currentLeftSpeed, leftSpeed);
        currentRightSpeed = smoothSpeed(currentRightSpeed, rightSpeed);
        
        analogWrite(ENA, currentLeftSpeed);
        analogWrite(ENB, currentRightSpeed);
    }
 
    int MotorControl::smoothSpeed(int current, int target) {
        if(abs(current - target) <= MAX_ACCELERATION) return target;
        return (current < target) ? current + MAX_ACCELERATION : current - MAX_ACCELERATION;
    }

    // ✅ AGREGAR función para obtener estado
    uint8_t MotorControl::getMotorStatus() {
        if(emergencyStopActive) return 0x01; // Emergencia activa
        if(currentLeftSpeed > 0 || currentRightSpeed > 0) return 0x02; // Movimiento
        return 0x00; // Detenido
    }

