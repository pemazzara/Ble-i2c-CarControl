// MotorController.cpp
#include "MotorController.h"

MotorController::MotorController(int enA_pin, int in1_pin, int in2_pin,
                               int enB_pin, int in3_pin, int in4_pin)
    : enA(enA_pin), in1(in1_pin), in2(in2_pin),
      enB(enB_pin), in3(in3_pin), in4(in4_pin) {
}

void MotorController::begin() {
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    stop();
    resetSafetyTimer();
}
// 🎯 NÚCLEO DEL SISTEMA DE PRIORIDADES
void MotorController::handleCommand(byte command, byte data) {
    resetSafetyTimer();
    
    // PRIORIDAD 1: EMERGENCY STOP
    if (command == CMD_EMERGENCY_STOP) {
        emergencyStop();
        return;
    }
    
    if (emergencyStopActive) {
        Serial.println("⚠️  Ignorando comando - Sistema en emergencia");
        return;
    }
    // ✅ AGREGAR CMD_STOP aquí - Prioridad alta
    if (command == CMD_STOP) {
        Serial.println("🛑 Comando STOP recibido");
        stop();
        currentMode = MODE_MANUAL;
        return;
    }
    // COMANDOS DE CONFIGURACIÓN (usar data)
    switch(command) {
        case CMD_SET_MANUAL_SPEED:
            manualSpeed = data;
            Serial.print("🔧 Velocidad manual configurada: ");
            Serial.println(manualSpeed);
            return;  // IMPORTANTE: return para que no procese como movimiento
        case CMD_SET_AUTO_SPEED:
            autoSpeed = data;
            Serial.print("🔧 Velocidad auto configurada: ");
            Serial.println(autoSpeed);
            return;  // IMPORTANTE: return aquí también
    }
    
    // COMANDOS DE MOVIMIENTO (NO usar data)
    if (command >= CMD_MANUAL_FORWARD && command <= CMD_MANUAL_RIGHT) {
        currentMode = MODE_MANUAL;
        executeManualCommand(command);  // ← SIN parámetro data
        return;
    }
    
    if (command >= CMD_AUTO_FORWARD && command <= CMD_AUTO_NAVIGATE) {
        currentMode = MODE_AUTONOMOUS;
        executeAutonomousCommand(command);  // ← SIN parámetro data
        return;
    }
}

// SEGURIDAD CONTINUA
void MotorController::updateSafety() {
    // Timeout de seguridad - si no hay comandos en X tiempo, parar
    if (isSafetyTimeout() && !emergencyStopActive) {
        Serial.println("⏰ Timeout de seguridad - Deteniendo motores");
        stop();
    }
}

void MotorController::emergencyStop() {
    emergencyStopActive = true;
    currentMode = MODE_EMERGENCY_STOP;
    stop();
    Serial.println("🛑 PARADA DE EMERGENCIA ACTIVADA");
}

// IMPLEMENTACIÓN DE MOVIMIENTOS
void MotorController::executeManualCommand(byte command) {
    // Usar manualSpeed configurada previamente
    switch(command) {
        case CMD_MANUAL_FORWARD:
            Serial.print("🎮 MANUAL: Adelante - Vel: ");
            Serial.println(manualSpeed);
            moveForward(manualSpeed);
            break;
        case CMD_MANUAL_BACKWARD:
            Serial.print("🎮 MANUAL: Atrás - Vel: ");
            Serial.println(manualSpeed);
            moveBackward(manualSpeed);
            break;
        case CMD_MANUAL_LEFT:
            Serial.print("🎮 MANUAL: Izquierda - Vel: ");
            Serial.println(manualSpeed);
            turnLeft(manualSpeed);
            break;
        case CMD_MANUAL_RIGHT:
            Serial.print("🎮 MANUAL: Derecha - Vel: ");
            Serial.println(manualSpeed);
            turnRight(manualSpeed);
            break;
    }
}

void MotorController::executeAutonomousCommand(byte command) {
    // Usar autoSpeed configurada previamente
    switch(command) {
        case CMD_AUTO_FORWARD:
            Serial.print("🤖 AUTO: Adelante - Vel: ");
            Serial.println(autoSpeed);
            moveForward(autoSpeed);
            break;
        case CMD_AUTO_BACKWARD:
            Serial.print("🤖 AUTO: Atrás - Vel: ");
            Serial.println(autoSpeed);
            moveBackward(autoSpeed);
            break;
        case CMD_AUTO_LEFT:
            Serial.print("🤖 AUTO: Izquierda - Vel: ");
            Serial.println(autoSpeed);
            turnLeft(autoSpeed);
            break;
        case CMD_AUTO_RIGHT:
            Serial.print("🤖 AUTO: Derecha - Vel: ");
            Serial.println(autoSpeed);
            turnRight(autoSpeed);
            break;
        case CMD_AUTO_NAVIGATE:
            Serial.print("🤖 AUTO: Navegación - Vel: ");
            Serial.println(autoSpeed);
            moveForward(autoSpeed);
            break;
    }
}



// 🎯 MOVIMIENTOS ACTUALIZADOS PARA L298N
void MotorController::moveForward(int speed) {
    // Motor izquierdo: adelante
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, speed);
    
    // Motor derecho: adelante
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, speed);
}

void MotorController::moveBackward(int speed) {
    // Motor izquierdo: atrás
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, speed);
    
    // Motor derecho: atrás
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, speed);
}

void MotorController::turnLeft(int speed) {
    // Motor izquierdo: atrás (o parado para giro suave)
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, speed);
    
    // Motor derecho: adelante
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    analogWrite(enB, speed);
}

void MotorController::turnRight(int speed) {
    // Motor izquierdo: adelante
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, speed);
    
    // Motor derecho: atrás (o parado para giro suave)
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    analogWrite(enB, speed);
}

void MotorController::stop() {
    analogWrite(enA, 0);
    analogWrite(enB, 0);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
}

// VERIFICACIONES DE SEGURIDAD
bool MotorController::isSafetyTimeout() {
    return (millis() - lastCommandTime) > SAFETY_TIMEOUT;
}

void MotorController::resetSafetyTimer() {
    lastCommandTime = millis();
}