// Esp32Slave - MotorControl.cpp
#include "MotorControl.h"
#include <cmath> // Para abs()

MotorControl::MotorControl() {}

void MotorControl::begin() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // 2. Configuración de CANALES LEDC (PWM nativo)
    
    // Canal A (Izquierdo)
    ledcSetup(LEDC_CH_A, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(ENA, LEDC_CH_A); 

    // Canal B (Derecho)
    ledcSetup(LEDC_CH_B, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(ENB, LEDC_CH_B);

    stop();
    resetSafetyTimer();
    Serial.println("🚗 MotorControl inicializado.");
}

void MotorControl::handleCommand(byte command, byte data) {
    resetSafetyTimer();
    
    // PRIORIDAD 1: EMERGENCY STOP
    if (command == CMD_EMERGENCY_STOP) {
        emergencyStop(); // Activa la bandera y para motores
        return;
    }
    
    if (emergencyStopActive && command != CMD_STOP) {
        Serial.println("⚠️  Ignorando comando - Sistema en emergencia");
        return;
    }

    if (command == CMD_STOP) {
        if (emergencyStopActive) emergencyStopActive = false; // ¡AQUÍ SE RESETEA LA BANDERA!
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

// En MotorControl.cpp - Agregar función de reset
void MotorControl::resetEmergency() {
    if (emergencyStopActive) {
        emergencyStopActive = false;
        Serial.println("🔄 Emergencia reseteada en Slave");
    }
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
// Nota: Esta función ahora espera la velocidad normalizada (0-1023)
void MotorControl::moveForward(int speed) {
    int duty = map(speed, 0, 255, 0, MAX_DUTY_CYCLE);
    // Motor Izquierdo (Canal A) - Adelante
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
    ledcWrite(LEDC_CH_A, duty);
    //digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, speed);
    // Motor Derecho (Canal B) - Adelante
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    ledcWrite(LEDC_CH_B, duty);
    //digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, speed);
}
void MotorControl::moveBackward(int speed) {
    int duty = map(speed, 0, 255, 0, MAX_DUTY_CYCLE);

    // Motor Izquierdo (Canal A) - Atrás
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
    ledcWrite(LEDC_CH_A, duty);
    
    // Motor Derecho (Canal B) - Atrás
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
    ledcWrite(LEDC_CH_B, duty);
}

void MotorControl::turnLeft(int speed) {
    int duty = map(speed, 0, 255, 0, MAX_DUTY_CYCLE);

    // Izq: Atrás (velocidad media para pivotar)
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); 
    ledcWrite(LEDC_CH_A, duty / 2); // Velocidad media
    
    // Der: Adelante
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    ledcWrite(LEDC_CH_B, duty);
}

void MotorControl::turnRight(int speed) {
    int duty = map(speed, 0, 255, 0, MAX_DUTY_CYCLE);

    // Izq: Adelante
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
    ledcWrite(LEDC_CH_A, duty);
    
    // Der: Atrás (velocidad media para pivotar)
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); 
    ledcWrite(LEDC_CH_B, duty / 2); // Velocidad media
}

void MotorControl::stop() {
    // Apagar PWM
    ledcWrite(LEDC_CH_A, 0); 
    ledcWrite(LEDC_CH_B, 0);
    
    // Poner pines de dirección en LOW/LOW (freno dinámico si el driver lo soporta, o coast)
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ⚠️ Usar setMotorSpeeds requiere también ajustar el PWM, no solo analogWrite.
void MotorControl::setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Suavizado de velocidad (se asume que leftSpeed/rightSpeed están en rango 0-255)
    currentLeftSpeed = smoothSpeed(currentLeftSpeed, leftSpeed);
    currentRightSpeed = smoothSpeed(currentRightSpeed, rightSpeed);
    
    // Convertir de 0-255 a 0-1023 (MAX_DUTY_CYCLE)
    int dutyLeft = map(currentLeftSpeed, 0, 255, 0, MAX_DUTY_CYCLE);
    int dutyRight = map(currentRightSpeed, 0, 255, 0, MAX_DUTY_CYCLE);
    
    ledcWrite(LEDC_CH_A, dutyLeft);
    ledcWrite(LEDC_CH_B, dutyRight);
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

