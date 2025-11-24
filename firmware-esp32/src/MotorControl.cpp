// MotorControl.cpp
#include <Arduino.h>
#include "MotorControl.h"
#include "Command.h" // Incluir los nuevos comandos


void MotorControl::begin() {

    Serial.println("✅ MotorControl listo - Comandos via I2C maestro");
    
    // Inicializar motor detenido
    stopMotor();
    currentManualSpeed = 0; 
}

void MotorControl::sendI2CCommand(uint8_t command, uint8_t value) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(command);
    byte error = Wire.endTransmission();
    
    if(error != 0) {
        Serial.printf("⚠️  Error enviando comando %d a Arduino: %d\n", command, error);
    }
}

void MotorControl::moveForward(int speed) {
    // Primero configurar la velocidad si es diferente
    if (currentManualSpeed != speed) {
        setManualSpeed(speed);
        currentManualSpeed = speed;
    }
    // Luego enviar comando de movimiento
    sendI2CCommand(CMD_MANUAL_FORWARD);
}

void MotorControl::moveBackward(int speed) {
    if (currentManualSpeed != speed) {
        setManualSpeed(speed);
        currentManualSpeed = speed;
    }
    sendI2CCommand(CMD_MANUAL_BACKWARD);
}

void MotorControl::turnLeft() {
    sendI2CCommand(CMD_MANUAL_LEFT);
}

void MotorControl::turnRight() {
    sendI2CCommand(CMD_MANUAL_RIGHT);
}

void MotorControl::stopMotor() {
    sendI2CCommand(CMD_STOP);
}

void MotorControl::enableObstacleAvoidance() {
    sendI2CCommand(CMD_OBSTACLE_AVOIDANCE);
}

void MotorControl::enableWallFollowing() {
    sendI2CCommand(CMD_WALL_FOLLOWING);
}

void MotorControl::emergencyStop() {
    sendI2CCommand(CMD_EMERGENCY_STOP);
}

void MotorControl::setManualSpeed(int speed) {
    sendI2CCommand(CMD_SET_MANUAL_SPEED, speed);
}

void MotorControl::setAvoidanceParameters(int minDistance, int speed, int turnSpeed) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(CMD_SET_AVOIDANCE_PARAMS);
    Wire.write(minDistance);
    Wire.write(speed);
    Wire.write(turnSpeed);
    Wire.endTransmission();
}

void MotorControl::setWallFollowParameters(int speed, int idealDistance) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(CMD_SET_WALL_FOLLOW_PARAMS);
    Wire.write(speed);
    Wire.write(idealDistance);
    Wire.endTransmission();
}

// Eliminar las funciones de giro suave o reimplementarlas
void MotorControl::softTurnLeft() {
    // Implementar giro suave como una combinación de comandos
    // o eliminar esta función si no está en los comandos del Arduino
    turnLeft(); // Usar giro normal por ahora
}

void MotorControl::softTurnRight() {
    turnRight(); // Usar giro normal por ahora
}

void MotorControl::setSonarState(bool enabled) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(CMD_SET_SONAR_STATE);
    Wire.write(enabled ? 1 : 0);
    Wire.endTransmission();
    sonarEnabled = enabled;
}

void MotorControl::enableAutonomousMode() {
    sendI2CCommand(CMD_AUTO_NAVIGATE);
}
void MotorControl::sendAutonomousCommand(byte command) {
    // Validar que el comando sea autónomo
    if (command >= CMD_AUTO_FORWARD && command <= CMD_AUTO_NAVIGATE) {
        sendI2CCommand(command);
    } else {
        Serial.println("❌ Comando autónomo no válido");
    }
}

// MotorControl.cpp