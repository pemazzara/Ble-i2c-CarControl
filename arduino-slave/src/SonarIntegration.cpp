// Arduino - SonarIntegration.cpp
#include "SonarIntegration.h"
#include <Arduino.h>

SonarIntegration::SonarIntegration(){} 

void SonarIntegration::update() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastSonarRead >= SONAR_INTERVAL) {
        lastDistance = readDistance();
        lastSonarRead = currentTime;
        
        // ❌ ELIMINAR envío automático al ESP32
        // Los datos se enviarán cuando el ESP32 los solicite
        
        // Solo mantener emergency flag local
        if (lastDistance < 50) {
            emergencyFlag = true;
            Serial.println("🚨 EMERGENCY STOP LOCAL - Obstáculo a " + String(lastDistance) + "mm");
        }
    }
}

void SonarIntegration::begin() {

    pinMode(SONAR_TRIG_PIN, OUTPUT);
    pinMode(SONAR_ECHO_PIN, INPUT);
    
    //Wire.begin(ESP32_I2C_ADDRESS); // Se hace en main.cpp
    Wire.onRequest([]() { 
        // Necesitamos una referencia a la instancia - esto se manejará en main.cpp
    });
    Wire.onReceive([](int byteCount) { 
        // Necesitamos una referencia a la instancia - esto se manejará en main.cpp  
    });
    
    Serial.println("🔊 Sonar HC-SR04 inicializado + I2C esclavo listo");
}



uint16_t SonarIntegration::readDistance() {
    digitalWrite(SONAR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG_PIN, LOW);
    
    long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 30000);
    uint16_t distance = (duration > 0) ? (duration * 0.34) / 2 : 1200;
    
    if (distance > 1200) distance = 1200;
    return distance;
}

uint16_t SonarIntegration::getDistance() {
    return lastDistance;
}
/*
void SonarIntegration::sendToESP32(uint8_t command, uint16_t data) {
    Wire.beginTransmission(0x09);
    Wire.write(command);
    Wire.write(highByte(data));
    Wire.write(lowByte(data));
    Wire.endTransmission();
}
*/
void SonarIntegration::handleI2CRequest() {
    Wire.write(highByte(lastDistance));
    Wire.write(lowByte(lastDistance));
    Wire.write(emergencyFlag ? 1 : 0);
}

void SonarIntegration::handleI2CReceive(int byteCount) {
    if (byteCount > 0) {
        uint8_t command = Wire.read();
        
        if (command == 0x20) {
            emergencyFlag = false;
            //Serial.println("🟢 Emergency reset recibido");
        }
    }
}

/* Arduino - SonarIntegration.cpp
#include <Arduino.h>
#include "SonarIntegration.h"
#include <Wire.h>

SonarIntegration sonar;

void SonarIntegration::begin() {
    pinMode(SONAR_TRIG_PIN, OUTPUT);
    pinMode(SONAR_ECHO_PIN, INPUT);
    
    Wire.begin(ESP32_I2C_ADDRESS); // Arduino como esclavo I2C
    Wire.onRequest([]() { sonar.handleI2CRequest(); });
    Wire.onReceive([](int byteCount) { sonar.handleI2CReceive(byteCount); });
    
    Serial.println("🔊 Sonar HC-SR04 inicializado + I2C esclavo listo");
}

void SonarIntegration::update() {
    unsigned long currentTime = millis();
    
    if (currentTime - lastSonarRead >= SONAR_INTERVAL) {
        lastDistance = readDistance();
        lastSonarRead = currentTime;
        
        // Enviar actualización automática al ESP32
        sendToESP32(I2C_SONAR_UPDATE, lastDistance);
        
        // Emergency stop local independiente
        if (lastDistance < 50) { // 5cm - EMERGENCIA
            emergencyFlag = true;
            sendToESP32(I2C_EMERGENCY_STOP, lastDistance);
            Serial.println("🚨 EMERGENCY STOP LOCAL - Obstáculo a " + String(lastDistance) + "mm");
        }
    }
}

uint16_t SonarIntegration::readDistance() {
    digitalWrite(SONAR_TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG_PIN, LOW);
    
    long duration = pulseIn(SONAR_ECHO_PIN, HIGH, 30000); // Timeout 30ms
    uint16_t distance = (duration > 0) ? (duration * 0.34) / 2 : 1200;
    
    // Filtrar lecturas erróneas
    if (distance > 1200) distance = 1200;
    
    return distance;
}
uint16_t SonarIntegration::getDistance() {
    return lastDistance;
}
void SonarIntegration::sendToESP32(uint8_t command, uint16_t data) {
    Wire.beginTransmission(0x09); // Dirección del ESP32
    Wire.write(command);
    Wire.write(highByte(data));
    Wire.write(lowByte(data));
    Wire.endTransmission();
}

void SonarIntegration::handleI2CRequest() {
    // ESP32 solicita datos - enviar última distancia
    Wire.write(highByte(lastDistance));
    Wire.write(lowByte(lastDistance));
    Wire.write(emergencyFlag ? 1 : 0);
}

void SonarIntegration::handleI2CReceive(int byteCount) {
    if (byteCount > 0) {
        uint8_t command = Wire.read();
        
        if (command == 0x20) { // Comando de reset de emergencia
            emergencyFlag = false;
            Serial.println("🟢 Emergency reset recibido");
        }
    }
}*/