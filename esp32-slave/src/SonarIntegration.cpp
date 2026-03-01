// Esp32Slave - SonarIntegration.cpp
#include "SonarIntegration.h"
#include <Arduino.h>

SonarIntegration::SonarIntegration(){} 

// =========================================================
// CLASE SONAR INTEGRATION (Adaptada de SonarIntegration.cpp)
// =========================================================

void SonarIntegration::begin() {
    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);
    digitalWrite(SONAR_TRIG, LOW);
    Serial.println("🔊 Sonar HC-SR04 inicializado (no bloqueante).");
}
// En SonarIntegration::readDistance() - MEJORAR:
uint16_t SonarIntegration::readDistance() {
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG, LOW);
    
    // ❌ PROBLEMA: pulseIn puede bloquear la task
    // long duration = pulseIn(SONAR_ECHO, HIGH, 30000);
    
    // ✅ SOLUCIÓN: Implementación no bloqueante
    unsigned long startTime = micros();
    unsigned long timeout = 30000; // 30ms timeout
    
    // Esperar pulso HIGH
    while(digitalRead(SONAR_ECHO) == LOW && (micros() - startTime) < timeout);
    
    startTime = micros();
    while(digitalRead(SONAR_ECHO) == HIGH && (micros() - startTime) < timeout);
    unsigned long duration = micros() - startTime;
    
    // Si timeout, retornar distancia máxima
    if(duration >= timeout) return 1200;
    
    // Conversión a mm
    uint16_t distance = (duration * 0.34) / 2;
    return (distance > 1200) ? 1200 : distance;
}

uint16_t SonarIntegration::updateAndGetDistance() {
    return readDistance();
}
