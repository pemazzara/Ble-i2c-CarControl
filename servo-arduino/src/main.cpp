// Arduino main.cpp - VERSIÓN CORREGIDA
#include <Arduino.h>
#include <Wire.h>
#include "MotorController.h"
#include "SonarIntegration.h"

// Instancias 
MotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4);
SonarIntegration sonar;

// Variables para compartir datos con ESP32
volatile uint16_t sharedSonarDistance = 0;
volatile bool sharedEmergencyFlag = false;
bool globalEmergencyStop = false;

// I2C Address
// Dirección I2C del Esclavo. Debe ser única en el bus (ej. 0x08, 8 en decimal).
#define SLAVE_ADDRESS 8

// Variable para almacenar el comando recibido.
int comandoRecibido = 0;

void handleI2CRequest() {
    // ✅ ESP32 solicita datos - enviar sonar + emergency
    Wire.write(highByte(sharedSonarDistance));
    Wire.write(lowByte(sharedSonarDistance));
    Wire.write(sharedEmergencyFlag ? 1 : 0);
    
    // Reset emergency después de reportarla
    sharedEmergencyFlag = false;
}

void handleI2CReceive(int byteCount) {
    // ✅ ESP32 envía comando - procesarlo
    if(byteCount > 0) {
        byte command = Wire.read();
        motorController.handleCommand(command);
    }
}

void updateSharedData() {
    // Actualizar datos compartidos desde las lecturas locales
    sharedSonarDistance = sonar.getDistance();
    
    // Solo set emergency flag, no ejecutar emergencyStop local
    if(sharedSonarDistance < 100) {
        sharedEmergencyFlag = true;
    }
}

void emergencyStop() {
    motorController.emergencyStop();
    globalEmergencyStop = true;
    Serial.println("🚨 EMERGENCY STOP ACTIVADO");
}

void resetEmergencyStop() {
    globalEmergencyStop = false;
    Serial.println("🟢 EMERGENCY STOP RESETEADO");
}

void setup() {
    Serial.begin(9600);
    // Inicializar componentes
    motorController.begin();  
    sonar.begin();

    Wire.begin(SLAVE_ADDRESS); // Inicia Wire como Esclavo con su dirección
    //Wire.onRequest(handleI2CRequest);
    Wire.onReceive(handleI2CReceive);

    Serial.println("🚗 Arduino Listo - Modo Esclavo I2C (0x08)");
    Serial.println("   - Esperando comandos del ESP32");
    Serial.println("   - Respondiendo solicitudes de datos");
    Serial.println("🚗 Arduino Ejecutor listo - Sonar + I2C activo");
}

void loop() {
    // ✅ ARDUINO NUNCA INICIA COMUNICACIÓN
    // Solo actualiza sensores y estado local
    
    motorController.updateSafety();
    sonar.update();
    updateSharedData();
    // 2. Detección proactiva de obstáculos lejanos
/*
    if (sharedSonarDistance < 400) {
        sendSonarAlertToESP32(sonarDist);
    }
*/  
    // 3. Emergency stop independiente
    if ( sharedSonarDistance < 100) {
        emergencyStop();
    }
    
    delay(10);
}
/* Se eliminó esta función ya que ahora es Esp32 quien pide los datos
void sendSonarAlertToESP32(uint16_t distance) {
    Wire.beginTransmission(0x09);
    Wire.write(0x40);
    Wire.write(highByte(distance));
    Wire.write(lowByte(distance));
    Wire.endTransmission();
}*/

/* Arduino main.cpp
#include <Arduino.h>
#include <Wire.h>
#include "MotorController.h"
#include "SonarIntegration.h"


bool globalEmergencyStop = false;

void sendSonarAlertToESP32(uint16_t distance);
void sendEmergencyToESP32();
void handleI2CCommand(int howMany);

// Instancias actualizada con pines L298N
MotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4);

void emergencyStop() {
    motorController.emergencyStop();
    globalEmergencyStop = true;
    Serial.println("🚨 EMERGENCY STOP ACTIVADO");
    
    // Notificar al ESP32
    sonar.sendToESP32(0x11, 0); // Código de emergencia
}

// ✅ FUNCIÓN PARA RESETEAR EMERGENCIA
void resetEmergencyStop() {
    globalEmergencyStop = false;
    Serial.println("🟢 EMERGENCY STOP RESETEADO");
}

void setup() {
    Serial.begin(9600);
    
    // Inicializar control de motores
    motorController.begin();
    sonar.begin();  
    // Configurar I2C Ya configurado en SonarIntegration
    //Wire.begin(I2C_ADDR);
    //Wire.onReceive(handleI2CCommand);
    Serial.println("🚗 Arduino Ejecutor listo - Sonar + I2C activo");
    Serial.println("🚗 Prioridades activas");
}
// Se pasó a SonarIntegration.cpp
void handleI2CCommand(int howMany) {
    if (howMany >= 1) {
        byte command = Wire.read();
        byte data = 0;
        
        // Solo leer data si es un comando que la necesita
        if ((command == CMD_SET_MANUAL_SPEED || command == CMD_SET_AUTO_SPEED) && 
            howMany >= 2) {
            data = Wire.read();
        }
        
        motorController.handleCommand(command, data);
    }
}

void loop() {
    // 1. Seguridad continua del motor controller
    motorController.updateSafety();
    sonar.update();

    // Detección proactiva de obstáculos lejanos
    uint16_t sonarDist = sonar.getDistance();

    if (sonarDist < 400) { // 40cm - enviar alerta preventiva
        sendSonarAlertToESP32(sonarDist);
            // Emergency stop local independiente
    
    if (sonarDist < 100) {
        emergencyStop();
        //sendEmergencyToESP32(); // Ya se envía en emergencyStop()
    }

    }
    if (globalEmergencyStop) {
        motorController.emergencyStop();
        return;
    }

    delay(10);
}
void sendSonarAlertToESP32(uint16_t distance) {
    Wire.beginTransmission(0x09);
    Wire.write(0x40); // Código de alerta preventiva
    Wire.write(highByte(distance));
    Wire.write(lowByte(distance));
    Wire.endTransmission();
}*/