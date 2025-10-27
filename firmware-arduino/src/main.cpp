// main.cpp
#include <Arduino.h>
#include <Wire.h>
#include "MotorController.h"
#include "ObstacleAvoider.h"


#define I2C_ADDR 0x08



void handleI2CCommand(int howMany);

// Instancias actualizada con pines L298N
MotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4);
ObstacleAvoider obstacleAvoider(TRIG_PIN, ECHO_PIN, motorController);


void setup() {
    Serial.begin(9600);
    
    // Inicializar control de motores
    motorController.begin();
    obstacleAvoider.begin();  
    // Configurar I2C
    Wire.begin(I2C_ADDR);
    Wire.onReceive(handleI2CCommand);
    
    Serial.println("🚗 Sistema Arduino listo - Prioridades activas");
}

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
    
    // 2. Solo ejecutar evasión de obstáculos si estamos en modo AUTO
    // y no hay emergencia Y el sonar está habilitado
    if (motorController.getCurrentMode() == MODE_AUTONOMOUS && 
        !motorController.isInEmergency() &&
        motorController.isSonarEnabled()) {
        obstacleAvoider.update();
        }
    delay(50);
}
