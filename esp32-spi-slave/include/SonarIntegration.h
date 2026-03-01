// Arduino - SonarIntegration.h
#ifndef SONAR_INTEGRATION_H
#define SONAR_INTEGRATION_H

#include <Arduino.h>
#include "SPIDefinitions.h"
#include <driver/rmt.h>


// Pines Sonar (HC-SR04)
const uint8_t SONAR_TRIG_PIN = 20;
const uint8_t SONAR_ECHO_PIN = 21;
const uint8_t EMERGENCY_THRESHOLD = 3; // 3 lecturas seguidas
#define FILTER_SIZE 5      // Tamaño del filtro de media móvil
//#define RMT_TX_CHANNEL RMT_CHANNEL_0  // Canal para el Trigger
#define RMT_RX_CHANNEL RMT_CHANNEL_4  // Canal para el Echo
#define RMT_CLK_DIV 80                 // 80MHz / 80 = 1 tick por microse
class SonarIntegration {
public:
    SonarIntegration(uint8_t trigPin = SONAR_TRIG_PIN, 
                     uint8_t echoPin = SONAR_ECHO_PIN);
    
    void begin();
    uint16_t triggerAndReadRMT();
    bool isEmergency();
    bool isSensorOK();
    void updateSonarData();
    void update();
    // Getters                // Diagnóstico
    bool getLatestSonarData(SonarSensorData_t &data);
    uint16_t getLastDistance();
    bool initialized;
private:
    uint8_t SONAR_TRIG;
    uint8_t SONAR_ECHO;
    SonarSensorData_t sonar_data;
    SemaphoreHandle_t data_mutex;
    uint16_t duration = 0;
    uint16_t distanceBuffer[FILTER_SIZE];
    uint8_t emergency_counter = 0;
};

#endif
