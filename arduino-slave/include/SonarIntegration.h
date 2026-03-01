// Arduino - SonarIntegration.h
#ifndef SONAR_INTEGRATION_H
#define SONAR_INTEGRATION_H

#include <Wire.h>

#define SONAR_TRIG_PIN 12
#define SONAR_ECHO_PIN 13
#define ESP32_I2C_ADDRESS 0x08
#define I2C_SONAR_UPDATE 0x10
#define I2C_EMERGENCY_STOP 0x11

class SonarIntegration {
private:
    unsigned long lastSonarRead = 0;
    const unsigned long SONAR_INTERVAL = 50; // 50ms entre lecturas
    uint16_t lastDistance = 0;
    bool emergencyFlag = false;

public:
    SonarIntegration();
    void begin();
    void update();
    uint16_t readDistance();
    uint16_t getDistance();
    void sendToESP32(uint8_t command, uint16_t data = 0);
    void handleI2CRequest();
    void handleI2CReceive(int byteCount);
};

//extern SonarIntegration sonar;

#endif