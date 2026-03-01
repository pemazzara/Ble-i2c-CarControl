// Arduino - SonarIntegration.h
#ifndef SONAR_INTEGRATION_H
#define SONAR_INTEGRATION_H

#include <Wire.h>

// Pines Sonar (HC-SR04)
#define SONAR_TRIG 38
#define SONAR_ECHO 37


class SonarIntegration {
private:
    uint16_t readDistance();
public:
    SonarIntegration();
    void begin();
    uint16_t updateAndGetDistance();
};
/*
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
*/
#endif