#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
// No incluir SonarIntegration.h - usar declaración adelantada

class SonarIntegration;  // Declaración adelantada

typedef struct {
    uint16_t distance;
    uint16_t raw_distance;
    bool emergency;
    bool sensor_ok;
    uint32_t timestamp;
    uint8_t error_count;
} SonarSensorData_t;

class SensorManager {
private:
    SonarSensorData_t sonar_data;
    SemaphoreHandle_t data_mutex;
    SonarIntegration* sonar_ref;  // Referencia al sonar existente
    
    
public:
    // Constructor recibe referencia al sonar existente
    SensorManager(SonarIntegration* sonar = nullptr);
    bool initialized;
    
    bool init();
    void update();
    bool getLatestSonarData(SonarSensorData_t &data);
    
    // Getters
    uint16_t getLastDistance();
    bool isEmergency();
    bool isSensorOK();
    
private:
    void updateSonarData();
};

#endif

/*
#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

#include <Arduino.h>
#include "SonarIntegration.h"

typedef struct {
    uint16_t distance;      // Distancia en mm (filtrada)
    uint16_t raw_distance;  // Distancia raw (sin filtrar)
    bool emergency;         // Emergencia detectada (< 200mm)
    bool sensor_ok;         // Sensor funcionando correctamente
    uint32_t timestamp;     // Cuando se tomó la lectura
    uint8_t error_count;    // Contador de errores
} SonarSensorData_t;

class SensorManager {
private:
    SonarSensorData_t sonar_data;
    SemaphoreHandle_t data_mutex;
    SonarIntegration* sonar;
    bool initialized;
    
public:
    SensorManager();
    ~SensorManager();
    
    bool init(uint8_t trig_pin = 38, uint8_t echo_pin = 37);
    void update();  // Llamar periódicamente para actualizar datos
    bool getLatestSonarData(SonarSensorData_t &data);
    
    // Métodos de utilidad
    uint16_t getLastDistance() { return sonar_data.distance; }
    bool isEmergency() { return sonar_data.emergency; }
    bool isSensorOK() { return sonar_data.sensor_ok; }
    
private:
    void updateSonarData();
};

#endif
*/