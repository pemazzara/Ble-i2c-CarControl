// SensorFusion.h
#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "SensorControl.h"
#include "Command.h" 

enum OperationMode {
  MODE_EMERGENCY_STOP,
  MODE_MANUAL,
  MODE_AUTONOMOUS,
  MODE_OBSTACLE_AVOIDANCE
};
class SensorFusion {
private:
    SensorControl* sensors;
    
    // Parámetros de navegación
    uint16_t safeDistance = 400;      // 40cm - distancia segura
    uint16_t warningDistance = 250;   // 25cm - advertencia
    uint16_t emergencyDistance = 150; // 15cm - emergencia
    
    // Historial para decisiones más inteligentes
    int lastDecision = 0;
    unsigned long lastObstacleTime = 0;
    bool obstacleMemory[3] = {false, false, false}; // F, L, R
    
public:
    SensorFusion(SensorControl& sensorCtrl);
    int scanLeftDetailed();
    int scanRightDetailed(); 
    int scanFrontDetailed();
        // Escaneo con giro físico (si tu hardware lo permite)
    int performPhysicalScan(int degrees);
    
    // Getters para datos crudos
    uint16_t getFrontDistance() const { return sensors->frontDistance; }
    uint16_t getLeftDistance() const { return sensors->leftDistance; }
    uint16_t getRightDistance() const { return sensors->rightDistance; }
    // Toma de decisiones
    byte calculateBestCommand();
    bool shouldEmergencyStop();
    bool isPathClear();
    
    // Análisis de entorno
    int getBestDirection();
    float calculatePathSafety();
    
    // Configuración
    void setSafetyDistances(uint16_t safe, uint16_t warn, uint16_t emerg);
    
private:
    void updateObstacleMemory();
    bool isStuckInLoop();
    int evaluateDirection(int leftDist, int rightDist);
};

#endif