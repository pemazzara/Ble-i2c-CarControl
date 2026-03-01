// SensorFusion.cpp
#include <Arduino.h>
#include "SensorFusion.h"
#include "Command.h" // Incluir comandos

SensorFusion::SensorFusion(SensorControl& sensorCtrl) {
    sensors = &sensorCtrl;
}

// En SensorFusion.cpp - corregir la función calculateBestCommand()
byte SensorFusion::calculateBestCommand() {
    sensors->readAll();
    updateObstacleMemory();
    
    // 1. Verificar parada de emergencia primero
    if (shouldEmergencyStop()) {
        return CMD_EMERGENCY_STOP;
    }
    
    // 2. Análisis de seguridad del camino
    float pathSafety = calculatePathSafety();
    
    // 3. Tomar decisión basada en el entorno
    int bestDir = getBestDirection();
    
    // 4. Prevenir loops
    if (isStuckInLoop()) {
        Serial.println("🔄 SensorFusion: Detectado loop - retrocediendo");
        return CMD_AUTO_BACKWARD;
    }
    
    // 5. Ejecutar decisión con comandos que SÍ existen
    switch(bestDir) {
        case 0: // Adelante
            if (pathSafety > 0.7f) {
                return CMD_AUTO_FORWARD;
            } else {
                // Navegación cautelosa - reducir velocidad
                // Podemos usar CMD_AUTO_FORWARD con velocidad configurada
                return CMD_AUTO_FORWARD;
            }
            
        case 1: // Izquierda
            return CMD_AUTO_LEFT;
            
        case 2: // Derecha
            return CMD_AUTO_RIGHT;
            
        case -1: // Retroceder/reorientar
            return CMD_AUTO_BACKWARD;
            
        default:
            return CMD_STOP; // Este ahora existe
    }
}


bool SensorFusion::shouldEmergencyStop() {
    return (sensors->frontDistance < emergencyDistance || 
            sensors->leftDistance < emergencyDistance / 2 || 
            sensors->rightDistance < emergencyDistance / 2);
}

bool SensorFusion::isPathClear() {
    return (sensors->frontDistance > safeDistance &&
            sensors->leftDistance > warningDistance &&
            sensors->rightDistance > warningDistance);
}

int SensorFusion::getBestDirection() {
    int front = sensors->frontDistance;
    int left = sensors->leftDistance;
    int right = sensors->rightDistance;
    
    // Prioridad 1: Camino frontal despejado
    if (front > safeDistance) {
        // Verificar si hay obstáculos laterales cercanos
        if (left < warningDistance && right > safeDistance) {
            return 2; // Derecha para esquivar obstáculo izquierdo
        } else if (right < warningDistance && left > safeDistance) {
            return 1; // Izquierda para esquivar obstáculo derecho
        } else {
            return 0; // Adelante - camino óptimo
        }
    }
    
    // Prioridad 2: Decidir giro cuando hay obstáculo frontal
    return evaluateDirection(left, right);
}

int SensorFusion::evaluateDirection(int leftDist, int rightDist) {
    // No solo comparar distancias, sino también calidad del camino
    int leftScore = leftDist;
    int rightScore = rightDist;
    
    // Penalizar direcciones con obstáculos recientes
    if (obstacleMemory[1]) leftScore *= 0.7f;  // Izquierda con historial malo
    if (obstacleMemory[2]) rightScore *= 0.7f; // Derecha con historial malo
    
    // Preferir dirección con mejor score
    if (leftScore > rightScore && leftScore > warningDistance) {
        return 1; // Izquierda
    } else if (rightScore > warningDistance) {
        return 2; // Derecha
    } else {
        return -1; // Retroceder
    }
}

float SensorFusion::calculatePathSafety() {
    float frontSafety = (float)sensors->frontDistance / safeDistance;
    float leftSafety = (float)sensors->leftDistance / safeDistance;
    float rightSafety = (float)sensors->rightDistance / safeDistance;
    
    // Safety general (0-1) donde 1 es completamente seguro
    float overallSafety = (frontSafety + leftSafety + rightSafety) / 3.0f;
    return constrain(overallSafety, 0.0f, 1.0f);
}

void SensorFusion::updateObstacleMemory() {
    // Actualizar memoria de obstáculos (últimas 3 lecturas)
    obstacleMemory[0] = (sensors->frontDistance < warningDistance);
    obstacleMemory[1] = (sensors->leftDistance < warningDistance);
    obstacleMemory[2] = (sensors->rightDistance < warningDistance);
    
    if (sensors->frontDistance < emergencyDistance) {
        lastObstacleTime = millis();
    }
}

bool SensorFusion::isStuckInLoop() {
    // Detectar si estamos girando en el mismo lugar
    static unsigned long lastDirectionChange = 0;
    static int consecutiveSameDecision = 0;
    
    if (millis() - lastDirectionChange > 5000) { // 5 segundos
        consecutiveSameDecision = 0;
    }
    
    int currentDecision = getBestDirection();
    if (currentDecision == lastDecision) {
        consecutiveSameDecision++;
    } else {
        consecutiveSameDecision = 0;
        lastDirectionChange = millis();
    }
    
    lastDecision = currentDecision;
    
    // Si tomamos la misma decisión muchas veces consecutivas
    return (consecutiveSameDecision > 8);
}

void SensorFusion::setSafetyDistances(uint16_t safe, uint16_t warn, uint16_t emerg) {
    safeDistance = safe;
    warningDistance = warn;
    emergencyDistance = emerg;
}

int SensorFusion::scanLeftDetailed() {
    Serial.println("🔍 Escaneo izquierda DETALLADO...");
    
    // Usar el sensor izquierdo real (ya tienes esta funcionalidad)
    sensors->readSensor(1); // Índice 1 = izquierdo
    int realDistance = sensors->leftDistance;
    
    // Análisis adicional del lado izquierdo
    float leftSafety = (float)realDistance / safeDistance;
    
    Serial.printf("   📏 Izquierda: %dmm | Seguridad: %.1f%%\n", 
                  realDistance, leftSafety * 100);
    
    return realDistance;
}

int SensorFusion::scanRightDetailed() {
    Serial.println("🔍 Escaneo derecha DETALLADO...");
    
    sensors->readSensor(2); // Índice 2 = derecho
    int realDistance = sensors->rightDistance;
    
    float rightSafety = (float)realDistance / safeDistance;
    
    Serial.printf("   📏 Derecha: %dmm | Seguridad: %.1f%%\n", 
                  realDistance, rightSafety * 100);
    
    return realDistance;
}

int SensorFusion::scanFrontDetailed() {
    Serial.println("🔍 Escaneo frontal DETALLADO...");
    
    sensors->readSensor(0); // Índice 0 = frontal
    int realDistance = sensors->frontDistance;
    
    float frontSafety = (float)realDistance / safeDistance;
    String safetyLevel = (frontSafety > 0.8) ? "ALTA" : 
                        (frontSafety > 0.5) ? "MEDIA" : "BAJA";
    
    Serial.printf("   📏 Frontal: %dmm | Seguridad: %s\n", 
                  realDistance, safetyLevel.c_str());
    
    return realDistance;
}

int SensorFusion::performPhysicalScan(int degrees) {
    // Si tu robot tiene capacidad de girar el sensor o el chasis
    Serial.printf("🔄 Escaneo físico de %d grados...\n", degrees);
    
    // Esta función requeriría integración con MotorControl
    // Para girar el robot y escanear múltiples ángulos
    
    uint16_t minDistance = 1200; // Cambiar a uint16_t para coincidir con el tipo
    int steps = abs(degrees) / 10;
    
    for (int i = 0; i < steps; i++) {
        // Girar 10 grados (depende de tu implementación de motores)
        // motor.turnRight(10); 
        delay(100);
        
        sensors->readSensor(0); // Leer frontal en nueva orientación
        
        // Usar la función min de Arduino o convertir tipos
        minDistance = (sensors->frontDistance < minDistance) ? sensors->frontDistance : minDistance;
        
        Serial.printf("   Ángulo %d°: %dmm\n", (i+1)*10, sensors->frontDistance);
    }
    
    // Regresar a posición original
    // motor.turnLeft(degrees);
    
    Serial.printf("📍 Distancia mínima en escaneo: %dmm\n", minDistance);
    return minDistance;
}

/*
int SensorFusion::performPhysicalScan(int degrees) {
    // Si tu robot tiene capacidad de girar el sensor o el chasis
    Serial.printf("🔄 Escaneo físico de %d grados...\n", degrees);
    
    // Esta función requeriría integración con MotorControl
    // Para girar el robot y escanear múltiples ángulos
    
    int minDistance = 1200; // Valor máximo
    int steps = abs(degrees) / 10;
    
    for (int i = 0; i < steps; i++) {
        // Girar 10 grados (depende de tu implementación de motores)
        // motor.turnRight(10); 
        delay(100);
        
        sensors->readSensor(0); // Leer frontal en nueva orientación
        minDistance = min(minDistance, sensors->frontDistance);
        
        Serial.printf("   Ángulo %d°: %dmm\n", (i+1)*10, sensors->frontDistance);
    }
    
    // Regresar a posición original
    // motor.turnLeft(degrees);
    
    Serial.printf("📍 Distancia mínima en escaneo: %dmm\n", minDistance);
    return minDistance;
}*/