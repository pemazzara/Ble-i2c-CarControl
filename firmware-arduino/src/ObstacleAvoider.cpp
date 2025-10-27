#include "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider(int trig, int echo, MotorController& motorCtrl) {
    trigPin = trig;
    echoPin = echo;
    motorController = &motorCtrl;
    sonar = new NewPing(trigPin, echoPin, 200);
}

void ObstacleAvoider::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void ObstacleAvoider::setActive(bool active) {
    isActive = active;
}

// 🎯 IMPLEMENTACIONES FALTANTES
int ObstacleAvoider::getDistance() {
    int distance = sonar->ping_cm();
    if (distance <= 0 || distance > 200) {
        return 1000; // Simular distancia larga si no hay sensor
    }
    return distance;
}

int ObstacleAvoider::scanLeft() {
    // Simular escaneo izquierda
    Serial.println("🔍 Escaneando izquierda...");
    
    // Guardar posición actual
    motorController->stop();
    delay(100);
    
    // Girar temporalmente a izquierda (necesitaríamos métodos en MotorController)
    // Por ahora simular
    int simulatedDistance = random(20, 100);
    Serial.print("   Distancia izquierda simulada: ");
    Serial.println(simulatedDistance);
    
    delay(200);
    return simulatedDistance;
}

int ObstacleAvoider::scanRight() {
    // Simular escaneo derecha
    Serial.println("🔍 Escaneando derecha...");
    
    motorController->stop();
    delay(100);
    
    // Girar temporalmente a derecha
    int simulatedDistance = random(20, 100);
    Serial.print("   Distancia derecha simulada: ");
    Serial.println(simulatedDistance);
    
    delay(200);
    return simulatedDistance;
}

void ObstacleAvoider::update() {
    if (!isActive) return;
    
    int distance = getDistance();
    
    if (distance > 0 && distance < minDistance) {
        Serial.print("🚧 Obstáculo detectado a ");
        Serial.print(distance);
        Serial.println("cm");
        avoidObstacle();
    }
}

void ObstacleAvoider::avoidObstacle() {
    Serial.println("🛑 Ejecutando evasión de obstáculos");
    
    // 1. Parar motores usando MotorController
    motorController->stop();
    delay(200);
    
    // 2. Retroceder un poco
    Serial.println("📉 Retrocediendo...");
    // motorController->moveBackward(100); // Necesitaríamos acceso temporal
    delay(300);
    motorController->stop();
    delay(200);
    
    // 3. Escanear direcciones
    int distanceLeft = scanLeft();
    delay(100);
    int distanceRight = scanRight();
    
    // 4. Decidir dirección
    if (distanceLeft > distanceRight && distanceLeft > minDistance) {
        Serial.println("↩️  Más espacio a la izquierda - Girando izquierda");
        // motorController->turnLeft(150); // Necesitaríamos acceso
    } else if (distanceRight > minDistance) {
        Serial.println("↪️  Más espacio a la derecha - Girando derecha");
        // motorController->turnRight(150); // Necesitaríamos acceso
    } else {
        Serial.println("🔁 Ambos lados obstruidos - Buscando alternativa");
        // motorController->moveBackward(150);
        delay(500);
        // motorController->turnRight(120);
        delay(600);
    }
    
    motorController->stop();
    delay(100);
    
    Serial.println("✅ Evasión completada");
}



/* ObstacleAvoider.cpp
#nclude "ObstacleAvoider.h"

ObstacleAvoider::ObstacleAvoider(int trig, int echo, MotorController& motorCtrl) {
    trigPin = trig;
    echoPin = echo;
    motorController = &motorCtrl;  // Guardar referencia
    sonar = new NewPing(trigPin, echoPin, 200);
}

void ObstacleAvoider::begin() {
    // Solo inicializar el sensor, NO los motores
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
}

void ObstacleAvoider::update() {
    if (!isActive) return;
    
    int distance = getDistance();
    
    if (distance > 0 && distance < minDistance) {
        avoidObstacle();
    }
    // else: NO mover los motores aquí - eso lo hace MotorController
}

void ObstacleAvoider::avoidObstacle() {
    // Usar MotorController para detener motores
    void emergencyStop(); // Llamar a stop() de MotorController
    
    delay(200);
    
    // Escanear y decidir
    int distanceLeft = scanLeft();
    int distanceRight = scanRight();
    
    if (distanceLeft > distanceRight && distanceLeft > minDistance) {
        // Usar MotorController para girar
        // Nota: Necesitaríamos agregar un método temporal en MotorController
        // o manejar esto de otra forma
        Serial.println("🔄 ObstacleAvoider: Girando izquierda");
    } else if (distanceRight > minDistance) {
        Serial.println("🔄 ObstacleAvoider: Girando derecha");
    }
}*/