// Esp32Slave - SonarIntegration.cpp
#include "SonarIntegration.h"
#include <Arduino.h>

SonarIntegration::SonarIntegration(uint8_t trigPin, uint8_t echoPin) 
    : SONAR_TRIG(trigPin), SONAR_ECHO(echoPin) {
    lastValidDistance = 1200; // Valor por defecto
    errorCount = 0;
    // Inicializar estructura de datos
    memset(&sonar_data, 0, sizeof(sonar_data));
    sonar_data.sensor_ok = false;
    sonar_data.distance = 1200;  // Valor por defecto
    sonar_data.raw_distance = 1200;
    sonar_data.timestamp = 0;
    sonar_data.error_count = 0;
    for(int i = 0; i < FILTER_SIZE; i++) distanceBuffer[i] = 1200;
}

void SonarIntegration::begin() {
    
    pinMode(SONAR_TRIG, OUTPUT);
    pinMode(SONAR_ECHO, INPUT);
    digitalWrite(SONAR_TRIG, LOW);
        // 1. Crear mutex para protección de datos
    data_mutex = xSemaphoreCreateMutex();
    if (!data_mutex) {
        Serial.println("❌ Sonar: Error creando mutex");
        return;
    }
    Serial.printf("🔊 Sonar HC-SR04 inicializado: TRIG=%d, ECHO=%d\n", 
                  SONAR_TRIG, SONAR_ECHO);
    initialized = true;
}

uint16_t SonarIntegration::readDistance() {
    // 1. Asegurar que TRIG está LOW antes de empezar
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(4); // Más tiempo de estabilización
    
    // 2. Enviar pulso de 10μs (mínimo requerido por HC-SR04)
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(12); // 10-12μs recomendado
    digitalWrite(SONAR_TRIG, LOW);
    
    // 3. ⚠️ CRÍTICO: ESPERAR que ECHO se ponga en HIGH
    // El HC-SR04 tarda ~100-150μs en responder
    unsigned long startTime = micros();
    unsigned long timeout = 5000; // 5ms máximo para empezar
    
    while(digitalRead(SONAR_ECHO) == LOW) {
        if (micros() - startTime > timeout) {
            // Serial.println("[Sonar] ⚠️ Timeout esperando inicio de eco");
            errorCount++;
            return lastValidDistance;
        }
    }
    
    // 4. Medir duración del pulso HIGH
    unsigned long pulseStart = micros();
    unsigned long maxPulseTime = 30000; // 30ms = ~5 metros
    
    while(digitalRead(SONAR_ECHO) == HIGH) {
        if (micros() - pulseStart > maxPulseTime) {
            // Serial.println("[Sonar] ⚠️ Timeout en pulso HIGH (objeto muy lejos o error)");
            errorCount++;
            
            // Si hay demasiados timeouts, resetear el sensor
            if (errorCount > 20) {
                Serial.println("[Sonar] ❌ Reset por demasiados timeouts");
                errorCount = 0;
                // Pequeño reset
                digitalWrite(SONAR_TRIG, LOW);
                delay(10);
            }
            
            return lastValidDistance;
        }
    }
    
    unsigned long pulseEnd = micros();
    unsigned long duration = pulseEnd - pulseStart;
    
    // 5. Validar duración razonable
    // Mínimo: ~116μs (2cm) - Máximo: ~23200μs (4m)
    if (duration < 116 || duration > 25000) {
        Serial.printf("[Sonar] ⚠️ Duración fuera de rango: %luμs\n", duration);
        errorCount++;
        return lastValidDistance;
    }
    
    // 6. Convertir a mm con precisión mejorada
    // Velocidad del sonido a 20°C: 343 m/s = 0.343 mm/μs
    uint16_t distance = (duration * 0.343) / 2;
    
    // 7. DEBUG solo si hay cambios significativos
    static uint16_t lastReportedDistance = 0;
    if (abs(distance - lastReportedDistance) > 50) { // Cambio > 5cm
        Serial.printf("[Sonar] %luμs → %dmm\n", duration, distance);
        lastReportedDistance = distance;
    }
    
    errorCount = 0; // Reset contador de errores
    return distance;
}

void SonarIntegration::update() {
    if (!initialized) return;
    Serial.println("Actualizando Sonar");
    updateSonarData();
    
    // Debug cada 2 segundos
    static uint32_t last_debug = 0;
    uint32_t now = millis();
    if (now - last_debug > 2000) {
        last_debug = now;
        
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Serial.printf("📡 SensorManager: %dmm, Emergency: %s, OK: %s\n",
                         sonar_data.distance,
                         sonar_data.emergency ? "YES" : "NO",
                         sonar_data.sensor_ok ? "YES" : "NO");
            xSemaphoreGive(data_mutex);
        }
    }
}
void SonarIntegration::updateSonarData() {
  
    // Usar los métodos existentes de SonarIntegration
    uint16_t raw_distance = readDistance();
    uint16_t filtered_distance = applyFilter(raw_distance);
      
    // Determinar estados
    bool emergency = (filtered_distance > 0 && filtered_distance < 200);
    bool sensor_ok = (filtered_distance >= 20 && filtered_distance <= 4000);
    
    // Actualizar con protección de mutex
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sonar_data.distance = filtered_distance;
        sonar_data.raw_distance = raw_distance;
        sonar_data.emergency = emergency;
        sonar_data.sensor_ok = sensor_ok;
        sonar_data.timestamp = millis();
        
        if (!sensor_ok) {
            sonar_data.error_count++;
        } else {
            sonar_data.error_count = 0;
        }
        
        xSemaphoreGive(data_mutex);
    }
}

uint16_t SonarIntegration::applyFilter(uint16_t rawDistance){
// Si es timeout o error, usar el último valor válido
    if (rawDistance == lastValidDistance && errorCount > 0) {
        // Solo log si hay muchos errores seguidos
        static uint8_t consecutiveErrors = 0;
        consecutiveErrors++;
        
        if (consecutiveErrors > 5) {
            Serial.printf("[Sonar] ⚠️ %d errores consecutivos\n", consecutiveErrors);
            consecutiveErrors = 0;
        }
        return lastValidDistance;
    }
    
    // Reset contador de errores consecutivos
    static uint8_t consecutiveErrors = 0;
    consecutiveErrors = 0;
    
    // Filtrar valores fuera de rango
    if (rawDistance < 20 || rawDistance > 4000) {
        Serial.printf("[Sonar] ⚠️ Fuera de rango: %dmm\n", rawDistance);
        return lastValidDistance;
    }
    
    // Actualizar filtro de media móvil
    distanceBuffer[distanceIndex] = rawDistance;
    distanceIndex = (distanceIndex + 1) % FILTER_SIZE;
    
    // Calcular promedio de valores válidos
    uint32_t sum = 0;
    uint8_t count = 0;
    
    for (int i = 0; i < FILTER_SIZE; i++) {
        uint16_t val = distanceBuffer[i];
        if (val >= 20 && val <= 4000) {
            sum += val;
            count++;
        }
    }
    
    if (count > 0) {
        lastValidDistance = sum / count;
    }
    
    return lastValidDistance;
}
/*
uint16_t SonarIntegration::updateAndGetDistance() {
    uint16_t rawDistance = readDistance();
    
    // Si es timeout o error, usar el último valor válido
    if (rawDistance == lastValidDistance && errorCount > 0) {
        // Solo log si hay muchos errores seguidos
        static uint8_t consecutiveErrors = 0;
        consecutiveErrors++;
        
        if (consecutiveErrors > 5) {
            Serial.printf("[Sonar] ⚠️ %d errores consecutivos\n", consecutiveErrors);
            consecutiveErrors = 0;
        }
        return lastValidDistance;
    }
    
    // Reset contador de errores consecutivos
    static uint8_t consecutiveErrors = 0;
    consecutiveErrors = 0;
    
    // Filtrar valores fuera de rango
    if (rawDistance < 20 || rawDistance > 4000) {
        Serial.printf("[Sonar] ⚠️ Fuera de rango: %dmm\n", rawDistance);
        return lastValidDistance;
    }
    
    // Actualizar filtro de media móvil
    distanceBuffer[distanceIndex] = rawDistance;
    distanceIndex = (distanceIndex + 1) % FILTER_SIZE;
    
    // Calcular promedio de valores válidos
    uint32_t sum = 0;
    uint8_t count = 0;
    
    for (int i = 0; i < FILTER_SIZE; i++) {
        uint16_t val = distanceBuffer[i];
        if (val >= 20 && val <= 4000) {
            sum += val;
            count++;
        }
    }
    
    if (count > 0) {
        lastValidDistance = sum / count;
    }
    
    return lastValidDistance;
}
*/
bool SonarIntegration::getLatestSonarData(SonarSensorData_t &data) {
    if (!initialized || !data_mutex) return false;
    
    bool success = false;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&data, &sonar_data, sizeof(SonarSensorData_t));
        xSemaphoreGive(data_mutex);
        success = true;
    }
    
    return success;
}

uint16_t SonarIntegration::getLastDistance() {
    uint16_t distance = 1200;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        distance = sonar_data.distance;
        xSemaphoreGive(data_mutex);
    }
    
    return distance;
}

bool SonarIntegration::isEmergency() {
    bool emergency = false;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        emergency = sonar_data.emergency;
        xSemaphoreGive(data_mutex);
    }
    
    return emergency;
}
bool SonarIntegration::isSensorOK() {
    bool ok = false;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ok = sonar_data.sensor_ok;
        xSemaphoreGive(data_mutex);
    }
    
    return ok;
}

/*
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
/*
uint16_t SonarIntegration::readDistance() {
    digitalWrite(SONAR_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG, LOW);
    
    // Timeout max 30ms para evitar bloqueo
    long duration = pulseIn(SONAR_ECHO, HIGH, 30000); 
    // Conversión a mm (velocidad del sonido 340m/s = 0.34mm/us)
    uint16_t distance = (duration > 0) ? (duration * 0.34) / 2 : 1200;
    
    if (distance > 1200) distance = 1200;
    return distance;
}
*/
