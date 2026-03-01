#include "SensorManager.h"
#include "SonarIntegration.h" 

SensorManager::SensorManager(SonarIntegration* sonar)
    : sonar_ref(sonar), initialized(false) {
    
    // Inicializar estructura de datos
    memset(&sonar_data, 0, sizeof(sonar_data));
    sonar_data.sensor_ok = false;
    sonar_data.distance = 1200;  // Valor por defecto
    sonar_data.raw_distance = 1200;
    sonar_data.timestamp = 0;
    sonar_data.error_count = 0;
}

 bool SensorManager::init() {
    if (initialized) return true;
    
    if (!sonar_ref) {
        Serial.println("❌ SensorManager: Referencia a Sonar es NULL");
        return false;
    }
    
    // 1. Crear mutex para protección de datos
    data_mutex = xSemaphoreCreateMutex();
    if (!data_mutex) {
        Serial.println("❌ SensorManager: Error creando mutex");
        return false;
    }
    
    // 2. El sonar ya está inicializado por main.cpp, solo tomamos lectura inicial
    updateSonarData();
    
    initialized = true;
    Serial.println("✅ SensorManager inicializado (usando Sonar existente)");
    return true;
}

void SensorManager::update() {
    if (!initialized || !sonar_ref) return;
    
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

void SensorManager::updateSonarData() {
    if (!sonar_ref) return;
    
    // Usar los métodos existentes de SonarIntegration
    uint16_t filtered_distance = sonar_ref->updateAndGetDistance();
    uint16_t raw_distance = sonar_ref->readDistance();
    
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

bool SensorManager::getLatestSonarData(SonarSensorData_t &data) {
    if (!initialized || !data_mutex) return false;
    
    bool success = false;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&data, &sonar_data, sizeof(SonarSensorData_t));
        xSemaphoreGive(data_mutex);
        success = true;
    }
    
    return success;
}

uint16_t SensorManager::getLastDistance() {
    uint16_t distance = 1200;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        distance = sonar_data.distance;
        xSemaphoreGive(data_mutex);
    }
    
    return distance;
}

bool SensorManager::isEmergency() {
    bool emergency = false;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        emergency = sonar_data.emergency;
        xSemaphoreGive(data_mutex);
    }
    
    return emergency;
}

bool SensorManager::isSensorOK() {
    bool ok = false;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        ok = sonar_data.sensor_ok;
        xSemaphoreGive(data_mutex);
    }
    
    return ok;
}