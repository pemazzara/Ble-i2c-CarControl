// Esp32Slave - SonarIntegration.cpp
#include "SonarIntegration.h"
#include <Arduino.h>
#include <driver/gpio.h>



SonarIntegration::SonarIntegration(uint8_t trigPin, uint8_t echoPin) 
    : SONAR_TRIG(trigPin), SONAR_ECHO(echoPin) {
    // Inicializar estructura de datos
    memset(&sonar_data, 0, sizeof(sonar_data));
    sonar_data.sensor_ok = false;
    sonar_data.distance = 1200;  // Valor por defecto
    sonar_data.duration = 0;
    sonar_data.emergency = false;
    sonar_data.timestamp = 0;
    sonar_data.error_count = 0;
    for(int i = 0; i < FILTER_SIZE; i++) distanceBuffer[i] = 1200;
}


void SonarIntegration::begin() {
    pinMode(SONAR_TRIG, OUTPUT);
    digitalWrite(SONAR_TRIG, LOW);
    pinMode(SONAR_ECHO, INPUT_PULLDOWN);
    
    data_mutex = xSemaphoreCreateMutex();
    if (!data_mutex) {
        Serial.println("❌ Sonar: Error creando mutex");
        return;
    }

    // 1. Si el canal ya estaba instalado, desinstalar primero (por si acaso)
    rmt_driver_uninstall(RMT_RX_CHANNEL);

    // 2. Configurar RMT para recepción
    rmt_config_t rmt_rx = {};
    rmt_rx.channel = RMT_RX_CHANNEL;
    rmt_rx.gpio_num = (gpio_num_t)SONAR_ECHO;
    rmt_rx.clk_div = 80;                     // → 1 tick = 1µs
    rmt_rx.mem_block_num = 1;
    rmt_rx.rmt_mode = RMT_MODE_RX;
    rmt_rx.rx_config.filter_en = true;
    rmt_rx.rx_config.filter_ticks_thresh = 100;   // Ignorar ruido < 100µs
    rmt_rx.rx_config.idle_threshold = 25000;      // Timeout > 25ms

    esp_err_t err = rmt_config(&rmt_rx);
    if (err != ESP_OK) {
        Serial.printf("❌ Error rmt_config: %d\n", err);
        return;
    }

    err = rmt_driver_install(rmt_rx.channel, 1000, 0);
    if (err != ESP_OK) {
        Serial.printf("❌ Error rmt_driver_install: %d\n", err);
        return;
    }

    Serial.printf("🔊 Sonar HC-SR04 con RMT canal %d en pin %d\n", RMT_RX_CHANNEL, SONAR_ECHO);
    initialized = true;
}


uint16_t SonarIntegration::triggerAndReadRMT() {
    uint16_t measured_distance = 0; // local
    uint32_t dur = 0;
    // 1. Enviar pulso de trigger de 10µs
    digitalWrite(SONAR_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(SONAR_TRIG, LOW);

    // 2. Preparar el RMT para recibir
    rmt_rx_start(RMT_RX_CHANNEL, true); // true = resetear el reloj interno

    // 3. Esperar y obtener los items del ringbuffer
    RingbufHandle_t rb = NULL;
    rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
    
    size_t rx_size = 0;
    rmt_item32_t* items = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size, pdMS_TO_TICKS(100)); // Timeout de 100ms


    if (items && rx_size >= 1) {
        // El primer item debería contener la duración del pulso HIGH (el echo)
        // Asumiendo que el pulso empieza con un flanco de subida (level0=1)
        if (items[0].level0 == 1) {
            dur = items[0].duration0; // Duración en ticks de 1µs
            if (dur > 115 && dur < 23500) {
                // Fórmula: (duración (µs) * 0.343 (mm/µs)) / 2
                // Podemos usar enteros para más velocidad: (duration_us * 343) / 2000
                measured_distance = (dur * 343) / 2000;
                duration = dur;
            }
        }
        vRingbufferReturnItem(rb, (void*) items);
    }

    rmt_rx_stop(RMT_RX_CHANNEL);
    // Actualizar estructura con mutex
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sonar_data.duration = dur;
        sonar_data.distance = measured_distance;
        sonar_data.sensor_ok = (measured_distance >= 20 && measured_distance <= 4000);
        if (measured_distance > 0 && measured_distance < 200) {
            if (emergency_counter < EMERGENCY_THRESHOLD) emergency_counter++;
        } else {
                    emergency_counter = 0;
                }
        sonar_data.emergency = (emergency_counter >= EMERGENCY_THRESHOLD);
        sonar_data.timestamp = millis();
        xSemaphoreGive(data_mutex);
    }
    return measured_distance;
}

void SonarIntegration::update() {
       // 3. NO ESPERES AQUÍ. Deja que el RMT trabaje.
    // En la siguiente vuelta del loop o tras un pequeño delay no bloqueante:
    triggerAndReadRMT();
    Serial.printf("[Sonar] %luμs → %dmm\n", sonar_data.duration, sonar_data.distance); 
}

uint16_t SonarIntegration::getLastDistance() {
    // 🛡️ GUARDIA CRÍTICA: Si el mutex no existe, abortar
    if (data_mutex == NULL) return 1200; // Valor por defecto
    // Usamos mutex porque el SPI (Master) y el Sensor (Task) 
    // podrían intentar acceder al mismo tiempo
    uint16_t dist = 1200;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        dist = sonar_data.distance;
        xSemaphoreGive(data_mutex);
    } else {
        dist = 1200; // Fallback rápido si el mutex está ocupado
    }
    return dist;
}

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


bool SonarIntegration::isEmergency() {
    bool emergency = true;
    if (millis() - sonar_data.timestamp > 500) return true;
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
