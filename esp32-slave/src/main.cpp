#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "MotorControl.h"
#include "SonarIntegration.h"
// =========================================================
// DEFINICIONES DE HARDWARE Y COMANDOS (Consolidado de Arduino)
// =========================================================
#define USE_SPI_SLAVE

#ifdef USE_SPI_SLAVE
    #include "SPISlave.h"
    #define SPI_CLK  39   
    #define SPI_MISO 40     
    #define SPI_MOSI 41   
    #define SPI_SS   42
#else    
    // Pines I2C (Se usarán los pines I2C por defecto o se definen aquí)
    // En el modo Slave, solo se necesitan las líneas SDA y SCL para Wire.begin(address).
    // Usaremos los pines por defecto del ESP32 si no se especifican otros.
    #define SLAVE_ADDR 0x04 // The I2C address of this slave device // Dirección I2C fija (0x08)
    #define I2C_SDA_PIN 41
    #define I2C_SCL_PIN 42
    TwoWire I2CSlave = TwoWire(0);
#endif
// =========================================================
// ESTRUCTURAS DE DATOS Y PRITIMIVAS FREERTOS
// =========================================================

// Estructura para almacenar los datos que el Máster va a solicitar
struct SharedSensorData {
    uint16_t sonarDistance;
    bool emergencyFlag;
};

// Estructura para los comandos recibidos por I2C que van a la Tarea Motor
struct MotorCommand {
    uint8_t cmd;
    uint8_t data; // Solo para comandos de configuración (velocidad, etc.)
};

// Mutex para proteger la estructura de datos que se comparte entre la Tarea Sensor 
// (escritura) y el handler I2C (lectura).
SemaphoreHandle_t xSensorDataMutex;
// Cola para enviar comandos I2C desde el handler I2C (ISR) a la Tarea Motor
QueueHandle_t xMotorCommandQueue;
// Instancia global para los datos de sensores
SharedSensorData g_sensorData = {0, false};
MotorControl motorController;
SonarIntegration sonar;

// =========================================================
// HANDLERS I2C (Rutinas de interrupción)
// =========================================================
#ifndef USE_SPI_SLAVE
// El Master solicita datos
void handleI2CRequest() {
    SharedSensorData temp;
    
    // Intentamos tomar el Mutex para leer la data compartida de forma segura
    if (xSemaphoreTakeFromISR(xSensorDataMutex, NULL) == pdTRUE) {
        temp = g_sensorData;
        xSemaphoreGiveFromISR(xSensorDataMutex, NULL);
    } else {
        // En caso de fallo (no debería ocurrir), enviar 0
        temp = {0, false};
    }
    // Preparar respuesta
    uint8_t response[4];
    response[0] = highByte(temp.sonarDistance);
    response[1] = lowByte(temp.sonarDistance);
    response[2] = temp.emergencyFlag ? 1 : 0;
    response[3] = motorController.getMotorStatus(); // Nuevo: estado motores
    
    I2CSlave.write(response, 4);
    
    /* El Máster espera 3 bytes: Distancia Alta, Distancia Baja, Bandera
    I2CSlave.write(highByte(temp.sonarDistance));
    I2CSlave.write(lowByte(temp.sonarDistance));
    I2CSlave.write(temp.emergencyFlag ? 1 : 0);
    */
    // No se hace Serial.print en ISR
}

// El Master envía comandos
void handleI2CReceive(int byteCount) {
    if (byteCount == 0) return;

    MotorCommand command;
    command.cmd = I2CSlave.read();
    
    // Los comandos de configuración tienen un byte de dato adicional
    if (command.cmd >= CMD_SET_MANUAL_SPEED && command.cmd <= CMD_SET_SONAR_STATE) {
        command.data = I2CSlave.available() ? I2CSlave.read() : 0;
    } else {
        command.data = 0;
    }
    
    // 🎯 CRUCIAL: Enviar comando a la cola, NO ejecutar lógica compleja aquí
    // Usamos 'FromISR' para interactuar con FreeRTOS desde una interrupción.
    xQueueSendFromISR(xMotorCommandQueue, &command, NULL);
    
    // Descartar bytes sobrantes
    while(I2CSlave.available()) I2CSlave.read();
}
#endif
// =========================================================
// TAREAS FREERTOS
// =========================================================
// En Slave - monitoreo de rendimiento
void logSystemStatus() {
    static unsigned long lastLog = 0;
    if(millis() - lastLog > 5000) {
        UBaseType_t stackMotor = uxTaskGetStackHighWaterMark(NULL);
        UBaseType_t queueItems = uxQueueMessagesWaiting(xMotorCommandQueue);
        
        Serial.printf("📊 SLAVE STATUS: Stack=%d, Queue=%d, Sonar=%dmm\n",
                     stackMotor, queueItems, g_sensorData.sonarDistance);
        lastLog = millis();
    }
}

// 🏃 TAREA DE CONTROL DE MOTOR (Alta Prioridad - Procesar comandos)
void motorTask(void *pvParameters) {
    MotorCommand cmd;
    
    motorController.begin();
    
    for(;;) {
        // 1. Esperar comando de la cola (Máx. 100ms)
        if(xQueueReceive(xMotorCommandQueue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            motorController.handleCommand(cmd.cmd, cmd.data);
        }
        
        // 2. Ejecutar chequeo de seguridad (Timeout si no hay comandos)
        motorController.updateSafety();
        
        // Dar un pequeño respiro a otras tareas
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 🔊 TAREA DE SENSORES (Prioridad Media - Recolectar datos)
void sensorTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // Leer cada 50ms

    sonar.begin();
    
    for(;;) {
        uint16_t distance = sonar.updateAndGetDistance();
        bool isEmergency = distance < 100; // Umbral de emergencia

        // 1. Ejecutar parada de emergencia local (si aplica)
        if (isEmergency) {
            motorController.emergencyStop();
        }

        // 2. Actualizar datos compartidos, protegidos por Mutex
        if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_sensorData.sonarDistance = distance;
            g_sensorData.emergencyFlag = isEmergency; // Bandera para el Máster
            xSemaphoreGive(xSensorDataMutex);
        } else {
            Serial.println("❌ ERROR: No se pudo tomar Mutex en SensorTask.");
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 🏁 SETUP

void setup() {
    Serial.begin(115200);
#ifdef USE_SPI_SLAVE
    // Task en Core 1 para máxima velocidad sin interrumpir WiFi/BLE (si usaras)
    xTaskCreatePinnedToCore(
        spiSlaveTask,
        "SPI_Slave",
        8192, // Un poco más de stack por si acaso
        NULL,
        10,   // Prioridad muy alta (FreeRTOS max suele ser 24 o 25)
        NULL,
        1     // Core 1
    );
#else
    // Inicializar I2C como SLAVE (¡CRUCIAL!)
    I2CSlave.begin(SLAVE_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, 400000); // 400kHz
    I2CSlave.onRequest(handleI2CRequest);
    I2CSlave.onReceive(handleI2CReceive);
#endif
    // Crear primitivas de FreeRTOS
    xSensorDataMutex = xSemaphoreCreateMutex();
    // Cola de comandos con capacidad para 5 comandos en espera
    xMotorCommandQueue = xQueueCreate(5, sizeof(MotorCommand)); 
    
    // Crear Tareas (Todas en Core 1 para simplificar)
    xTaskCreatePinnedToCore(
        motorTask,
        "MotorControl",
        4096, // Tamaño de la pila
        NULL,
        2, // Prioridad 2
        NULL,
        1  // Core 1
    );
    
    xTaskCreatePinnedToCore(
        sensorTask,
        "SensorRead",
        3072, // Tamaño de la pila
        NULL,
        1, // Prioridad 1
        NULL,
        1 // Core 1
    );
    
    Serial.println("✅ ESP32 SLAVE (Actuador) FreeRTOS Iniciado.");
#ifndef USE_SPI_SLAVE
    Serial.printf("   - I2C Slave Address: 0x%X\n", SLAVE_ADDR);
#endif
    Serial.println("   - Esperando comandos y solicitudes de datos del ESP32 Master.");
}

// 🔁 LOOP (Se puede dejar vacío o usar para tareas no críticas)
void loop() {
    // Todo el trabajo pesado se maneja en las Tareas de FreeRTOS.
    vTaskDelay(pdMS_TO_TICKS(1000));
}