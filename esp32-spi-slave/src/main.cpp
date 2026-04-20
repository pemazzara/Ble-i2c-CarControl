// main.cpp del ESP32 Slave
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "MotorControl.h"
#include "sonar_integration.h"
#include "SpeedController.h"
#include "SPISlave.h"
#include "SPIDefinitions.h"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"

#define CONTROL_PERIOD_MS 20
#define CONTROL_PERIOD_S (CONTROL_PERIOD_MS / 1000.0f)
// =========================================================
// VARIABLES GLOBALES
// =========================================================

MotorControl motorController;
UltraSonicMeasure sonar;
SpeedController speedController;
SPISlave spiSlave(motorController, sonar); 

// =========================================================
// Handles de tareas
TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t spiTaskHandle = NULL;
TaskHandle_t sonarTaskHandle = NULL;

// =========================================================
// DECLARACIÓN DE TAREAS
// =========================================================
void motorTask(void *pvParameters);
void sonarTask(void *pvParameters);
void spiSlaveTask(void *pvParameters);
void heapMonitorTask(void* pvParameters);
void logSystemStatus();
void printResetReason();
unsigned long lastMotorCheck = 0;
SonarSensorData_t sonar_data;


void setup_tasks() {
    Serial.println("🔧 Configurando tareas...");
    // Tarea MOTOR en Core 1 (alta prioridad, mucho stack)
    xTaskCreatePinnedToCore(
        motorTask,
        "MotorControl",
        8192,
        NULL,
        4,  // Prioridad media
        &motorTaskHandle,   // Handle
        1   // Core 1
    );
  
    // Tarea SPI SLAVE en Core 1 (misma prioridad que motor)
    xTaskCreatePinnedToCore(
        spiSlaveTask,
        "SPI_Slave",
        6144,        // 6KB stack
        &spiSlave,
        4,           // Misma prioridad que motor
        NULL,
        1            // Core 1 exclusivo
    );
    
    // Tarea SENSORES en Core 0 (baja prioridad)
    xTaskCreatePinnedToCore(
        sonarTask,
        "Sonar",
        8192,        // 8KB stack
        &sonar,
        1,  // Prioridad baja
        NULL,
        0   // Core 0
    );  
    /* Tarea MONITOR en Core 0 (prioridad mínima)
        xTaskCreatePinnedToCore(
        heapMonitorTask,
        "HeapMonitor",
        2048,
        NULL,
        1,  // Prioridad baja
        NULL,
        0   // Core 1
    );*/
 
    Serial.println("✅ Sistema inicializado correctamente");
    Serial.println("   - Core 1: Motor + SPI (Alta prioridad)");
    Serial.println("   - Core 0: Sensores + Monitor (Baja prioridad)");
}

void check_heap() {
    size_t free_heap = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    size_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    
    Serial.printf("Heap libre: %d bytes, Min libre: %d bytes\n", 
                  free_heap, min_free);
    
    if (free_heap < 2048) {
        Serial.println("⚠️ ALERTA: Heap bajo!");
    }
}



// =========================================================
// TAREAS
// =========================================================
/*
void heapMonitorTask(void* pvParameters) {
    esp_task_wdt_add(NULL);
    while(1) {
        // 2. Resetear watchdog en cada iteración
        esp_task_wdt_reset();  
        check_heap();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}*/



void sonarTask(void* pvParameters) {
    UBaseType_t stack_start = uxTaskGetStackHighWaterMark(NULL);
    Serial.printf("[SonarTask] Stack inicial libre: %d palabras\n", stack_start);
    //esp_task_wdt_add(NULL);
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());

    // Variables locales para optimizar
    uint32_t last_measurement = 0;
    uint32_t last_stack_check = 0;
    uint32_t measurement_count = 0;
    
    // Verificar que SensorManager esté inicializado
    if (!sonar.initialized) {
        Serial.println("⚠️ SonarTask: SensorManager no inicializado correctamente");
    }
    
    while(1) {
        // 2. Resetear watchdog en cada iteración
        esp_task_wdt_reset();
        
        uint32_t now = millis();
        
        // 1. Verificar stack periódicamente
        if (now - last_stack_check > 5000) {
            last_stack_check = now;
            UBaseType_t stack_free = uxTaskGetStackHighWaterMark(NULL);
            Serial.printf("[SonarTask] Stack libre: %d bytes, Mediciones: %d\n",
                         stack_free * sizeof(StackType_t), measurement_count);
        }
        
        // 2. Actualizar sensor cada 50ms (20Hz - ideal para sonar)
        if (now - last_measurement >= 50) {
            last_measurement = now;
            measurement_count++;
            
            // Actualizar datos del sensor
            sonar.sonarUpdate();
           
            // Verificar emergencia
            if (sonar.isEmergency()) {
                motorController.emergencyStop();
                sonar.getLatestSonarData(sonar_data);
                Serial.printf("🚨 EMERGENCIA SONAR: %dmm\n", 
                             (int)sonar_data.distance);
            }
    
        }
        
        //vTaskDelay(pdMS_TO_TICKS(10));        
        // 3. Pausa optimizada, Frecuencia del sonar: ≈ 50 Hz (período 20 ms).
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}
void motorTask(void *pvParameters) {
    esp_task_wdt_add(NULL);  
    // Variables locales para optimizar stack
    uint32_t last_stack_check = 0;
    uint32_t last_spi_check = 0;
    uint32_t last_ramp_update = 0;
    uint32_t command_count = 0;
    uint32_t last_valid_command_ms = millis();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
    
    while(1) {
        esp_task_wdt_reset();
        uint32_t now = millis();
        static uint32_t stop_until = 0;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
        // 1. Verificar si hay nuevo comando del Master
        if (SPISlave::isCommandReady()) {
            ControlCommand_t cmd = SPISlave::getLastCommand();
            Serial.printf("✅ Cmd Listo: Type=%d, PWM: %d, Angle: %d\n", cmd.type, cmd.speed, cmd.angle);   
            speedController.setTargetFromMaster(cmd);
            last_valid_command_ms = now; // Actualizar cada vez que llega algo del Master
        }
        
        
        // --- WATCHDOG DE SEGURIDAD SPI ---
        if (now - last_valid_command_ms > 500) {
            motorController.setPWM(0, 0, true);
        }else {
            speedController.updateControl();
        }              
    }
}

/*
void motorTask(void* pvParameters) {
    esp_task_wdt_add(NULL);
    Serial.println("[MotorTask] Iniciada");
     
    // Variables locales para optimizar stack
    uint32_t last_stack_check = 0;
    uint32_t last_spi_check = 0;
    uint32_t last_ramp_update = 0;
    uint32_t command_count = 0;
    uint32_t last_valid_command_ms = millis();
    
    while(1) {
        // 2. Resetear watchdog en cada iteración
        esp_task_wdt_reset();
        uint32_t now = millis();
        static uint32_t stop_until = 0;

        // 2. Procesar comandos SPI si hay disponibles
        if (SPISlave::isCommandReady()) {
            ControlCommand_t cmd = SPISlave::getLastCommand();
            motorController.handleSPICommand(&cmd);
            // Señalar que hemos procesado el comando
            SPISlave::commandProcessed();
            last_valid_command_ms = now; // Actualizar cada vez que llega algo del Master
        }

        // --- WATCHDOG DE SEGURIDAD SPI ---
        if (now - last_valid_command_ms > 500) {
            // Si el Master no ha dicho nada en 0.5s, paramos por seguridad
            motorController.setDrive(0, 0, true);
        }

        // 3. Actualizar rampas de motor (50Hz)
        if (now - last_ramp_update >= 20) {
            last_ramp_update = now;
            motorController.updateRamping();
        }
        
        // 4. Pausa mínima
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}*/

void spiSlaveTask(void* pvParameters) {
    esp_task_wdt_add(NULL);
    // Crear una referencia a spiSlave
    SPISlave &spiSlave = *(SPISlave *)pvParameters;
        // Añadir esta tarea al watchdog
    esp_task_wdt_add(xTaskGetCurrentTaskHandle());
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(1); // 1ms para máxima reactividad  

    Serial.println("[SPI Slave Task] Iniciada");
    
    // Variables locales
    uint32_t last_stack_check = 0;
    uint32_t processed_count = 0;
    
    while(1) {
        // Resetear watchdog de esta tarea
        esp_task_wdt_reset();
        uint32_t now = millis();
        TickType_t lastWakeTime = xTaskGetTickCount();
        const TickType_t frequency = pdMS_TO_TICKS(1); // 1ms para máxima reactividad
        
        // Verificar stack
        if (now - last_stack_check > 15000) {
            last_stack_check = now;
            UBaseType_t stack_free = uxTaskGetStackHighWaterMark(NULL);
            Serial.printf("[SPI Slave] Stack libre: %d bytes, Procesados: %d\n",
                         stack_free * sizeof(StackType_t), processed_count);
        }
        
        // Procesar comunicación SPI
        spiSlave.processSPICommunication();
        processed_count++;

        vTaskDelay(pdMS_TO_TICKS(1)); // 👈 INDISPENSABLE
    }
}

void printResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    
    Serial.println("\n=== RAZÓN DE REINICIO ===");
    switch(reason) {
        case ESP_RST_POWERON:
            Serial.println("Power-on reset");
            break;
        case ESP_RST_EXT:
            Serial.println("External pin reset");
            break;
        case ESP_RST_SW:
            Serial.println("Software reset");
            break;
        case ESP_RST_PANIC:
            Serial.println("Software panic reset");
            break;
        case ESP_RST_INT_WDT:
            Serial.println("Interrupt watchdog reset");
            break;
        case ESP_RST_TASK_WDT:
            Serial.println("Task watchdog reset");
            break;
        case ESP_RST_WDT:
            Serial.println("Other watchdog reset");
            break;
        case ESP_RST_DEEPSLEEP:
            Serial.println("Deep sleep reset");
            break;
        case ESP_RST_BROWNOUT:
            Serial.println("Brownout reset");
            break;
        case ESP_RST_SDIO:
            Serial.println("SDIO reset");
            break;
        default:
            Serial.printf("Unknown reset reason: %d\n", reason);
    }
    Serial.println("=========================\n");
}


// 📊 MONITOREO DEL SISTEMA ACTUALIZADO
void logSystemStatus() {
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 5000) {
        // Usar las nuevas estructuras, no las viejas
        sonar.getLatestSonarData(sonar_data);
        Serial.printf("[Status] Sonar: %dmm | ", (int)sonar_data.distance);
        Serial.printf("Motor: L=%d, R=%d\n", 
                     motorController.getCurrentLeft(),
                     motorController.getCurrentRight());
        
        // Verificar stacks de tareas
        if (motorTaskHandle) {
            UBaseType_t motor_stack = uxTaskGetStackHighWaterMark(motorTaskHandle);
            Serial.printf("   MotorTask stack: %d bytes\n", motor_stack * 4);
        }
        
        lastLog = millis();
    }
}
// =========================================================
// SETUP
// =========================================================
void setup() {
    Serial.begin(115200);
    delay(2000); // Esperar para estabilidad
    printResetReason();
    Serial.println("\n=== ESP32 SLAVE - CON WATCHDOG ===\n");
    // 1. Configurar watchdog para todo el sistema
    uint32_t wdt_timeout_s = 5; // 5 segundos
    esp_err_t wdt_ret = esp_task_wdt_init(wdt_timeout_s, true); // true = activar 'panic' (reiniciar)

    if (wdt_ret != ESP_OK) {
        Serial.println("❌ Error inicializando WDT");
    }
    
    // 2. Añadir tarea principal al watchdog
    esp_task_wdt_add(NULL);

    // 1. Inicializar componentes
    sonar.begin();
    if (!spiSlave.init()) {
        Serial.println("❌ ERROR: SPI Slave no se inicializó");
        while(1) delay(1000);
    }
    
    motorController.begin();
    speedController.begin();
       
    // Inicializar SPI (usa referencias del constructor)

    //spiSlave.checkHealth();
    /* Configurar timers para chequeos periódicos 
    esp_timer_create_args_t timer_args = {
        .callback = [](void* arg) {
            ((SPISlave*)arg)->checkHealth();
        },
        .arg = &spiSlave,
        .name = "spi_health"
    };*/
    
    // 2. Crear primitivas FreeRTOS // Crear tareas
    setup_tasks();
    
    Serial.println("✅ Sistema FreeRTOS iniciado correctamente");
    Serial.println("   - SPI Slave en Core 1");
    Serial.println("   - Esperando comandos del Master...");
}

// =========================================================
// LOOP
// =========================================================
void loop() {
    esp_task_wdt_reset(); // RESET CRÍTICO
    //logSystemStatus();
    vTaskDelay(pdMS_TO_TICKS(1000)); 
}
// =========================================================
// FIN DEL ARCHIVO
