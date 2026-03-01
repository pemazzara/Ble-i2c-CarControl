// main.cpp - ESP32 con FreeRTOS
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "BluetoothLeConnect.h"
#include "SensorControl.h"
#include "Command.h"
#include "esp_task_wdt.h"

// Prioridades de tasks (mayor número = mayor prioridad)
#define TASK_PRIORITY_SAFETY    4  // Máxima prioridad - emergencias
#define TASK_PRIORITY_I2C       3  // Comunicación I2C  
#define TASK_PRIORITY_NAV       2  // Navegación
#define TASK_PRIORITY_BLE       1  // Comunicación BLE

#define USE_SPI_MASTER

#ifdef USE_SPI_MASTER
    #include "SPIMaster.h"
    /*
ESP32-S3 MASTER          ESP32-S3 SLAVE
GPIO 41 (MOSI) ------- GPIO 41 (MOSI)
GPIO 40 (MISO) ------- GPIO 40 (MISO)  
GPIO 39 (SCLK) ------- GPIO 39 (SCLK)
GPIO 42 (CS)   ------- GPIO 42 (CS)
      GND      -------      GND
      */
    #define SPI_CLK  39   
    #define SPI_MISO 40     
    #define SPI_MOSI 41   
    #define SPI_SS   42
#else
    #include <Wire.h>
    // Definición pines I2C ESP32 Master
    #define I2C_SDA_PIN 41
    #define I2C_SCL_PIN 42
    #define SLAVE_ADDR 0x04
    TwoWire I2CMaster = TwoWire(0);
#endif
// Prioridades
#define TASK_PRIORITY_SAFETY    4
#define TASK_PRIORITY_COMM      3
#define TASK_PRIORITY_NAV       2  
#define TASK_PRIORITY_BLE       1
// Estados del sistema de navegación
enum SystemState {
 STATE_I,   // Estado inicial de espera
 STATE_F,   // Escaneo Frontal (se espera comando 'L')
 STATE_L,   // Escaneo Izquierdo (se espera comando 'R')
 STATE_R,   // Escaneo Derecho (se espera comando 'F')
 STATE_READY // Listo para operar
};
SystemState currentState = STATE_I;


// Handles globales de FreeRTOS
TaskHandle_t xSafetyTaskHandle = NULL;
#ifndef USE_I2C_MASTER
TaskHandle_t xSPITaskHandle = NULL;
#else
TaskHandle_t xI2CTaskHandle = NULL;
#endif
TaskHandle_t xNavigationTaskHandle = NULL;
TaskHandle_t xBLETaskHandle = NULL;

QueueHandle_t xSensorQueue;
QueueHandle_t xCommandQueue;

// ✅ DECLARAR TODAS LAS TASKS
void safetyTask(void *pvParameters);
void navigationTask(void *pvParameters);
void bleTask(void *pvParameters);
#ifdef USE_SPI_MASTER
    void spiMasterTask(void *pvParameters);
#else
    void i2cTask(void *pvParameters);
#endif

void handleSystemState();
void processStateCommand(char command);
void checkJTAGPins();

// Estructuras de datos para comunicación entre tasks
struct SensorData {
    uint16_t sonarDistance;
    uint16_t tofFront;
    uint16_t tofLeft; 
    uint16_t tofRight;
    bool emergency;
};

struct Command {
    uint8_t cmd;
    uint8_t data;
};

// Instancias globales
BluetoothLeConnect ble;
SensorControl sensors;
SensorData dataSensors;
bool autonomousMode = false;
void handleSystemState();
void processStateCommand(char command);

// ✅ SETUP FREERTOS SIMPLIFICADO
void setupFreeRTOS() {
    Serial.println("🔧 Inicializando FreeRTOS...");
    
    // Crear colas
    xSensorQueue = xQueueCreate(1, sizeof(SensorData));
    xCommandQueue = xQueueCreate(10, sizeof(Command));
    
    if (xSensorQueue == NULL || xCommandQueue == NULL) {
        Serial.println("❌ ERROR: No se pudieron crear las colas");
        return;
    }

    // Crear tasks
#ifndef USE_SPI_MASTER
    Serial.println("   Creando task I2C...");
    xTaskCreatePinnedToCore(
        i2cTask,
        "I2C_Master", 
        4096,
        NULL,
        TASK_PRIORITY_COMM,
        &xI2CTaskHandle,
        1
    );
#else
    Serial.println("   Creando task SPI...");
    xTaskCreatePinnedToCore(
        spiMasterTask,
        "SPI_Master", 
        8192,  // ✅ Aumentar stack para SPI
        NULL,   // ✅ Sin parámetros complejos
        TASK_PRIORITY_COMM,
        &xSPITaskHandle,
        1
    );
#endif

    // Tasks comunes
    xTaskCreatePinnedToCore(
        safetyTask,
        "Safety",
        4096,
        NULL,
        TASK_PRIORITY_SAFETY,
        &xSafetyTaskHandle,
        1
    );
    
    xTaskCreatePinnedToCore(
        navigationTask,
        "Navigation",
        4096, 
        NULL,
        TASK_PRIORITY_NAV,
        &xNavigationTaskHandle,
        1
    );
    
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        4096,
        NULL,
        TASK_PRIORITY_BLE,
        &xBLETaskHandle,
        0
    );
    
    Serial.println("✅ FreeRTOS inicializado");
}


void handleSystemState() {

switch(currentState) {
    case STATE_I:
        Serial.println("🔵 ESTADO I: Esperando 'F' para iniciar calibración");
    break;
    case STATE_F:
    // Ejecuta la acción de escaneo en este estado (Frontal)
        dataSensors.tofFront = sensors.readSensor(0);      
        Serial.printf("🟡 ESTADO F: Frontal = %dmm - Esperando 'L'\n", dataSensors.tofFront);
    break; // Espera el comando 'L' para pasar al siguiente estado

    case STATE_L:
    // Ejecuta la acción de escaneo en este estado (Izquierdo)
        dataSensors.tofLeft = sensors.readSensor(1);
        Serial.printf("🟠 ESTADO L: Izquierdo = %dmm - Esperando 'R'\n", dataSensors.tofLeft);
    break; // Espera el comando 'R' para pasar al siguiente estado

    case STATE_R:
    // Ejecuta la acción de escaneo en este estado (Derecho)
        dataSensors.tofRight = sensors.readSensor(2);
        Serial.printf("🔴 ESTADO R: Derecho = %dmm - Esperando 'F' para finalizar\n", dataSensors.tofRight);
    break; // Espera el comando 'F' para pasar a READY

    case STATE_READY:
        dataSensors.tofFront = sensors.readSensor(0);     
        Serial.printf("🟡 ESTADO READY: Frontal = %dmm", dataSensors.tofFront);
        // ... Lógica normal de READY ...
        if (millis() % 5000 < 100) {
            Serial.println("🟢 SISTEMA LISTO - Modo operacional");
    }
    break;
    }
}

void processStateCommand(char command) {
 switch(currentState) {
    case STATE_I:
        if (command == 'F') { // Comando 'F' inicia la calibración (I -> F)
        currentState = STATE_F;
        Serial.println("🎯 Transición: I → F (Iniciar Escaneo Frontal)");
    }
    break;
    case STATE_F:
        if (command == 'L') { // Comando 'L' pasa al escaneo izquierdo (F -> L)
        currentState = STATE_L;
        Serial.println("🎯 Transición: F → L (Escaneo Izquierdo)");
    }
    break;
    case STATE_L:
        if (command == 'R') { // Comando 'R' pasa al escaneo derecho (L -> R)
        currentState = STATE_R;
        Serial.println("🎯 Transición: L → R (Escaneo Derecho)");
        }
    break;
    case STATE_R:
        if (command == 'F') { // Comando 'F' finaliza y pasa a READY (R -> READY)
        currentState = STATE_READY;
        Serial.println("🎯 Transición: R → READY");
        Serial.println("🚗 ¡SISTEMA LISTO PARA OPERAR!");
        // performSensorDiagnostics(); // Si quieres ejecutar esto al final
    }
    break;
    case STATE_READY:
        // En READY, esta función podría procesar comandos de navegación si no están en modo autónomo.
    break;
    }
}


// 📡 TASK DE I2C (Servidor I2C - Único que accede al bus I2C)
// ✅ TASK I2C (solo si no usamos SPI)
#ifndef USE_SPI_MASTER
void i2cTask(void *pvParameters) {
    Serial.println("📡 I2C Task (Servidor) iniciada");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(80); // 80ms
    
    SensorData sensorData = {0};
    Command command;
    
    for(;;) {
        // 1. Procesar comandos de la cola (no bloqueante)
        while(xQueueReceive(xCommandQueue, &command, 0) == pdTRUE) {
            // Enviar comando al Arduino
            I2CMaster.beginTransmission(SLAVE_ADDR);
            I2CMaster.write(command.cmd);
            I2CMaster.endTransmission();
            
            Serial.printf("✅ Comando %d enviado a Arduino\n", command.cmd);
        }
        
        // 2. Leer datos del sonar del Arduino
        I2CMaster.requestFrom(SLAVE_ADDR, 3);
        if(I2CMaster.available() >= 3) {
            sensorData.sonarDistance = (I2CMaster.read() << 8) | I2CMaster.read();
            sensorData.emergency = (I2CMaster.read() == 1);
        } else {
            sensorData.sonarDistance = 0;
            sensorData.emergency = false;
        }
        
        // 3. Leer datos del VL53L0X // se pasó a navigationTask
        //tofScanner.navigationScan(sensorData.tofLeft, sensorData.tofFront, sensorData.tofRight);
        
        // 4. Enviar datos a la cola (para otras tasks)
        // Si la cola está llena, sobreescribir el dato más antiguo
        xQueueOverwrite(xSensorQueue, &sensorData);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
#endif

// 🚨 TASK DE SEGURIDAD (Máxima Prioridad)
void safetyTask(void *pvParameters) {
    Serial.println("🛡️  Safety Task iniciada");
    
    SensorData sensorData;
    bool lastEmergencyState = false;

    for(;;) {
        // Revisar cola de sensores continuamente (sin remover los datos)
        if(xQueuePeek(xSensorQueue, &sensorData, 0) == pdTRUE) {
            // ✅ Solo actuar si el estado de emergencia CAMBIA
            if(sensorData.emergency && !lastEmergencyState) {
                Serial.println("🚨 EMERGENCY DETECTADA - Parada inmediata");
                
                // Enviar comando de emergencia a la cola de comandos
                Command emergencyCommand = {0x00, 0}; // CMD_EMERGENCY_STOP
                xQueueSend(xCommandQueue, &emergencyCommand, 0);
                
                // Notificar a otras tasks
                autonomousMode = false;
                lastEmergencyState = true;
            }
            // ✅ Resetear cuando la emergencia termina
            else if (!sensorData.emergency && lastEmergencyState) {
                Serial.println("✅ EMERGENCY RESUELTA");
                lastEmergencyState = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50)); // Revisar cada 50ms
    }
}

// 🧭 TASK DE NAVEGACIÓN
void navigationTask(void *pvParameters) {
    Serial.println("🧭 Navigation Task iniciada");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 200ms
    
    SensorData sensorData;
    Command command;
    
    for(;;) {
        if(currentState == STATE_READY && autonomousMode) { 
            // 1. Leer solo si el sistema está listo
            sensors.readAll();
            // 2. Leer datos del sonar de la cola
            if(xQueueReceive(xSensorQueue, &sensorData, 0) == pdTRUE) {
                // Actualizar solo los campos TOF que i2cTask ya no actualiza
                sensorData.tofLeft = sensors.leftDistance;
                sensorData.tofFront = sensors.frontDistance;
                sensorData.tofRight = sensors.rightDistance;
                // Re-enviar los datos COMBINADOS a la cola para SafetyTask, etc.
                xQueueOverwrite(xSensorQueue, &sensorData);
                // Lógica de decisión
                uint16_t effectiveFront = min(sensorData.tofFront, sensorData.sonarDistance);
                
                if(effectiveFront < 100) {
                    command.cmd = 0x00; // EMERGENCY_STOP
                } else if(effectiveFront > 300) {
                    command.cmd = 0x01; // FORWARD
                } else if(sensorData.tofLeft > sensorData.tofRight) {
                    command.cmd = 0x03; // TURN_LEFT
                } else {
                    command.cmd = 0x04; // TURN_RIGHT
                }
                
                // Enviar comando a la cola de comandos
                xQueueSend(xCommandQueue, &command, 0);
                
                Serial.printf("🤖 NAV: S:%d TOF[L:%d F:%d R:%d] -> CMD:%d\n",
                            sensorData.sonarDistance, sensorData.tofLeft,
                            sensorData.tofFront, sensorData.tofRight, command.cmd);
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 📱 TASK DE BLE
void bleTask(void *pvParameters) {
    Serial.println("📱 BLE Task iniciada");

    for(;;) {
        // 1. Procesar la actualización del stack BLE
        ble.update();
        
        String bleCommand = ble.getLastCommand();

        if (bleCommand.length() > 0) {
            char cmd = bleCommand.charAt(0);
            Command command; // Declarada aquí para usarla si el estado es READY

            // 2. Lógica de Transición/Calibración (STATE_I, F, L, R)
            if (currentState != STATE_READY) {
                // Si NO estamos en READY, intentamos usar el comando para la transición de estados.
                // handleSystemState (en loop) ejecuta la acción (TOF, Servo), processStateCommand hace el cambio de estado.
                processStateCommand(cmd);
            } 
            
            // 3. Lógica de Operación (STATE_READY)
            // Solo procesamos comandos de navegación/modo autónomo si estamos en STATE_READY
            else { // Esto significa: (currentState == STATE_READY)
                
                if (cmd == 'A') {
                    autonomousMode = true;
                    Serial.println("🤖 MODO AUTÓNOMO ACTIVADO");
                } else if (cmd == 'M') {
                    autonomousMode = false;
                    command.cmd = 0x05; // STOP
                    xQueueSend(xCommandQueue, &command, 0);
                    Serial.println("👋 MODO MANUAL ACTIVADO");
                } 
                // Asegúrate de que las constantes CMD_MANUAL_X estén definidas
                else if (cmd == 'F') {
                    command.cmd = CMD_MANUAL_FORWARD;
                    xQueueSend(xCommandQueue, &command, 0);
                } else if (cmd == 'B') {
                    command.cmd = CMD_MANUAL_BACKWARD;
                    xQueueSend(xCommandQueue, &command, 0);
                } else if (cmd == 'L') {
                    command.cmd = CMD_MANUAL_LEFT;
                    xQueueSend(xCommandQueue, &command, 0);
                } else if (cmd == 'R') {
                    command.cmd = CMD_MANUAL_RIGHT;
                    xQueueSend(xCommandQueue, &command, 0);
                } else if (cmd == 'S') {
                    command.cmd = CMD_STOP;
                    xQueueSend(xCommandQueue, &command, 0);
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Ejecutar cada 100ms
    }
}
void printResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("🔄 Reset Reason: ");
    switch(reason) {
        case ESP_RST_POWERON: Serial.println("Power On"); break;
        case ESP_RST_EXT: Serial.println("External Reset"); break;
        case ESP_RST_SW: Serial.println("Software Reset"); break;
        case ESP_RST_PANIC: Serial.println("Exception/Panic"); break;
        case ESP_RST_INT_WDT: Serial.println("Interrupt Watchdog"); break;
        case ESP_RST_TASK_WDT: Serial.println("Task Watchdog"); break;
        case ESP_RST_WDT: Serial.println("Other Watchdog"); break;
        case ESP_RST_DEEPSLEEP: Serial.println("Deep Sleep"); break;
        case ESP_RST_BROWNOUT: Serial.println("Brownout"); break;
        case ESP_RST_SDIO: Serial.println("SDIO Reset"); break;
        default: Serial.println("Unknown"); break;
    }
}
// 🏁 SETUP - Inicialización
void setup() {
    Serial.begin(115200);
    delay(1000); // Esperar a que Serial esté listo
     printResetReason();
#ifdef USE_SPI_MASTER
    Serial.println("   Modo: SPI (Alta Velocidad)");
    //esp_task_wdt_init(30, false); // 30 segundos
    checkJTAGPins();
#else
    Serial.println("   Modo: I2C (Compatible)");
    // Inicializar hardware (código Arduino tradicional)
    I2CMaster.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000); // SDA, SCL, 400kHz
    I2CMaster.beginTransmission(SLAVE_ADDR);
    I2CMaster.printf("Iniciando comunicación con esclavo I2C en dirección 0x%02X\n", SLAVE_ADDR);
    uint8_t error = I2CMaster.endTransmission(true);
    Serial.printf("Resultado de conexión I2C: %d (0=OK)\n", error);
#endif
    sensors.begin();
    ble.begin("CarRobot-FreeRTOS");
    // Configurar FreeRTOS
    setupFreeRTOS();
    // Crear recursos de FreeRTOS
    //xSensorQueue = xQueueCreate(1, sizeof(SensorData)); // Solo necesita el último dato
    //xCommandQueue = xQueueCreate(10, sizeof(Command));
    
    // ✅ ELIMINAR SEMÁFORO I2C - ya no es necesario con patrón servidor
    // xI2CSemaphore = xSemaphoreCreateMutex(); // ← COMENTADO/ELIMINADO
    
    // ✅ BALANCEO DE NÚCLEOS
    // BLE en Core 0 (donde corre el stack BLE)
    // Crear tasks de FreeRTOS

    /* BLE Task en Core 0 (por el stack BLE)
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        4096,
        NULL,
        TASK_PRIORITY_BLE, 
        &xBLETaskHandle,
        0 // Core 0
    );
    
    // Las demás tasks en Core 1
    xTaskCreatePinnedToCore(
        safetyTask,
        "Safety",
        4096,
        NULL,
        TASK_PRIORITY_SAFETY,
        &xSafetyTaskHandle,
        1 // Core 1
    );
  
    xTaskCreatePinnedToCore(
        i2cTask,
        "I2C_Master", 
        4096,
        NULL,
        TASK_PRIORITY_I2C,
        &xI2CTaskHandle,
        1 // Core 1
    );

    xTaskCreatePinnedToCore(
        navigationTask,
        "Navigation",
        4096, 
        NULL,
        TASK_PRIORITY_NAV,
        &xNavigationTaskHandle,
        1 // Core 1
    );
    */
    Serial.println("🚗 Sistema FreeRTOS iniciado - Tasks ejecutándose");
    Serial.println("   Core 0: BLE");
    Serial.println("   Core 1: Safety, I2C, Navigation");
#ifndef USE_SPI_MASTER
    Serial.println("   ✅ Sin semáforos I2C - Patrón Servidor activo");
#endif
}

// ✅ VERIFICACIÓN PINES JTAG
void checkJTAGPins() {
#ifdef USE_SPI_MASTER
    Serial.println("🔍 Verificando pines JTAG...");
    
    gpio_config_t pin_cfg = {};
    pin_cfg.pin_bit_mask = (1ULL << 39) | (1ULL << 40) | (1ULL << 41) | (1ULL << 42);
    pin_cfg.mode = GPIO_MODE_INPUT;
    pin_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    pin_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pin_cfg.intr_type = GPIO_INTR_DISABLE;
    
    esp_err_t ret = gpio_config(&pin_cfg);
    
    if (ret == ESP_OK) {
        Serial.println("✅ Pines JTAG disponibles para SPI");
    } else {
        Serial.printf("❌ Error pines JTAG: 0x%x\n", ret);
    }
#endif
}

// 🔁 LOOP principal (en Core 1) - Puede usarse para tareas de baja prioridad
void loop() {
    // El loop se ejecuta en Core 1 por defecto
    // Puedes usarlo para tareas no críticas o monitoreo
// Llama al gestor de estados para permitir las transiciones automáticas
    handleSystemState();
    static unsigned long lastStatus = 0;
    if(millis() - lastStatus > 10000) {
        Serial.println("💚 Sistema FreeRTOS funcionando...");
        // Monitoreo de uso de colas
        UBaseType_t sensorQueueMessages = uxQueueMessagesWaiting(xSensorQueue);
        UBaseType_t commandQueueMessages = uxQueueMessagesWaiting(xCommandQueue);
        
        Serial.printf("   📊 Cola Sensores: %d, Cola Comandos: %d\n", 
                     sensorQueueMessages, commandQueueMessages);
        
        lastStatus = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Esperar 100ms sin bloquear el Core 1
}