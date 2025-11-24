// main.cpp - ESP32 con FreeRTOS
#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "BluetoothLeConnect.h"
#include "SensorControl.h"
#include "Command.h"

// Prioridades de tasks (mayor número = mayor prioridad)
#define TASK_PRIORITY_SAFETY    4  // Máxima prioridad - emergencias
#define TASK_PRIORITY_I2C       3  // Comunicación I2C  
#define TASK_PRIORITY_NAV       2  // Navegación
#define TASK_PRIORITY_BLE       1  // Comunicación BLE
// Definición pines I2C ESP32 con Arduino
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 20
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
TaskHandle_t xI2CTaskHandle = NULL;
TaskHandle_t xNavigationTaskHandle = NULL;
TaskHandle_t xBLETaskHandle = NULL;

QueueHandle_t xSensorQueue;
QueueHandle_t xCommandQueue;


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
            Wire.beginTransmission(0x08);
            Wire.write(command.cmd);
            Wire.endTransmission();
            
            Serial.printf("✅ Comando %d enviado a Arduino\n", command.cmd);
        }
        
        // 2. Leer datos del sonar del Arduino
        Wire.requestFrom(0x08, 3);
        if(Wire.available() >= 3) {
            sensorData.sonarDistance = (Wire.read() << 8) | Wire.read();
            sensorData.emergency = (Wire.read() == 1);
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

// 🚨 TASK DE SEGURIDAD (Máxima Prioridad)
void safetyTask(void *pvParameters) {
    Serial.println("🛡️  Safety Task iniciada");
    
    SensorData sensorData;
    
    for(;;) {
        // Revisar cola de sensores continuamente (sin remover los datos)
        if(xQueuePeek(xSensorQueue, &sensorData, 0) == pdTRUE) {
            if(sensorData.emergency) {
                Serial.println("🚨 EMERGENCY DETECTADA - Parada inmediata");
                
                // Enviar comando de emergencia a la cola de comandos
                Command emergencyCommand = {0x00, 0}; // CMD_EMERGENCY_STOP
                xQueueSend(xCommandQueue, &emergencyCommand, 0);
                
                // Notificar a otras tasks
                autonomousMode = false;
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

// 🏁 SETUP - Inicialización
void setup() {
    Serial.begin(115200);
    
    // Inicializar hardware (código Arduino tradicional)
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN); // SDA, SCL, 400kHz
    sensors.begin();
    ble.begin("CarRobot-FreeRTOS");
    
    // Crear recursos de FreeRTOS
    xSensorQueue = xQueueCreate(1, sizeof(SensorData)); // Solo necesita el último dato
    xCommandQueue = xQueueCreate(10, sizeof(Command));
    
    // ✅ ELIMINAR SEMÁFORO I2C - ya no es necesario con patrón servidor
    // xI2CSemaphore = xSemaphoreCreateMutex(); // ← COMENTADO/ELIMINADO
    
    // ✅ BALANCEO DE NÚCLEOS
    // BLE en Core 0 (donde corre el stack BLE)
    // Crear tasks de FreeRTOS

    // BLE Task en Core 0 (por el stack BLE)
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
    
    Serial.println("🚗 Sistema FreeRTOS iniciado - Tasks ejecutándose");
    Serial.println("   Core 0: BLE");
    Serial.println("   Core 1: Safety, I2C, Navigation");
    Serial.println("   ✅ Sin semáforos I2C - Patrón Servidor activo");
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
    
    delay(1000);
}