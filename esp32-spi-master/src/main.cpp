// main.cpp - ESP32 con FreeRTOS
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "BluetoothLeConnect.h"
#include "SensorControl.h"
#include "SPIMaster.h"
#include "config.h"
#include "GradientAligner.h"
#include "esp_task_wdt.h"
#include "system_state.h"


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
// Prioridades
#define TASK_PRIORITY_SAFETY    4
#define TASK_PRIORITY_SPI       3
#define TASK_PRIORITY_NAV       2  
#define TASK_PRIORITY_BLE       1

#define DISTANCIA_CRITICA_STOP 150 // 15cm o 150mm - Detener inmediatamente

// Estados del sistema de navegación
enum InitialState {
 STATE_I,   // Estado inicial de espera
 STATE_F,   // Escaneo Frontal (se espera comando 'L')
 STATE_L,   // Escaneo Izquierdo (se espera comando 'R')
 STATE_R,   // Escaneo Derecho (se espera comando 'F')
 STATE_READY // Listo para operar
};
InitialState currentState = STATE_I;


// Handles globales de FreeRTOS
TaskHandle_t xSafetyTaskHandle = NULL;
TaskHandle_t xSensorRead_Handle = NULL;
TaskHandle_t xSPITaskHandle = NULL;
TaskHandle_t xNavigationTaskHandle = NULL;
TaskHandle_t xBLETaskHandle = NULL;

QueueHandle_t xControlQueue;
QueueHandle_t xStatusQueue;
QueueHandle_t xBLECommandQueue;
//QueueHandle_t xSPICommandQueue; 

SemaphoreHandle_t sensorMutex;

void setupFreeRTOS();
void printResetReason();
// ✅ DECLARAR TODAS LAS TASKS
void safetyTask(void *pvParameters);
void sensorRead_Task(void *pvParameters);
void navigationTask(void *pvParameters);
void bleTask(void *pvParameters);
void spiMasterTask(void *pvParameters);
void masterFSMTask(void *pvParameters);

void handleSystemState();
void handleBLECommand(const BLECommand_t& cmd, ControlCommand_t& spiCmd, bool& pending);
void updateFSMFromSlaveData(const SensorData_t& data);
void sendStatusUpdate();
void processStateCommand(char command);
void checkJTAGPins();
void speedsToSpeedAngle(int16_t left, int16_t right, int& speed, int& angle);


// Instancias globales
static SPIMaster spiMaster; // Vive para siempre en el segmento de datos
BluetoothLeConnect ble;
SensorControl sensors;
SensorData_t globalSensorData;
//AngleOptimizer angleOptimizer;
//MasterStatusPacket_t statusPacket;  

bool autonomousMode = false;


// Funciones manejo FSM
// Traduce el comando recibido por BLE 
// a una transición en la FSM del Master.
void handleBLECommand(const BLECommand_t& cmd, ControlCommand_t& spiCmd, bool& pending) {
    switch (cmd.type) {
        case 0x02: // CMD_HEARTBEAT (Asegúrate de que coincida con el enum de Android)
                    // No hacemos nada con los motores, pero el simple hecho de entrar aquí
                    // ya debería haber actualizado el timestamp de "Última comunicación"
                    // en tu lógica de salud del sistema.
                    Serial.println("💓 Heartbeat recibido y validado");
                    break;
        case 0x05: // CMD_SET_MODE
                    SystemState::setBaseState(SYSTEM_STATE_READY);
                    if (cmd.targetMode == 0x02 || cmd.targetMode == 0x00) { // MANUAL o JOYSTICK
                        SystemState::setManualMode();
                    } 
                    else if (cmd.targetMode == 0x03) { // AUTOMATIC
                        SystemState::setAutonomousMode();
                    }
                    break;
                    
        case 0x07: // CMD_IDLE
                    if (SystemState::getBaseState() == SYSTEM_STATE_READY) {
                    SystemState::setIdleMode();
                    }
                    break;
        case 0x0B: // CMD_CALIBRATE
                    if (SystemState::getBaseState() == SYSTEM_STATE_BOOT ||
                        SystemState::getBaseState() == SYSTEM_STATE_READY) {
                        SystemState::setCalibrate();
                    }
                    break;

        case 0x00: // CMD_STOP (Emergencia)
                    SystemState::setEmergency();
                    break;
                
        case 0x09: // RESET_EMERGENCY
                    SystemState::setBaseState(SYSTEM_STATE_READY);
                    SystemState::setReadySubState(READY_SUBSTATE_IDLE);
                    break;

        case 0x01: // CMD_DRIVE
            // Solo si estamos en READY y modo MANUAL
            if (SystemState::getBaseState() == SYSTEM_STATE_READY &&
                SystemState::getReadySubState() == READY_SUBSTATE_MANUAL) {
                // Preparar comando SPI (tipo CMD_DRIVE)
                spiCmd.type = CMD_DRIVE;
                spiCmd.speed = cmd.speed;
                spiCmd.angle = cmd.angle;
                pending = true;   // Marcar pendiente
            }
            break;
    }
}
// Actualiza la FSM según el estado y los datos provenientes del Slave.
void updateFSMFromSlaveData(const SensorData_t& data) {
    SystemBaseState_t currentMasterState = SystemState::getBaseState();
    
    // 1. Verificar salud (El Watchdog se resetea en SPIMaster::sendCommand)
    SystemState::checkHealth();
    
    // Si el Watchdog mató el sistema, no procesamos lógica de sensores
    if (!SystemState::isCommActive() || currentMasterState == SYSTEM_STATE_ERROR) {
        return; 
    }

    // 2. PRIORIDAD MÁXIMA: Emergencia (Hardware o Software)
    bool slaveInEmergency = (data.lastSlaveFlags & 0x01) || data.emergency || (data.lastSlaveState == SLAVE_STATE_EMERGENCY);
    
    if (slaveInEmergency) {
        if (currentMasterState != SYSTEM_STATE_EMERGENCY) {
            SystemState::setEmergency();
        }
        return; 
    }

    // 3. Error Crítico del Slave (Ej: Fallo de driver de motor)
    if (data.lastSlaveFlags & 0x08) {
        SystemState::setError();
        return;
    }

    // 4. Sincronización de Calibración
    if (data.lastSlaveState == SLAVE_STATE_CALIBRATION) {
        if (currentMasterState != SYSTEM_STATE_CALIBRATE) {
            SystemState::setCalibrate();
        }
        return;
    }

    // 5. Gestión del estado READY
    if (data.calibrationValid) {
        // Si el Slave está listo pero el Master sigue en BOOT/CALIBRATE
        if (currentMasterState != SYSTEM_STATE_READY) {
            SystemState::setReady(); 
        }
        
        // Si el Slave está en IDLE, nosotros forzamos IDLE (Seguridad)
        if (data.lastSlaveState == SLAVE_STATE_IDLE) {
            if (SystemState::getReadySubState() != READY_SUBSTATE_IDLE) {
                SystemState::setIdleMode();
            }
        }
    } else {
        // 6. Sin calibración: No podemos estar en READY
        if (currentMasterState == SYSTEM_STATE_READY) {
            SystemState::setCalibrate();
        }
    }
}

// sendStatusUpdate 
// Generar y enviar paquete BLE con el estado actual del Master 
// y datos relevantes del Slave. Este paquete se enviará periódicamente 
// (ej. cada 500ms) o cuando haya cambios significativos en el estado.
void sendStatusUpdate() {
    // Obtener estado actual del Master
    SystemBaseState_t base = SystemState::getBaseState();
    ReadySubState_t sub = SystemState::getReadySubState();


    // Obtener datos del Slave (última copia local, ya bajo mutex en el bucle)
    // usamos la copia local que ya tenemos en la tarea, o la leemos de nuevo.
    // Asumimos que globalSensorData está accesible en el ámbito de la tarea.

    MasterStatusPacket_t packet;
    packet.msg_id = 0x81;
    packet.baseState = (uint8_t)base;
    packet.subState = (uint8_t)sub;

    // Construir alerts (bits)
    uint8_t alerts = 0;
    if (globalSensorData.emergency) alerts |= 0x01;
    if (!SystemState::isCommActive()) alerts |= 0x02;
    if (globalSensorData.batteryLow) alerts |= 0x04; // (añadir campo a SensorData si no)
    packet.alerts = alerts;

    // Progreso de calibración: si estamos en CALIBRATE, usar el progreso del Slave (si está en CALIBRATION)
    packet.progress = (base == SYSTEM_STATE_CALIBRATE) ? globalSensorData.calibrationProgress : 0;

    packet.distance = globalSensorData.sonarDistance;
    //Serial.printf("Global distance : %d ", globalSensorData.sonarDistance);
    // Parámetros de calibración (punto fijo)
    packet.K_fixed = (uint16_t)(globalSensorData.calibrationK * 1000);
    packet.tau_fixed = (uint16_t)(globalSensorData.calibrationTau * 100);

    // needs_ack: si estamos en estado que requiere confirmación (ej. EMERGENCY)
    packet.needs_ack = (base == SYSTEM_STATE_EMERGENCY || base == SYSTEM_STATE_ERROR) ? 1 : 0;

    // Enviar por BLE (asumiendo que tenemos acceso a la característica)
    if (xStatusQueue != NULL) {
        // Usamos un timeout de 0 (o muy bajo) para no bloquear la FSM
        if (xQueueSend(xStatusQueue, &packet, 0) != pdTRUE) {
            // Opcional: Serial.println("⚠️ Status Queue llena");
            Serial.println("⚠️ No se pudo enviar status update: Queue llena");
        }
    }
}

uint16_t fuseFrontalDistances(SensorData_t &data) {
    uint8_t confidence = 0;
    uint16_t final_dist = 0;

    // 1. Validar rangos individuales (filtros de salud)
    bool sonar_valid = (data.sonarDistance > 20 && data.sonarDistance < 4000);
    bool tof_valid = (data.tofFront > 20 && data.tofFront < 2000);

    // 2. Lógica de decisión
    if (sonar_valid && tof_valid) {
        int16_t diff = abs((int16_t)data.sonarDistance - (int16_t)data.tofFront);
        
        if (diff < 150) { // Si coinciden razonablemente (15cm)
            final_dist = (data.sonarDistance + data.tofFront) / 2; // Fusión promedio
            confidence |= 0x18; // Bits de fuente: 11 (Fused)
        } else {
            // Discrepancia: El objeto es raro (quizás un cristal o una rejilla)
            // Regla de oro: Ante la duda, confía en la distancia MÁS CORTA (Seguridad)
            final_dist = min(data.sonarDistance, data.tofFront);
            confidence |= 0x04; // Bit de discrepancia
            confidence |= (data.sonarDistance < data.tofFront) ? 0x08 : 0x10; 
        }
    } else if (tof_valid) {
        final_dist = data.tofFront;
        confidence |= 0x10; // Fuente: TOF
    } else if (sonar_valid) {
        final_dist = data.sonarDistance;
        confidence |= 0x08; // Fuente: Sonar
    }

    data.slaveInternalStatus = confidence; // Guardamos el veredicto
    return final_dist;
}
// 🏁 SETUP - Inicialización
void setup() {
    Serial.begin(115200);
    delay(1000); // Esperar a que Serial esté listo
    printResetReason();
    Serial.println("   Modo: SPI (Alta Velocidad)");
    //esp_task_wdt_init(30, false); // 30 segundos
    checkJTAGPins();
    SystemState::begin();
    SystemState::setReady();
    //SystemState::setBaseState(SYSTEM_STATE_READY);
    //SystemState::setReadySubState(READY_SUBSTATE_IDLE);
    sensors.begin();
    spiMaster.begin();
    ble.begin("CarRobot-FreeRTOS");
    // Configurar FreeRTOS
    setupFreeRTOS();
    
    // Añade esto en setup() para verificar el tamaño
    Serial.printf("Tamaño de ControlCommand_t: %d bytes\n", sizeof(ControlCommand_t));
    Serial.printf("Tamaño esperado: %d bytes\n", 
              1 +    // type
              2 +    // speed
              2 +    // angle
              2 +    // timestamp
              1     // state
); // Total: 8 bytes
    Serial.println("🚗 Sistema FreeRTOS iniciado - Tasks ejecutándose");
    Serial.println("   Core 0: BLE");
}

// ✅ VERIFICACIÓN PINES JTAG
void checkJTAGPins() {
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
}
/*
void stopRobot() {
    ControlCommand_t stopCmd;
    memset(&stopCmd, 0, sizeof(ControlCommand_t));

    stopCmd.type = CMD_STOP;
    stopCmd.speed = 0;
    stopCmd.angle = 0;
    stopCmd.timestamp = millis();

    // Intentamos enviarlo al frente de la cola para que sea inmediato
    if (xControlQueue != NULL) {
        if (xQueueOverwrite(xControlQueue, &stopCmd) != pdTRUE) {
            Serial.println("⚠️ Fallo crítico: Cola de control llena al intentar parar");
        } else {
            Serial.println("🛑 Comando STOP enviado a la cola");
        }
    }
}
// En el Master, en setup() o cuando inicies:
void resetSlaveEmergency() {
    Serial.println("🔄 Reseteando emergencia en Slave...");   
    // 1. Enviar STOP para resetear emergencia
    ControlCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = CMD_STOP;
    cmd.timestamp = millis();
    if (xQueueOverwrite(xControlQueue, &cmd) != pdTRUE) {
            Serial.println("⚠️ Cola de control llena, comando descartado");
        }  
    delay(100);
    
    
    Serial.println("✅ Slave reseteado (emergencia limpiada)");
}*/

// ✅ SETUP FREERTOS SIMPLIFICADO
void setupFreeRTOS() {
    Serial.println("🔧 Inicializando FreeRTOS...");
    
    // Crear colas
    xControlQueue = xQueueCreate(1, sizeof(ControlCommand_t));
    xBLECommandQueue = xQueueCreate(10, sizeof(BLECommand_t));
    //xSPICommandQueue = xQueueCreate(10, sizeof(ControlCommand_t));
    xStatusQueue = xQueueCreate(10, sizeof(MasterStatusPacket_t));
    sensorMutex = xSemaphoreCreateMutex();
    ControlCommand_t vacio = {0};
    xQueueOverwrite(xControlQueue, &vacio);
    
    if (xControlQueue == NULL || sensorMutex == NULL ) {
        Serial.println("❌ ERROR: No se pudo crear xControlQueue");
        while(1);
    }

    // Crear tasks
    // Tasks en Core 1 (Ordenadas por prioridad real)
    // Serial.println("   Creando task SPI...");
    xTaskCreatePinnedToCore(
        spiMasterTask,
        "SPI_Master", 
        8192,  // ✅ Aumentar stack para SPI
        &spiMaster,   // ✅ Sin parámetros complejos
        3,  // Prioridad alta para SPI
        &xSPITaskHandle,
        1
    );
    // Crear tarea para la máquina de estados del Master
    xTaskCreatePinnedToCore(
        masterFSMTask,          // Función de la tarea
        "MasterFSM",            // Nombre
        4096,                   // Stack size (bytes)
        NULL,                   // Parámetros
        4,                      // Prioridad (mayor que tareas de comunicación)
        NULL,                   // Handle de la tarea
        1                       // Core (1 = segundo core en ESP32)
    );
    /* Tasks comunes
    xTaskCreatePinnedToCore(
        safetyTask,
        "Safety",
        4096,
        NULL,
        5, //TASK_PRIORITY_SAFETY,
        &xSafetyTaskHandle,
        1
    );
    */
    Serial.println("   Creando task Navigation...");
    xTaskCreatePinnedToCore(
        navigationTask,
        "Navigation",
        4096, 
        NULL,
        2, //TASK_PRIORITY_NAV,
        &xNavigationTaskHandle,
        1
    );
    // Task en Core 0 (Aislada para radio)
    Serial.println("   Creando task BLE...");
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        8192,
        NULL,
        5, //TASK_PRIORITY_BLE,
        &xBLETaskHandle,
        0
    );
    Serial.println("   Creando task SensorRead...");
    xTaskCreatePinnedToCore(
        sensorRead_Task,     // Función
        "SensorRead",        // Nombre
        8192,               // ← AUMENTA ESTE VALOR (ej: 8192 o 16384)
        NULL,               // Parámetros
        3, //configMAX_PRIORITIES - 2,
        &xSensorRead_Handle,
        1
    );
    Serial.println("✅ FreeRTOS inicializado");
}

void speedsToSpeedAngle(int16_t left, int16_t right, int& speed, int& angle) {
    // 1. Si ambos son cero, es STOP
    if (left == 0 && right == 0) {
        speed = 0;
        angle = 90;  // Ángulo por defecto, no importa porque speed=0
        return;
    }

    // 2. Calcular velocidad media (escala 0-1023)
    speed = (abs(left) + abs(right)) / 2;
    if (speed > 1023) speed = 1023;

    // 3. Determinar direcciones individuales
    bool leftForward = left >= 0;
    bool rightForward = right >= 0;

    // 4. Calcular factor de giro normalizado entre -1 y 1
    //    -1 = giro máximo a la izquierda, 1 = giro máximo a la derecha
    float turnFactor;
    float maxSpeed = max(abs(left), abs(right));
    if (maxSpeed == 0) {
        turnFactor = 0;
    } else {
        // La diferencia de velocidades normalizada por la máxima
        float diff = (right - left) / 2.0f;  // media de la diferencia
        turnFactor = diff / maxSpeed;
        // turnFactor queda entre -1 y 1 aproximadamente
    }

    // 5. Mapear turnFactor a ángulo según el caso
    if (leftForward && rightForward) {
        // Ambos adelante: ángulo entre 0° (derecha) y 180° (izquierda)
        // turnFactor = -1 -> giro izquierda (180°)
        // turnFactor = 0  -> recto (90°)
        // turnFactor = 1  -> giro derecha (0°)
        angle = 90 - (int)(turnFactor * 90);
    }
    else if (!leftForward && !rightForward) {
        // Ambos atrás: ángulo entre 180° y 360°
        // turnFactor = -1 -> giro izquierda atrás (180°? cuidado)
        // Realmente queremos que atrás recto sea 270°
        // Para atrás, el factor de giro invierte el sentido
        angle = 270 - (int)(turnFactor * 90);
    }
    else if (leftForward && !rightForward) {
        // Giro izquierda sobre el eje (left adelante, right atrás)
        // Esto debería dar un ángulo cercano a 180°
        // La magnitud del giro depende de la relación de velocidades
        float ratio = (float)abs(right) / (float)abs(left);
        angle = 180 - (int)(90 * ratio);
    }
    else if (!leftForward && rightForward) {
        // Giro derecha sobre el eje (left atrás, right adelante)
        float ratio = (float)abs(left) / (float)abs(right);
        angle = (int)(90 * ratio);
    }

    // 6. Normalizar ángulo a [0, 360)
    angle = (angle + 360) % 360;
}

void handleSystemState() {
    sensors.readAll(); // Actualizar todas las lecturas de sensores
switch(currentState) {
    case STATE_I:
        Serial.println("🔵 ESTADO I: Esperando 'F' para iniciar calibración");
    break;
    case STATE_F:
    // Ejecuta la acción de escaneo en este estado (Frontal)
        globalSensorData.tofFront = sensors.frontDistance;      
        Serial.printf("🟡 ESTADO F: Frontal = %dmm - Esperando 'L'\n", globalSensorData.tofFront);
    break; // Espera el comando 'L' para pasar al siguiente estado

    case STATE_L:
    // Ejecuta la acción de escaneo en este estado (Izquierdo)
        globalSensorData.tofLeft = sensors.leftDistance;
        Serial.printf("🟠 ESTADO L: Izquierdo = %dmm - Esperando 'R'\n", globalSensorData.tofLeft);
    break; // Espera el comando 'R' para pasar al siguiente estado

    case STATE_R:
    // Ejecuta la acción de escaneo en este estado (Derecho)
        globalSensorData.tofRight = sensors.rightDistance;
        Serial.printf("🔴 ESTADO R: Derecho = %dmm - Esperando 'F' para finalizar\n", globalSensorData.tofRight);
    break; // Espera el comando 'F' para pasar a READY

    case STATE_READY:
        globalSensorData.tofFront = sensors.frontDistance;     
        globalSensorData.tofLeft = sensors.leftDistance;
        globalSensorData.tofRight = sensors.rightDistance;
        Serial.printf("🟡 ESTADO READY: Frontal = %dmm, Izquierdo = %dmm, Derecho = %dmm\n", 
                      globalSensorData.tofFront, globalSensorData.tofLeft, globalSensorData.tofRight);
        // ... Lógica normal de READY ...
        if (millis() % 5000 < 100) {
            Serial.println("🟢 SISTEMA LISTO - Modo operacional");
    }
    break;
    }
}

void processStateCommand(ControlCommand_t command) {
 switch(currentState) {
    case STATE_I:
        if (command.type == CMD_DRIVE && command.angle == 90) { // Comando 'F' inicia la calibración (I -> F)
        currentState = STATE_F;
        Serial.println("🎯 Transición: I → F (Iniciar Escaneo Frontal)");
    }
    break;
    case STATE_F:
        if (command.type == CMD_DRIVE && command.angle == 180) { // Comando 'L' pasa al escaneo izquierdo (F -> L)
        currentState = STATE_L;
        Serial.println("🎯 Transición: F → L (Escaneo Izquierdo)");
    }
    break;
    case STATE_L:
        if (command.type == CMD_DRIVE && command.angle == 0) { // Comando 'R' pasa al escaneo derecho (L -> R)
        currentState = STATE_R;
        Serial.println("🎯 Transición: L → R (Escaneo Derecho)");
        }
    break;
    case STATE_R:
        if (command.type == CMD_STOP) { // Comando Stop finaliza y pasa a READY (R -> READY)
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
void masterFSMTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20 Hz

    // Variables locales para la tarea
    ControlCommand_t pendingSPICmd;      // Comando SPI pendiente
    bool pendingSPI = false;         // Flag de comando pendiente

    // Configurar callbacks de la FSM (se mantienen)
    // Capturamos por referencia [&] para poder modificar las variables locales
    SystemState::setOnEnterEmergency([&pendingSPICmd, &pendingSPI]() {
        Serial.println("[Master] ¡EMERGENCIA ACTIVA!");
        ControlCommand_t stopCmd;
        memset(&stopCmd, 0, sizeof(stopCmd));
        stopCmd.type = CMD_STOP;
        stopCmd.speed = 0;
        stopCmd.angle = 0;
        // OPCIÓN B (Pro): Saltarse la fila y enviar YA a la cola de hardware
        // Como la cola es de tamaño 1, esto machaca cualquier movimiento previo.
        xQueueOverwrite(xControlQueue, &stopCmd);
    });

    SystemState::setOnExitEmergency([&pendingSPI]() {
        pendingSPI = false;
        Serial.println("[Master] EXIT EMERGENCY");
    });

    SystemState::setOnEnterCalibrate([&pendingSPICmd, &pendingSPI]() {
        Serial.println("[Master] ENTER CALIBRATE");
        ControlCommand_t calibCmd;
        memset(&calibCmd, 0, sizeof(calibCmd));
        calibCmd.type = CMD_CALIBRATE;
        calibCmd.speed = 0;
        calibCmd.angle = 0;
        pendingSPICmd = calibCmd;
        pendingSPI = true;
    });
    SystemState::setOnExitCalibrate([&pendingSPI]() {
        Serial.println("[Master] EXIT CALIBRATE");
        pendingSPI = false;
    });
    SystemState::setOnEnterReady([&pendingSPICmd, &pendingSPI]() {
        Serial.println("[Master] ENTER READY");
        ControlCommand_t activeCmd;
        memset(&activeCmd, 0, sizeof(activeCmd));
        activeCmd.type = CMD_READY;
        activeCmd.speed = 0;
        activeCmd.angle = 0;
        pendingSPICmd = activeCmd;
        pendingSPI = true;
    });
    SystemState::setOnExitReady([&pendingSPICmd, &pendingSPI]() {
        Serial.println("[Master] EXIT READY");
        ControlCommand_t idleCmd;
        memset(&idleCmd, 0, sizeof(idleCmd));    
        idleCmd.type = CMD_IDLE;
        idleCmd.speed = 0;
        idleCmd.angle = 0;
        pendingSPICmd = idleCmd;
        pendingSPI = true;
    });

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 2. Procesar comandos BLE pendientes
        BLECommand_t bleCmd;
        while (xQueueReceive(xBLECommandQueue, &bleCmd, 0) == pdTRUE) {
            // La función handleBLECommand ahora recibe referencia a pendingSPICmd y pendingSPI
            handleBLECommand(bleCmd, pendingSPICmd, pendingSPI);
            //xQueueOverwrite(xControlQueue, &bleCmd);
        }

        // 3. Actualizar la FSM según los datos del Slave
        //updateFSMFromSlaveData(globalSensorData);

        // 4. Si hay un comando SPI pendiente, encolarlo
        if (pendingSPI) {
            // xQueueOverwrite siempre tiene éxito si la cola es de tamaño 1
            xQueueOverwrite(xControlQueue, &pendingSPICmd);
            pendingSPI = false; // Resetear flag
        }

        // 5. Generar y enviar estado por BLE
        sendStatusUpdate();
    }
}
void spiMasterTask(void *pvParameters) {
    // 1. Validar parámetros de entrada
    if (pvParameters == nullptr) {
        Serial.println("❌ Error: Parámetros de SPI Task nulos");
        vTaskDelete(NULL);
        return;
    }
    SPIMaster &spiMaster = *(SPIMaster *)pvParameters;
    // 2. Esperar a que las colas estén listas
    while (xControlQueue == NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    TickType_t xLastWakeTime = xTaskGetTickCount();
    // 1. Inicializamos con STOP para que el robot nazca frenado
    ControlCommand_t lastCmd;
    memset(&lastCmd, 0, sizeof(ControlCommand_t)); // Limpieza inicial
    lastCmd.type = CMD_STOP; 
    uint32_t lastUpdate = millis(); // Empezamos contando desde ahora
    

    for(;;) {
       
        ControlCommand_t newCmd;
        
        // A. Intentamos actualizar el comando desde la cola
        if (xQueueReceive(xControlQueue, &newCmd, 0) == pdTRUE) {
            lastCmd = newCmd;
            lastUpdate = millis();
        }
                  

        // B. SEGURIDAD: Si han pasado > 500ms sin datos de la App, sobreescribimos a STOP
        if (millis() - lastUpdate > 500) {
            lastCmd.type = CMD_STOP;
            lastCmd.speed = 0;
            lastCmd.angle = 0;
            lastCmd.timestamp = millis();        
        }
        // C. Solo enviamos el comando si es STOP o DRIVE, para evitar enviar comandos de estado o calibración innecesarios
        if (lastCmd.type == CMD_DRIVE) {

            Serial.printf("Enviando comando: Type=%d, Speed=%d, Angle=%d, Timestamp=%u\n", 
                          lastCmd.type, lastCmd.speed, lastCmd.angle, lastCmd.timestamp);
                  
            // C. Ejecución de la comunicación SPI
            spiMaster.sendCommand(&lastCmd); 
        }   
               
            // Prioridad 2: Si el robot ya está en stop, pedimos telemetría
            //spiMaster.requestSensorData();
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

// Task ESPECÍFICA para sensores (alta frecuencia)
void sensorRead_Task(void *pvParameters) {
    while(1) {

        sensors.readAll(); // Lee TOFs locales
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            globalSensorData.tofFront = sensors.frontDistance;
            globalSensorData.tofLeft = sensors.leftDistance;
            globalSensorData.tofRight = sensors.rightDistance;            // ... actualizar resto de TOFs ...
            globalSensorData.lastTofUpdate = millis();
            xSemaphoreGive(sensorMutex);
        }
         
        //spiMaster.evaluarEmergenciaInmediata(globalSensorData);
        vTaskDelay(pdMS_TO_TICKS(20));
    }  // Tarea crítica: Lectura del sensor
}

void navigationTask(void *pvParameters) {
    GradientAligner aligner;
    ControlCommand_t command;
    uint32_t navigationCycle = 0;

    while(1) {
        navigationCycle++;
        
        // 1. Fusión de sensores (Pasamos la estructura global protegida)
        uint16_t front_dist = 0;
        uint16_t left_dist = 0;
        uint16_t right_dist = 0;
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            front_dist = fuseFrontalDistances(globalSensorData);
            left_dist = globalSensorData.tofLeft;
            right_dist = globalSensorData.tofRight;
            xSemaphoreGive(sensorMutex);
        }
        if (left_dist > 0 && right_dist > 0) {
            aligner.setSideSensors(left_dist, right_dist);
        } else {
            aligner.disableSideSensors();
        }
        // 2. Lógica de Decisión
        if (autonomousMode && front_dist < 500 && front_dist > 0) { 
            // Hay obstáculo: Calculamos ángulo óptimo x3*
            int16_t optimal_angle = aligner.computeAngle(front_dist);
            
            command.speed = 200; // Velocidad de exploración
            command.angle = optimal_angle;
            command.type = CMD_DRIVE;
            
            // --- TELEMETRÍA DEL VECTOR X ---
            // Formato compatible con Serial Plotter: x1:dist, x2:avel, x3:angle, x4:conf
            Serial.printf(">x1:%u,x3:%d,status:%u\n", 
                          front_dist, optimal_angle, globalSensorData.sensorStatus);
        } 
        else if (autonomousMode) {
            // Camino despejado: Ir recto a velocidad normal
            command.speed = 400; 
            command.angle = 90;
            command.type = CMD_DRIVE;
        }

        // 3. Envío de Comando
        command.timestamp = millis();
        xQueueOverwrite(xControlQueue, &command);
        //xQueueSendToFront(xControlQueue, &command, 0); // Enviar al frente para máxima prioridad

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

// 🚨 TASK DE SEGURIDAD (Máxima Prioridad)
void safetyTask(void *pvParameters) {
    Serial.println("🛡️ Safety Task - Iniciada");
    bool lastEmergencyState = false;
    uint32_t failCount = 0;
    const uint32_t MAX_FAILS = 3; // Necesitamos 3 fallos seguidos para saltar

    vTaskDelay(pdMS_TO_TICKS(2000)); // Espera de inicialización

    for(;;) {
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            uint32_t ahora = millis();
            // 1. Validar frescura (1.5s de margen para ser permisivos con el SPI)
            bool sonarAlive = (ahora - globalSensorData.lastSonarUpdate < 1500);
            bool tofAlive = (ahora - globalSensorData.lastTofUpdate < 1500);
            bool sensoresVivos = sonarAlive && tofAlive;
            // 2. Lógica de Disparo
            if(globalSensorData.emergency || !sensoresVivos) {
                failCount++;
            } else {
                failCount = 0; // Reset si todo está bien
            }
            if(failCount >= MAX_FAILS && !lastEmergencyState) {
                if(!sensoresVivos) {
                    Serial.printf("🚨 EMERGENCY - Timeout! (Sonar:%d, TOF:%d)\n", sonarAlive, tofAlive);
                } else {
                    Serial.println("🚨 EMERGENCY - Colisión Inminente!");
                }

                ControlCommand_t emergencyCommand = {0};
                emergencyCommand.type = CMD_STOP;
                emergencyCommand.speed = 0;
                emergencyCommand.angle = 90; // Ángulo neutro

                xQueueOverwrite(xControlQueue, &emergencyCommand); // Overwrite para saltar la cola
                //if(xQueueSendToFront(xControlQueue, &emergencyCommand, 0) != pdTRUE) {
                //    Serial.println("⚠️ Cola llena - Fallo crítico de emergencia");
                //}
                autonomousMode = false;
                lastEmergencyState = true;
            }
            
            // 4. Reset
            else if (!globalSensorData.emergency && sensoresVivos && lastEmergencyState) {
                Serial.println("✅ Condiciones normales restauradas.");
                lastEmergencyState = false;
                failCount = 0;
            }
            xSemaphoreGive(sensorMutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz es suficiente para seguridad mecánica
    }
}

void bleTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms = 20Hz
    MasterStatusPacket_t statusPacket;
    for(;;) {
        ble.update();
        // Obtener último comando BLE como estructura
        if (ble.hasNewCommand) { // Solo si realmente llegó algo nuevo por BLE
            BLECommand_t bleCmd = ble.getLastCommand();
            // Enviar a la cola de la FSM
            if (xQueueSend(xBLECommandQueue, &bleCmd, 0) != pdTRUE) {
                Serial.println("⚠️ BLE: cola de comandos llena");
            }
        }
        /*
            ControlCommand_t cmd;
            cmd.type = bleCmd.type;
            cmd.speed = bleCmd.speed;
            cmd.angle = bleCmd.angle;        
            cmd.timestamp = bleCmd.timestamp;
            // Asegurar timestamp actual si no viene
            if (cmd.timestamp == 0) {
                cmd.timestamp = millis();
            }
            // 1. Los paros de emergencia siempre deben pasar
            if (cmd.type == CMD_STOP) {
                xQueueOverwrite(xControlQueue, &cmd); // Enviar inmediatamente el STOP
                autonomousMode = false;
                Serial.println("🛑 Comando STOP recibido por BLE - Acción inmediata");
            }

            // Debug
            Serial.printf("🎮 BLE Cmd: targetMode=0x%02X,type=0x%02X, speed=%d, angle=%d\n",
                         bleCmd.targetMode, cmd.type, cmd.speed, cmd.angle);    
            // Lógica de Transición/Calibración (STATE_I, F, L, R)
            //if (currentState != STATE_READY) {
                // Si NO estamos en READY, intentamos usar el comando para la transición de estados.cd
            //    processStateCommand(cmd);
            //} 
            
            if (cmd.type == CMD_DRIVE ) {
                xQueueOverwrite(xControlQueue, &cmd); // Enviar al frente para máxima prioridad
                autonomousMode = false; // Desactivar modo autónomo ante un stop
            } 
            // 2. Si no es emergencia, verificamos el modo
            else if (bleCmd.targetMode == MANUAL) {
                autonomousMode = false;  // Desactivar modo autónomo
            }else if (bleCmd.targetMode == AUTOMATIC) {
                autonomousMode = true; // Activar modo autónomo
            } 
            // 3. Si es autónomo y NO es un heartbeat, avisamos que ignoramos el comando
            else if (bleCmd.type != CMD_HEARTBEAT && bleCmd.targetMode == AUTOMATIC) {
                autonomousMode = true; // Activamos modo autónomo si el comando lo indica
                Serial.println("⚠️ Modo Autónomo activo - Comando manual ignorado");
            }          
        }*/
        // Enviar estado actual por BLE (puedes ajustar la frecuencia)
                // 2. Enviar estado a la app (si hay paquete listo)
        if (xQueueReceive(xStatusQueue, &statusPacket, 0) == pdTRUE) {
            ble.sendBLEPacket(&statusPacket);
        }
        // Esperar usando delay fijo para timing consistente
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
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


// 🔁 LOOP principal (en Core 1) - Puede usarse para tareas de baja prioridad
void loop() {
    // El loop se ejecuta en Core 1 por defecto
    // Puedes usarlo para tareas no críticas o monitoreo
    // Llama al gestor de estados para permitir las transiciones automáticas
    //handleSystemState();
    //static unsigned long lastStatus = 0;
    //if(millis() - lastStatus > 10000) {
    //    Serial.println("💚 Sistema FreeRTOS funcionando...");
        // Monitoreo de uso de colas
        //UBaseType_t sensorQueueMessages = uxQueueMessagesWaiting(xSensorQueue);
        //UBaseType_t commandQueueMessages = uxQueueMessagesWaiting(xControlQueue);
        
        //Serial.printf("   📊 Cola Sensores: %d, Cola Comandos: %d\n", 
        //             sensorQueueMessages, commandQueueMessages);
        
    //    lastStatus = millis();
    //}

    /* Ejemplo de uso de la función de alto nivel

    uint16_t distancia;
    uint8_t estadoSlave;


    if (spiMaster.getCleanSensorData(&globalSensorData)) {
        // --- Lógica de navegación ---
        if (globalSensorData.distance < 30) {
            Serial.printf("⚠️ Obstáculo a %d cm. Girando...\n", globalSensorData.distance);
            // cmd.type = CMD_TURN_LEFT; spiMaster.sendCommand(&cmd);
        } else {
            Serial.println("✅ Camino despejado.");
        }
    } else {
        // El Master internamente ya está contando errores 
        // y llamará a reconnect() si es necesario.
        Serial.println("❌ Error de enlace SPI.");
    }
*/
    // El robot actualiza sus sentidos 20 veces por segundo
    //updateRobotLogic();
    vTaskDelay(pdMS_TO_TICKS(100)); // Esperar 100ms sin bloquear el Core 1
    
    
}