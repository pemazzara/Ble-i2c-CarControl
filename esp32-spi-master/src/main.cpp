// main.cpp - ESP32 con FreeRTOS
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "BluetoothLeConnect.h"
#include "SensorControl.h"
#include "SPIMaster.h"
#include "config.h"


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
TaskHandle_t xSensorRead_Handle = NULL;
TaskHandle_t xSPITaskHandle = NULL;
TaskHandle_t xNavigationTaskHandle = NULL;
TaskHandle_t xBLETaskHandle = NULL;

QueueHandle_t xSensorQueue;
QueueHandle_t xControlQueue;
// Ejemplo para proteger acceso SPI:
static SemaphoreHandle_t spi_mutex = NULL;

//extern void evaluarEmergenciaInmediata(const SensorData_t &sd);
void setupFreeRTOS();
void printResetReason();
// ✅ DECLARAR TODAS LAS TASKS
void safetyTask(void *pvParameters);
void sensorRead_Task(void *pvParameters);
void navigationTask(void *pvParameters);
void bleTask(void *pvParameters);
void spiMasterTask(void *pvParameters);

void handleSystemState();
void processStateCommand(char command);
void checkJTAGPins();
void speedsToSpeedAngle(int16_t left, int16_t right, int& speed, int& angle);


// Instancias globales
static SPIMaster spiMaster; // Vive para siempre en el segmento de datos
BluetoothLeConnect ble;
SensorControl sensors;
SensorData_t dataSensors;

bool autonomousMode = false;
void handleSystemState();
void processStateCommand(char command);

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
// 🏁 SETUP - Inicialización
void setup() {
    Serial.begin(115200);
    delay(1000); // Esperar a que Serial esté listo
     printResetReason();
    Serial.println("   Modo: SPI (Alta Velocidad)");
    //esp_task_wdt_init(30, false); // 30 segundos
    checkJTAGPins();
    // Configurar FreeRTOS
    setupFreeRTOS();
    sensors.begin();
    spiMaster.begin();
    ble.begin("CarRobot-FreeRTOS");
    /* Añade esto en setup() para verificar el tamaño
    Serial.printf("Tamaño de ControlCommand_t: %d bytes\n", sizeof(ControlCommand_t));
    Serial.printf("Tamaño esperado: %d bytes\n", 
              1 +    // type
              2 +    // speed
              2 +    // angle
              2 +    // distance
              4 +    // timestamp
              1 +    // priority
              1 +    // status
              1      // targetMode
); // Total: 14 bytes
*/
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

void stopRobot() {
    ControlCommand_t stopCmd;
    memset(&stopCmd, 0, sizeof(ControlCommand_t));

    stopCmd.type = CMD_STOP;
    stopCmd.speed = 0;
    stopCmd.angle = 0;
    stopCmd.distance = 0;           // O CMD_EMERGENCY_STOP según la gravedad
    stopCmd.timestamp = millis();
    stopCmd.priority = 10;          // Prioridad alta
    stopCmd.status = 0x01;
    stopCmd.targetMode = MODE_MANUAL;  // Al detenerse, solemos pasar a manual por seguridad

    // Intentamos enviarlo al frente de la cola para que sea inmediato
    if (xControlQueue != NULL) {
        if (xQueueSend(xControlQueue, &stopCmd, 0) != pdTRUE) {
            Serial.println("⚠️ Fallo crítico: Cola de control llena al intentar parar");
        } else {
            Serial.println("🛑 Comando STOP enviado a la cola");
        }
    }
}
/*
void updateRobotLogic() {
    SensorData_t sensorsData = dataSensors;
    ControlCommand_t nextCmd;
    memset(&nextCmd, 0, sizeof(ControlCommand_t));

    // 1. Obtener la última lectura de sensores (desde la cola que llena la spiMasterTask)
    if (xQueuePeek(xSensorQueue, &sensorsData, 0) == pdTRUE) {
        uint32_t now = millis();
        // VALIDACIÓN: ¿Están los datos frescos?
        bool sonarAlive = (now - sensorsData.lastSonarUpdate < 500); // 500ms timeout
        bool tofsAlive = (now - sensorsData.lastTofUpdate < 500);
        if (!sonarAlive || !tofsAlive) {
            Serial.println("🚨 ERROR: Sensores desactualizados. Modo SEGURO.");
            stopRobot();
            return;
        }
        
        // 2. Lógica de decisión
        if (sensorsData.sonarDistance < 30) {
            // Peligro: Generar comando de parada o giro
            nextCmd.type = CMD_EMERGENCY_STOP;
            nextCmd.priority = 10; // Máxima prioridad
        } else {
            // Todo despejado: Seguir adelante
            nextCmd.type = CMD_MOVE_FORWARD;
            nextCmd.speed = 150;
        }
        // 3. ENVIAR A LA COLA (No al motor directamente)
        // La spiMasterTask recogerá esto y lo mandará por SPI
        xQueueSend(xControlQueue, &nextCmd, 0);
        //procesarNavegacion(sensorsData);
    }
}*/
// En el Master, en setup() o cuando inicies:
void resetSlaveEmergency() {
    Serial.println("🔄 Reseteando emergencia en Slave...");   
    // 1. Enviar STOP para resetear emergencia
    ControlCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = CMD_STOP;
    cmd.targetMode = MODE_MANUAL;
    cmd.timestamp = millis();
    cmd.priority = 10;
    spiMaster.sendCommand(&cmd);
    delay(100);
    
    // 2. Enviar SET_MODE a MANUAL
    cmd.type = CMD_SET_MODE;
    cmd.targetMode = MODE_MANUAL;
    
    spiMaster.sendCommand(&cmd);
    delay(100);
    
    Serial.println("✅ Slave reseteado (emergencia limpiada)");
}
void executeSPICommand(ControlCommand_t cmd) {
    // Asegurar timestamp y prioridad
    if (cmd.timestamp == 0) cmd.timestamp = millis();
    if (cmd.priority == 0) cmd.priority = 1; // Prioridad por defecto
    
    // Determinar modo basado en autonomousMode si no está especificado
    if (cmd.targetMode != MODE_AUTO && cmd.targetMode != MODE_MANUAL) {
        cmd.targetMode = autonomousMode ? MODE_AUTO : MODE_MANUAL;
    }
    
    // Log según prioridad
    if (cmd.priority >= 9) {
        Serial.printf("🚨 URGENTE: Cmd 0x%02X | Prio: %d | Mode: %s\n",
                     cmd.type, cmd.priority,
                     cmd.targetMode == MODE_AUTO ? "AUTO" : "MANUAL");
    } else if (cmd.priority >= 5) {
        Serial.printf("⚠️  ALTA: Cmd 0x%02X | Prio: %d\n", cmd.type, cmd.priority);
    } else {
        // Log reducido para comandos normales
        // Serial.printf("[SPI] Cmd: 0x%02X\n", cmd.type);
    }
    cmd.timestamp = millis();
    // Enviamos la estructura completa a la cola
        if (xQueueSend(xControlQueue, &cmd, 0) != pdTRUE) {
            Serial.println("⚠️ Cola de control llena, comando descartado");
        }   
}

// ✅ SETUP FREERTOS SIMPLIFICADO
void setupFreeRTOS() {
    Serial.println("🔧 Inicializando FreeRTOS...");
    
    // Crear colas
    xControlQueue = xQueueCreate(10, sizeof(ControlCommand_t));
    xSensorQueue = xQueueCreate(1, sizeof(SensorData_t));
    spi_mutex = xSemaphoreCreateMutex();
    SensorData_t vacio = {0};
    xQueueOverwrite(xSensorQueue, &vacio);
    
    if (xControlQueue == NULL || xSensorQueue == NULL ) {
        Serial.println("❌ ERROR: No se pudo crear xControlQueue");
        while(1);
    }

    // Crear tasks
    // Tasks en Core 1 (Ordenadas por prioridad real)
    Serial.println("   Creando task SPI...");
    xTaskCreatePinnedToCore(
        spiMasterTask,
        "SPI_Master", 
        8192,  // ✅ Aumentar stack para SPI
        &spiMaster,   // ✅ Sin parámetros complejos
        3,  // Prioridad alta para SPI
        &xSPITaskHandle,
        1
    );
    // Tasks comunes
    xTaskCreatePinnedToCore(
        safetyTask,
        "Safety",
        4096,
        NULL,
        5, //TASK_PRIORITY_SAFETY,
        &xSafetyTaskHandle,
        1
    );

    xTaskCreatePinnedToCore(
    sensorRead_Task,     // Función
    "SensorRead",        // Nombre
    4096,               // ← AUMENTA ESTE VALOR (ej: 8192 o 16384)
    NULL,               // Parámetros
    3, //configMAX_PRIORITIES - 2,
    &xSensorRead_Handle,
    1
);
    
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
    xTaskCreatePinnedToCore(
        bleTask,
        "BLE",
        8192,
        NULL,
        1, //TASK_PRIORITY_BLE,
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

void spiMasterTask(void *pvParameters) {
    SPIMaster &spiMaster = *(SPIMaster *)pvParameters;
    // 1. Inicializamos con STOP para que el robot nazca frenado
    ControlCommand_t lastCmd = {CMD_STOP, 0, 0, 0}; 
    uint32_t lastBleUpdate = millis(); // Empezamos contando desde ahora
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for(;;) {
        ControlCommand_t newCmd;
        
        // A. Intentamos actualizar el comando desde la cola BLE
        if (xQueueReceive(xControlQueue, &newCmd, 0) == pdTRUE) {
            lastCmd = newCmd;
            lastBleUpdate = millis();
        }

        // B. SEGURIDAD: Si han pasado > 500ms sin datos de la App, sobreescribimos a STOP
        if (millis() - lastBleUpdate > 500) {
            lastCmd.type = CMD_STOP;
            lastCmd.speed = 0;
            // No reseteamos lastBleUpdate para que siga entrando aquí hasta que conecte BLE
        }

        // C. Ejecución de la comunicación SPI
        if (xSemaphoreTake(spi_mutex, pdMS_TO_TICKS(10))) {
            
            // Prioridad 1: Si hay una intención de movimiento o stop forzado, enviamos
            // Nota: Enviamos lastCmd SIEMPRE o bajo una condición de "DRIVE"
            if (lastCmd.type == CMD_STOP) {
                xQueueReset(xControlQueue); // Limpiar comandos acumulados en el Master
                spiMaster.sendCommand(&lastCmd);     
            }// Si el comando es de movimiento
            else if (lastCmd.speed > 0) {
                // Enviamos el movimiento tal cual lo recibimos
                spiMaster.sendCommand(&lastCmd);
            } 
            else {
                // Prioridad 2: Si el robot ya está en stop, pedimos telemetría
                spiMaster.requestSensorData();
            }

            xSemaphoreGive(spi_mutex);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));
    }
}

// Task ESPECÍFICA para sensores (alta frecuencia)
void sensorRead_Task(void *pvParameters) {

    SensorData_t sensorData;
    memset(&sensorData, 0, sizeof(SensorData_t));

    // Define el período de lectura (e.g., 20ms -> 50Hz)
    const TickType_t xDelay = pdMS_TO_TICKS(20); 
    TickType_t xLastWakeTime;
        // 5. Para lectura secuencial de TOF
    uint8_t currentTofSensor = 0;
    uint8_t tofCycle = 0;
    uint16_t val = 0;

    xLastWakeTime = xTaskGetTickCount();

    while(1) {
        // Monitorear stack
        UBaseType_t freeStack = uxTaskGetStackHighWaterMark(NULL);
        printf("SensorRead Stack free: %u bytes\n", freeStack * sizeof(StackType_t));
         // Espera hasta el siguiente ciclo de 20ms (ejecución periódica)
        vTaskDelayUntil(&xLastWakeTime, xDelay); 
        // 1. Obtener el estado actual de la cola para NO borrar el Sonar
        xQueuePeek(xSensorQueue, &sensorData, 0);
        // --- Lógica del Sensor ---
        // ============================================
        // 3. LECTURA ROTATIVA DE SENSORES TOF (Multiplexado cada 150ms)
        // 2. Leer sensor rotativo
        val = sensors.readSensor(currentTofSensor);
        // ============================================
        // 2. Actualizar SOLO los TOF (lógica rotativa)
        if (++tofCycle >= 3) {
            tofCycle = 0;
        // 3. Actualizar campo específico
            if(currentTofSensor == SENSOR_FRONT) sensorData.tofFront = val;
            else if(currentTofSensor == SENSOR_LEFT) sensorData.tofLeft = val;
            else if(currentTofSensor == SENSOR_RIGHT) sensorData.tofRight = val;
            // 4. Actualizar salud y tiempo
            sensorData.lastTofUpdate = millis();
            sensorData.sensorStatus |= (1 << 1); // Set bit 1 (TOF OK)
            currentTofSensor = (currentTofSensor + 1) % 3;
        }

        // --- Comunicación ---
        // Envía el dato a la cola. Espera 0 ticks (no bloquear)
        xQueueOverwrite(xSensorQueue, &sensorData); 
        // Usamos Overwrite porque solo queremos el dato más reciente
        spiMaster.evaluarEmergenciaInmediata(sensorData);

    }
}    // Tarea crítica: Lectura del sensor


void navigationTask(void *pvParameters) {
    Serial.println("🧭 Navigation Task - Iniciada");
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 200ms = 5Hz
    
    ControlCommand_t command;
    SensorData_t sensorData;
    uint32_t navigationCycle = 0;
    int speed, angle;
    
    // Instancia del algoritmo de evasión
    ObstacleAvoidance obstacleAvoidance;
    
    // Configuración inicial (ajusta según tus necesidades)
    obstacleAvoidance.setStrategy(ObstacleAvoidance::SIMPLE_TURN);
    obstacleAvoidance.setSafetyDistance(150);   // 15cm
    obstacleAvoidance.setTurnSpeed(600);        // Velocidad de giro
    
    vTaskDelay(pdMS_TO_TICKS(100)); 
    
    for(;;) {
        navigationCycle++;
        
        // Solo operar si el sistema está listo y en modo autónomo
        if(autonomousMode) { 
            // 1. Obtener datos de sensores
            if(xQueuePeek(xSensorQueue, &sensorData, 0) == pdTRUE) {
                // 2. Calcular comando usando ObstacleAvoidance
                ControlOutput output = obstacleAvoidance.calculateCommand(
                    sensorData.sonarDistance,
                    sensorData.tofFront,
                    sensorData.tofLeft,
                    sensorData.tofRight
                );
               
                // 2. Convertir ControlOutput a ControlCommand_t
                memset(&command, 0, sizeof(ControlCommand_t));
                command.timestamp = millis();
                command.priority = output.priority;
                command.targetMode = MODE_AUTO;

                // 3. Convertir velocidades diferenciales a speed/angle
                speedsToSpeedAngle(output.leftSpeed, output.rightSpeed, speed, angle);
                command.type = (speed == 0) ? CMD_STOP : CMD_DRIVE;
                command.speed = speed;
                command.angle = angle;
                command.priority = output.priority;
                                
                // 3. Enviar comando a la cola de control
                if(xQueueSend(xControlQueue, &command, 0) != pdTRUE) {
                    Serial.println("⚠️ Cola de navegación llena - Comando descartado");
                } else if (navigationCycle % 5 == 0) { // Log cada 5 ciclos
                    Serial.printf("🧭 Ciclo %d - Comando: type=0x%02X, speed=%d, angle=%d\n", 
                                 navigationCycle, command.type, command.speed, command.angle);
                }
            }
            else {
                // No hay datos de sensores disponibles
                if (navigationCycle % 20 == 0) {
                    Serial.println("⚠️ Navigation: Sin datos de sensores");
                }
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 🚨 TASK DE SEGURIDAD (Máxima Prioridad)
void safetyTask(void *pvParameters) {
    Serial.println("🛡️ Safety Task - Iniciada");
    SensorData_t sensorData;
    bool lastEmergencyState = false;
    uint32_t failCount = 0;
    const uint32_t MAX_FAILS = 3; // Necesitamos 3 fallos seguidos para saltar

    vTaskDelay(pdMS_TO_TICKS(2000)); // Espera de inicialización

    for(;;) {
        if(xQueuePeek(xSensorQueue, &sensorData, 0) == pdTRUE) {
            uint32_t ahora = millis();
            
            // 1. Validar frescura (1.5s de margen para ser permisivos con el SPI)
            bool sonarAlive = (ahora - sensorData.lastSonarUpdate < 1500);
            bool tofAlive = (ahora - sensorData.lastTofUpdate < 1500);
            bool sensoresVivos = sonarAlive && tofAlive;

            // 2. Lógica de Disparo
            if(sensorData.emergency || !sensoresVivos) {
                failCount++;
            } else {
                failCount = 0; // Reset si todo está bien
            }

            // 3. Ejecutar Emergencia
            if(failCount >= MAX_FAILS && !lastEmergencyState) {
                if(!sensoresVivos) {
                    Serial.printf("🚨 EMERGENCY - Timeout! (Sonar:%d, TOF:%d)\n", sonarAlive, tofAlive);
                } else {
                    Serial.println("🚨 EMERGENCY - Colisión Inminente!");
                }

                ControlCommand_t emergencyCommand = {0};
                emergencyCommand.type = CMD_STOP;
                emergencyCommand.priority = 255; // Prioridad absoluta
                emergencyCommand.targetMode = MODE_MANUAL;

                //xQueueOverwrite(xControlQueue, &emergencyCommand); // Overwrite para saltar la cola
                if(xQueueSendToFront(xControlQueue, &emergencyCommand, 0) != pdTRUE) {
                    Serial.println("⚠️ Cola llena - Fallo crítico de emergencia");
            }
                autonomousMode = false;
                lastEmergencyState = true;
            }
            
            // 4. Reset
            else if (!sensorData.emergency && sensoresVivos && lastEmergencyState) {
                Serial.println("✅ Condiciones normales restauradas.");
                lastEmergencyState = false;
                failCount = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz es suficiente para seguridad mecánica
    }
}

void bleTask(void *pvParameters) {
    //ble.begin("ESP32_SPI_Master");
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 50ms = 20Hz
    for(;;) {
        ble.update();
        // Obtener último comando BLE como estructura
        if (ble.hasNewCommand) { // Solo si realmente llegó algo nuevo por BLE
            ControlCommand_t cmd = ble.getLastCommand();
 
            // Asegurar timestamp actual si no viene
                if (cmd.timestamp == 0) {
                    cmd.timestamp = millis();
                }
            // Debug
            Serial.printf("🎮 BLE Cmd: type=0x%02X, speed=%d, angle=%d\n",
                         cmd.type, cmd.speed, cmd.angle);            
        
            // 1. Los paros de emergencia siempre deben pasar
            if (cmd.type == CMD_STOP) {
                xQueueReset(xControlQueue);
                xQueueSend(xControlQueue, &cmd, pdMS_TO_TICKS(10));
            } 
            // 2. Si no es emergencia, verificamos el modo
            else if (cmd.targetMode == MODE_MANUAL) {
                autonomousMode = false;  // Desactivar modo autónomo
                xQueueSend(xControlQueue, &cmd, 0);
            } 
            // 3. Si es autónomo y NO es un heartbeat, avisamos que ignoramos el comando
            else if (cmd.type != CMD_HEARTBEAT && cmd.targetMode == MODE_AUTO) {
                autonomousMode = true; // Activamos modo autónomo
                Serial.println("⚠️ Modo Autónomo activo - Comando manual ignorado");
            }          
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


    if (spiMaster.getCleanSensorData(distancia, estadoSlave)) {
        // --- Lógica de navegación ---
        if (distancia < 30) {
            Serial.printf("⚠️ Obstáculo a %d cm. Girando...\n", distancia);
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
    vTaskDelay(pdMS_TO_TICKS(50)); // Esperar 100ms sin bloquear el Core 1
    
    
}