/* 🎮 TASK DE CONTROL - VERSIÓN COMPLETA SPI
#include "SPIMaster.h"
#include "control_types.h"

// Declaraciones extern
extern QueueHandle_t xControlQueue;
extern QueueHandle_t xSensorQueue;
extern SPIMaster spiMaster;

// Variables globales del sistema (deberían estar en otro archivo)
extern volatile ControlMode currentMode;
extern volatile bool autonomousMode;

// Contadores para debug
static uint32_t spiSuccessCount = 0;
static uint32_t spiErrorCount = 0;

void controlTask(void *pvParameters) {
    Serial.println("🎮 Control Task - INICIANDO (SPI Direct)");

    // ✅ VERIFICAR DEPENDENCIAS
    if (xControlQueue == NULL) {
        Serial.println("❌ ERROR: xControlQueue no disponible");
        vTaskDelete(NULL);
        return;
    }
    
    if (xSensorQueue == NULL) {
        Serial.println("⚠️  ADVERTENCIA: xSensorQueue no disponible");
    }
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz (más rápido para mejor respuesta)
    
    ControlCommand_t currentCmd;
    uint32_t commandCount = 0;
    
    // Para comandos temporizados
    TickType_t commandEndTime = 0;
    bool commandActive = false;
    ControlCommand_t activeCommand;
    
    // Para modo autónomo
    SensorData_t sensorData;
    uint32_t autonomousCycle = 0;
    
    Serial.println("✅ Control Task - LISTA");
    
    for(;;) {
        // 1. ✅ PROCESAR COMANDOS DE CONTROL (ALTA PRIORIDAD)
        while(xQueueReceive(xControlQueue, &currentCmd, 0) == pdTRUE) {
            commandCount++;
            
            // Debug detallado para primeros comandos
            if (commandCount <= 5) {
                Serial.printf("🎮 #%d: Tipo=%d, Prio=%d, Param1=%d\n", 
                            commandCount, currentCmd.type, 
                            currentCmd.priority, currentCmd.param1);
            }
            
            // Procesar el comando
            processControlCommand(currentCmd, commandEndTime, 
                                 commandActive, activeCommand);
        }
        
        // 2. ✅ EJECUTAR COMANDO TEMPORIZADO ACTIVO
        if (commandActive && xTaskGetTickCount() >= commandEndTime) {
            Serial.println("⏰ Comando temporal finalizado - STOP");
            spiMaster.sendCommand(SPI_CMD_MOTOR_STOP, 0);
            commandActive = false;
        }
        
        // 3. ✅ LÓGICA DE MODO AUTÓNOMO (si navigationTask no existe)
        if (currentMode == MODE_AUTONOMOUS && autonomousMode) {
            autonomousCycle++;
            
            // Cada 10 ciclos (~200ms) verificar sensores para seguridad
            if (autonomousCycle % 10 == 0 && xSensorQueue != NULL) {
                if (xQueueReceive(xSensorQueue, &sensorData, 0) == pdTRUE) {
                    // Verificación básica de seguridad
                    uint16_t frontDistance = sensorData.sonarDistance;
                    
                    if (frontDistance > 0 && frontDistance < 100) {
                        // Obstáculo muy cerca - emergencia
                        Serial.printf("🚨 AUTO: Obstáculo cerca %dmm - STOP\n", frontDistance);
                        ControlCommand_t emergencyCmd;
                        emergencyCmd.type = CMD_EMERGENCY_STOP;
                        emergencyCmd.priority = 10;
                        processControlCommand(emergencyCmd, commandEndTime, 
                                             commandActive, activeCommand);
                    }
                }
            }
        }
        
        // 4. ✅ HEARTBEAT / DEBUG
        static uint32_t heartbeatCounter = 0;
        heartbeatCounter++;
        if (heartbeatCounter % 250 == 0) {  // Cada 5 segundos
            Serial.printf("🎮 Control: Modo=%d, Auto=%d, Cmds=%d, SPI_OK=%d, SPI_ERR=%d\n",
                         currentMode, autonomousMode, commandCount,
                         spiSuccessCount, spiErrorCount);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ✅ FUNCIÓN PARA PROCESAR COMANDOS DE CONTROL
void processControlCommand(ControlCommand_t cmd, 
                          TickType_t &endTime, 
                          bool &active, 
                          ControlCommand_t &activeCmd) {
    
    // Comandos de MODO tienen prioridad especial
    if (cmd.type == CMD_SET_MODE) {
        handleModeChange(cmd.targetMode);
        return;
    }
    
    // Comandos de EMERGENCIA tienen máxima prioridad
    if (cmd.type == CMD_EMERGENCY_STOP) {
        handleEmergencyStop();
        active = false;
        return;
    }
    
    // Solo procesar comandos de movimiento si estamos en modo adecuado
    if (currentMode == MODE_MANUAL || 
        (currentMode == MODE_AUTONOMOUS && cmd.priority >= 5)) {
        
        // Ejecutar comando SPI
        bool success = executeSPICommand(cmd);
        
        if (success) {
            spiSuccessCount++;
            
            // Si el comando tiene duración, programar fin
            if (cmd.type == CMD_MOVE_FORWARD || cmd.type == CMD_MOVE_BACKWARD) {
                if (cmd.param2 > 0) {  // param2 = duración en ms
                    endTime = xTaskGetTickCount() + pdMS_TO_TICKS(cmd.param2);
                    active = true;
                    activeCmd = cmd;
                }
            }
        } else {
            spiErrorCount++;
        }
    } else {
        Serial.printf("⚠️  Comando %d ignorado (Modo=%d, Prio=%d)\n", 
                     cmd.type, currentMode, cmd.priority);
    }
}

// ✅ FUNCIÓN PARA EJECUTAR COMANDOS SPI
bool executeSPICommand(ControlCommand_t cmd) {
    uint8_t spiCommand = 0;
    uint8_t spiData = 0;
    bool sendCommand = true;
    String actionDesc = "";
    
    // Mapear ControlCommand_t a comandos SPI
    switch(cmd.type) {
        // =========== MOVIMIENTO BÁSICO ===========
        case CMD_MOVE_FORWARD:
            spiCommand = SPI_CMD_MOTOR_FORWARD;
            spiData = constrain(cmd.param1, 10, 100);
            actionDesc = "Adelante " + String(spiData) + "%";
            break;
            
        case CMD_MOVE_BACKWARD:
            spiCommand = SPI_CMD_MOTOR_BACKWARD;
            spiData = constrain(cmd.param1, 10, 80);
            actionDesc = "Atrás " + String(spiData) + "%";
            break;
            
        case CMD_TURN_LEFT:
            spiCommand = SPI_CMD_MOTOR_LEFT;
            spiData = constrain(cmd.param1, 10, 90);
            actionDesc = "Izquierda " + String(spiData) + "°";
            break;
            
        case CMD_TURN_RIGHT:
            spiCommand = SPI_CMD_MOTOR_RIGHT;
            spiData = constrain(cmd.param1, 10, 90);
            actionDesc = "Derecha " + String(spiData) + "°";
            break;
            
        case CMD_ROTATE_LEFT:
            spiCommand = SPI_CMD_MOTOR_ROTATE_LEFT;
            spiData = constrain(cmd.param1, 10, 180);
            actionDesc = "Rotar izquierda " + String(spiData) + "°";
            break;
            
        case CMD_ROTATE_RIGHT:
            spiCommand = SPI_CMD_MOTOR_ROTATE_RIGHT;
            spiData = constrain(cmd.param1, 10, 180);
            actionDesc = "Rotar derecha " + String(spiData) + "°";
            break;
            
        // =========== CONTROL ===========
        case CMD_STOP:
            spiCommand = SPI_CMD_MOTOR_STOP;
            actionDesc = "Parada";
            break;
            
        case CMD_EMERGENCY_STOP:
            spiCommand = SPI_CMD_EMERGENCY_STOP;
            actionDesc = "PARADA DE EMERGENCIA";
            break;
            
        // =========== CONFIGURACIÓN ===========
        case CMD_SET_SPEED:
            spiCommand = SPI_CMD_SET_MANUAL_SPEED;
            spiData = constrain(cmd.param1, 10, 100);
            actionDesc = "Velocidad base " + String(spiData) + "%";
            break;
            
        case CMD_CALIBRATE:
            spiCommand = SPI_CMD_CALIBRATE;
            actionDesc = "Calibrar";
            break;
            
        // =========== SENSORES ===========
        case CMD_GET_STATUS:
            // No es un comando SPI directo
            sendCommand = false;
            actionDesc = "Solicitar estado";
            sendSystemStatus();  // Función para enviar estado por BLE/Serial
            break;
            
        default:
            Serial.printf("❌ Comando desconocido: %d\n", cmd.type);
            return false;
    }
    
    if (sendCommand) {
        // Debug detallado (primeras ejecuciones y luego ocasional)
        static uint32_t execCount = 0;
        execCount++;
        
        if (execCount <= 10 || execCount % 50 == 0) {
            Serial.printf("🎮 SPI: %s -> Cmd=0x%02X, Data=0x%02X\n", 
                         actionDesc.c_str(), spiCommand, spiData);
        }
        
        // Enviar comando SPI
        bool success = spiMaster.sendCommand(spiCommand, spiData);
        
        if (success) {
            if (execCount % 100 == 0) {
                Serial.printf("✅ SPI éxito: %s\n", actionDesc.c_str());
            }
            return true;
        } else {
            Serial.printf("❌ SPI fallo: %s (Cmd=0x%02X)\n", 
                         actionDesc.c_str(), spiCommand);
            return false;
        }
    }
    
    return true;  // Para comandos que no requieren SPI
}

// ✅ MANEJAR CAMBIO DE MODO
void handleModeChange(ControlMode newMode) {
    ControlMode oldMode = currentMode;
    currentMode = newMode;
    
    // Detener cualquier movimiento en progreso
    spiMaster.sendCommand(SPI_CMD_MOTOR_STOP, 0);
    
    // Actualizar autonomousMode según el modo
    switch(newMode) {
        case MODE_MANUAL:
            autonomousMode = false;
            Serial.println("🔀 MODO MANUAL ACTIVADO");
            break;
            
        case MODE_AUTONOMOUS:
        case MODE_WALL_FOLLOW:
        case MODE_LINE_FOLLOW:
            autonomousMode = true;
            Serial.printf("🔀 MODO AUTÓNOMO ACTIVADO (Tipo: %d)\n", newMode);
            break;
            
        case MODE_CALIBRATION:
            autonomousMode = false;
            Serial.println("🔀 MODO CALIBRACIÓN ACTIVADO");
            break;
    }
    
    // Notificar cambio por BLE (opcional)
    // ble.sendModeChange(oldMode, newMode);
}

// ✅ MANEJAR PARADA DE EMERGENCIA
void handleEmergencyStop() {
    Serial.println("🚨🚨🚨 PARADA DE EMERGENCIA EJECUTADA 🚨🚨🚨");
    
    // 1. Enviar comando de emergencia por SPI
    spiMaster.sendCommand(SPI_CMD_EMERGENCY_STOP, 0);
    
    // 2. Cambiar a modo manual
    currentMode = MODE_MANUAL;
    autonomousMode = false;
    
    // 3. Log detallado
    Serial.println("✅ Sistema en modo manual tras emergencia");
    
    // 4. Notificar por BLE (opcional)
    // ble.sendEmergencyStop();
}

// ✅ ENVIAR ESTADO DEL SISTEMA
void sendSystemStatus() {
    Serial.println("📊 ESTADO DEL SISTEMA:");
    Serial.printf("  - Modo actual: %d\n", currentMode);
    Serial.printf("  - Modo autónomo: %s\n", autonomousMode ? "SI" : "NO");
    Serial.printf("  - Comandos SPI exitosos: %d\n", spiSuccessCount);
    Serial.printf("  - Errores SPI: %d\n", spiErrorCount);
    Serial.printf("  - Heap libre: %d bytes\n", esp_get_free_heap_size());
    
    // También enviar por BLE si está conectado
    // ble.sendStatus(currentMode, autonomousMode, spiSuccessCount, spiErrorCount);
}*/