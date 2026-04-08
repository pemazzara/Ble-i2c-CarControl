#include "SPIMaster.h"
#include "SPIDefinitions.h"
#include <esp_err.h>
#include <esp_log.h>

static WORD_ALIGNED_ATTR SPIFrame_t master_tx_buffer;
static WORD_ALIGNED_ATTR SPIResponseFrame_t master_rx_buffer;
extern SensorData_t globalSensorData; 
extern SemaphoreHandle_t sensorMutex;
 

SPIMaster::SPIMaster() : initialized(false), spi(nullptr) {
    // Inicializamos la copia de seguridad con valores neutros
    memset(&last_drive_payload, 0, sizeof(ControlCommand_t));
    last_drive_payload.type = CMD_DRIVE;
    last_drive_payload.speed = 0;
    last_drive_payload.angle = 0;
    memset(&master_tx_buffer, 0, sizeof(SPIFrame_t));
    memset(&master_rx_buffer, 0, sizeof(SPIResponseFrame_t));
}
bool SPIMaster::testSPI() {
    Serial.println("\n=== TEST SPI BÁSICO ===");
    
    if (!initialized) {
        Serial.println("❌ SPI no inicializado");
        return false;
    }
    
    // Test 1: Enviar byte simple
    uint8_t test_data[] = {0xAA, 0x55, 0x01, 0x02};
    
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    
    trans.length = sizeof(test_data) * 8;
    trans.tx_buffer = test_data;
    trans.rx_buffer = nullptr;
    
    Serial.printf("Enviando %d bytes de prueba...\n", sizeof(test_data));
    
    esp_err_t ret = spi_device_transmit(spi, &trans);
    
    if (ret == ESP_OK) {
        Serial.println("✅ Test SPI exitoso");
        return true;
    } else {
        Serial.printf("❌ Test SPI fallido: %s\n", esp_err_to_name(ret));
        return false;
    }
}
bool SPIMaster::begin() {
    Serial.println("[SPI Master] Inicializando...");
    // Configurar pines
    pinMode(SPI_MASTER_SS, OUTPUT);
    digitalWrite(SPI_MASTER_SS, HIGH); // Desactivar CS por defecto
    
    // 1. Configuración del bus SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MASTER_MOSI,
        .miso_io_num = SPI_MASTER_MISO,
        .sclk_io_num = SPI_MASTER_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = 0,
        .intr_flags = 0
    };
    
    // 2. Configuración del dispositivo SPI
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,                          // SPI Mode 0
        .duty_cycle_pos = 128,              // 50% duty cycle
        .cs_ena_pretrans = 0,               // No pre-transaction CS
        .cs_ena_posttrans = 0,              // No post-transaction CS
        .clock_speed_hz = 1000000,         // 1 MHz
        .input_delay_ns = 0,
        .spics_io_num = SPI_MASTER_SS,
        .flags = 0,    
        .queue_size = 1,                // Tamaño de cola
        .pre_cb = NULL,
        .post_cb = NULL,

    };
    
    // 3. Inicializar bus SPI
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("[SPI Master] ❌ Error init bus: %s\n", esp_err_to_name(ret));
        return false;
    }
    
    // 4. Añadir dispositivo al bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        Serial.printf("[SPI Master] ❌ Error add device: %s\n", esp_err_to_name(ret));
        return false;
    }
    initialized = true;
    // Test rápido de SPI
    Serial.println("[SPI Master] 🔧 Probando comunicación SPI...");
    if (!testSPI()) {
        Serial.println("[SPI Master] ⚠️ Test SPI falló, pero continuando...");
    }

    Serial.println("[SPI Master] ✅ Inicializado correctamente");
    Serial.printf("   Frecuencia: %d Hz, Mode: %d\n", devcfg.clock_speed_hz, devcfg.mode);
    
    
    return true;
}

bool SPIMaster::reconnect() {
    Serial.println("[SPI] Re-inicializando bus por errores persistentes...");
    // Remover el dispositivo del bus
    spi_bus_remove_device(spi);
    // Volver a configurar
    return begin(); 
}

bool SPIMaster::getCleanSensorData(uint16_t &dist, uint8_t &st) {
    // 1. Primer intento: "Despertar" al slave y pedir lectura
    ControlCommand_t cmd = last_drive_payload;
    cmd.type = CMD_READ_SENSORS;
    
    if (!sendCommand(&cmd)) return false;
    // 2. Espera estratégica 
    // Damos tiempo a que la tarea del Slave procese 'processCommand' 
    // y llame a 'prepareResponse' para la siguiente transacción.
    delayMicroseconds(500); 

    // 3. Segundo intento: Recoger la respuesta preparada
    cmd.type = CMD_STATUS; 
    if (sendCommand(&cmd)) {
        // La respuesta válida ya está en master_rx_buffer
        dist = master_rx_buffer.payload.motors.distance;
        st = master_rx_buffer.payload.motors.motor_flags;
        return true;
    }    
    return false;
}

/*
bool SPIMaster::requestSensorData() {
    if (!initialized) return false;
    
    // 1. Definir variables temporales para la lectura SPI
    uint16_t dist = 0;
    uint8_t stat = 0;

    // 2. Realizar la comunicación SPI (fuera del mutex para no bloquear el sistema)
    if (!getCleanSensorData(dist, stat)) {
         handleCommunicationErrors(master_rx_buffer.magic_word);
        // Si falla el SPI, entramos un momento para marcar el error
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            globalSensorData.sensorStatus &= ~(1 << 0); // Sonar OFF
            xSemaphoreGive(sensorMutex);
        }
        return false;
    }

    // 3. Si el SPI fue exitoso, actualizamos la estructura global
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        
        // ACTUALIZACIÓN SELECTIVA:
        // No necesitamos "preservar" los TOF manualmente porque NO vamos a 
        // sobreescribir toda la estructura, solo los campos del Sonar.
        
        globalSensorData.sonarDistance = dist;
        globalSensorData.lastSonarUpdate = millis();
        globalSensorData.sensorStatus |= (1 << 0); // Bit 0 = Sonar OK
        

        // Evaluar emergencias (el bit 0x80 del slave o distancias críticas)
        if (stat & 0x80) globalSensorData.emergency = true;
        
        evaluarEmergenciaInmediata(globalSensorData);

        xSemaphoreGive(sensorMutex);
        return true;
    }

    return false;
}
 */           
 /*          
void SPIMaster::evaluarEmergenciaInmediata(const SensorData_t &sd) {
    bool peligro = false;

    // 1. Verificar Sonar (Frontal)
    if (sd.sonarDistance > 0 && sd.sonarDistance < DISTANCIA_CRITICA_STOP) {
        peligro = true;
    }
    
    // 2. Verificar TOF Frontal
    if (sd.tofFront > 0 && sd.tofFront < DISTANCIA_CRITICA_STOP) {
        peligro = true;
    }

    // 3. Si hay peligro, enviar comando de parada DIRECTO a la cola de control
    if (peligro) {
        ControlCommand_t stopCmd;
        memset(&stopCmd, 0, sizeof(ControlCommand_t));
        
        stopCmd.type = CMD_STOP;
        stopCmd.speed = 0;
        stopCmd.angle = 0;
        stopCmd.timestamp = millis();
        
        // Enviamos al frente de la cola para que sea lo primero que procese SPIMaster
        if (xControlQueue != NULL) {
            xQueueOverwrite(xControlQueue, &stopCmd);
            // Opcional: Log para saber que la emergencia se activó por hardware
            // Serial.println("🚨 [EMERGENCIA HARDWARE] ¡Obstáculo muy cercano!");
        }
    }
}
*/
void SPIMaster::handleCommunicationErrors(uint8_t magic) {
    switch (magic) {
        case 0xFF:
            Serial.println("[SPI] 🚨 ERROR: Slave desconectado o MISO en HIGH permanente.");
            Serial.println("      Verificar: Alimentación del Slave y cable G12 (MISO).");
            break;
        case 0x00:
            Serial.println("[SPI] ⚠️ WARNING: Línea MISO en LOW (Respuesta vacía).");
            Serial.println("      Posible: El Slave no ha encolado la transacción a tiempo.");
            break;
        case 0xA5:
            Serial.println("[SPI] 🔄 ERROR DE ESPEJO: El Slave devolvió lo mismo que enviamos.");
            Serial.println("      Verificar: ¿Están puenteados MOSI y MISO?");
            break;
        default:
            Serial.printf("[SPI] ❓ Magic inesperado: 0x%02X. Posible ruido en el bus.\n", magic);
            // Si hay mucho ruido, podrías bajar la velocidad del reloj SPI aquí
            break;
    }
}
uint8_t SPIMaster::getNextMsgId() {
    static uint8_t msgCounter = 0;
    if (++msgCounter == 0) msgCounter = 1; // saltar 0
    return msgCounter;
}

bool SPIMaster::sendCommand(const ControlCommand_t *cmd) {
    if (!cmd || !initialized) return false;
    if (spi == nullptr) {
        Serial.println("❌ Error: SPI Handle es NULL!");
        return false;
    }
    if (cmd->type == CMD_DRIVE) {
        memcpy(&last_drive_payload, cmd, sizeof(ControlCommand_t));
    }
    // 1. Preparar Frame TX
    memset(&master_tx_buffer, 0, sizeof(SPIFrame_t));
    // 2. Llenar cabecera
    master_tx_buffer.msg_id = getNextMsgId();
    master_tx_buffer.magic_word = 0xA5;
    // 3. Copiar el payload (el comando)
    // Usamos memcpy por seguridad, aunque al ser estructuras packed 
    // podrías asignar directamente: 
    master_tx_buffer.payload = *cmd;
    //memcpy(&master_tx_buffer.payload, cmd, sizeof(ControlCommand_t));
    master_tx_buffer.checksum = calcularChecksum((uint8_t*)&master_tx_buffer, sizeof(SPIFrame_t) - 2);
    // 2. Limpiar RX para evitar datos antiguos
    memset(&master_rx_buffer, 0, sizeof(SPIResponseFrame_t));

    // 3. Configurar Transacción (Usando el método que demostró ser estable)
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = sizeof(SPIFrame_t) * 8;
    trans.tx_buffer = &master_tx_buffer;
    trans.rx_buffer = &master_rx_buffer;

    // 4. Transmitir (Modo Interrupción/Cola)
    if (spi_device_transmit(spi, &trans) != ESP_OK) {
        return false;
    }

    // 5. Validar Respuesta del Slave
    // Ahora que la comunicación es estable, validamos el "eco"
    if (master_rx_buffer.magic_word == SPI_MAGIC_SLAVE) {
        uint16_t checkRecibido = calcularChecksum(&master_rx_buffer, sizeof(SPIResponseFrame_t) - 2);
        
        if (checkRecibido == master_rx_buffer.checksum) {
            errorCounter = 0; // Comunicación perfecta
            processResponse(master_rx_buffer); // Procesar la respuesta para actualizar estados y sensores
            return true;
        } else {
            // Opcional: Log de checksum solo si necesitas depurar ruido
            Serial.printf("CS Error: Rec %04X, Calc %04X\n", master_rx_buffer.checksum, checkRecibido);
        }
    }

    errorCounter++;
    return false;
}

void processResponse(const SPIResponseFrame_t& res) {
    if (res.type == TYPE_SENSORS) {
        Serial.printf("Distancia: %d mm\n", res.payload.motors.distance);
    } else if (res.type == TYPE_SYSTEM) {
        Serial.printf("Progreso Calibración: %d%%\n", res.payload.system.progress);
    }
}
bool SPIMaster::processResponse(const SPIResponseFrame_t& response) {
    if (response.type == TYPE_SENSORS) {
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10)) != pdTRUE) {        
        return false; // No pudimos acceder a los datos, la respuesta no se procesó
        // 1. Actualización de estados básicos
        globalSensorData.sonarDistance = response.payload.motors.distance;
        globalSensorData.sensorStatus = response.payload.motors.motor_flags;
        globalSensorData.lastTofUpdate = millis();
        xSemaphoreGive(sensorMutex);
    }
        Serial.printf("Distancia: %d mm\n", response.payload.motors.distance);
    } else if (response.type == TYPE_SYSTEM) {
        if (response.payload.system.state == SLAVE_STATE_READY && (response.payload.system.K_fixed == 0 || response.payload.system.tau_fixed == 0)) {
            //globalSensorData.calibration_valid = false;
            xSemaphoreGive(sensorMutex);        
            Serial.println("[SPI] Inconsistencia: Slave READY sin parámetros. Re-calibrando...");
            return startCalibration(); // Esto enviará el comando en el siguiente ciclo
        }
        Serial.printf("Progreso Calibración: %d%%\n", response  .payload.system.progress);
    }

/*
    // 2. Verificación de Integridad de Calibración
    // Si el Slave dice estar listo pero no envió parámetros válidos


    // 3. Manejo de Emergencia
    if (response.slave_state == SLAVE_STATE_CALIBRATION) {
        globalSensorData.emergency = false;
    } else {
        // Marcamos emergencia si el estado es EMERGENCY o si el bit 0 de flags está activo
        globalSensorData.emergency = (response.slave_state == SLAVE_STATE_EMERGENCY) ||
                                     ((response.progress_or_flags & 0x01) != 0);
    }

    // 4. Procesamiento de parámetros K y Tau (Punto Fijo)
    if (response.K_fixed > 0 && response.tau_fixed > 0) {
        globalSensorData.calibrationK = response.K_fixed / 1000.0f;
        globalSensorData.calibrationTau = response.tau_fixed / 100.0f;
        globalSensorData.calibrationValid = true;
    } else {
        globalSensorData.calibrationValid = false;
    }

    // Notificamos éxito al sistema
    SystemState::notifySyncSuccess();
    errorCounter = 0;
*/
    xSemaphoreGive(sensorMutex);
    return true;
}

bool SPIMaster::startCalibration() {
    ControlCommand_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.type = CMD_CALIBRATE;
    cmd.timestamp = millis();
    Serial.println("[SPI Master] Implementar calibración...");
    return false; // Por ahora solo logueamos, la implementación real dependerá de tu sistema de control
    //return sendCommand(&cmd); // Enviar el comando de calibración, la respuesta se procesará en el siguiente ciclo
}

SPIResponseFrame_t SPIMaster::getLastResponse() {
    return master_rx_buffer;
}

bool SPIMaster::isSlaveInEmergency() {
    return (master_rx_buffer.payload.motors.motor_flags & 0x01) != 0;
}

bool SPIMaster::isSlaveMoving() {
    return (master_rx_buffer.payload.motors.motor_flags & 0x02) != 0;
}

void SPIMaster::testCommunication() {
    Serial.println("\n🔧 TEST DE COMUNICACIÓN SPI");
    Serial.println("===========================");
    
    ControlCommand_t testCmd;
    
    // Test 1: STOP (debería resetear emergencia)
    Serial.println("Test 1: Enviando STOP...");
    memset(&testCmd, 0, sizeof(testCmd));

    testCmd.type = CMD_STOP;
    testCmd.speed = 0;
    testCmd.angle = 0;
    testCmd.timestamp = millis();
    
    bool success = sendCommand(&testCmd);
    Serial.println(success ? "  ✅ OK" : "  ❌ FALLÓ");
    delay(200);
    
    // Test 2: SET_MODE a MANUAL
    Serial.println("Test 2: Configurando modo MANUAL...");

    testCmd.type = CMD_SET_MODE;
    testCmd.speed = 0;
    testCmd.angle = 0;
    testCmd.timestamp = millis();
    
    success = sendCommand(&testCmd);
    Serial.println(success ? "  ✅ OK" : "  ❌ FALLÓ");
    delay(200);
    
    // Test 3: MOVE_FORWARD
    Serial.println("Test 3: Enviando MOVE_FORWARD...");
    testCmd.type = CMD_DRIVE;
    testCmd.speed = 800;
    testCmd.angle = 0;
    testCmd.timestamp = millis();
    
    success = sendCommand(&testCmd);
    if (success) {
        Serial.println("  ✅ OK");
        if (isSlaveInEmergency()) {
            Serial.println("  ⚠️  Pero Slave sigue en emergencia");
        }
        if (isSlaveMoving()) {
            Serial.println("  ✅ Slave reporta movimiento");
        }
    } else {
        Serial.println("  ❌ FALLÓ");
    }
    delay(500);
    
    // Test 4: STOP de nuevo
    Serial.println("Test 4: Enviando STOP...");
    testCmd.type = CMD_STOP;
    testCmd.speed = 0;
    
    success = sendCommand(&testCmd);
    Serial.println(success ? "  ✅ OK" : "  ❌ FALLÓ");
    
    Serial.println("🔧 TEST COMPLETADO");
}


