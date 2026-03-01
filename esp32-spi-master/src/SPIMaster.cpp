#include "SPIMaster.h"
#include "SPIDefinitions.h"
#include <esp_err.h>
#include <esp_log.h>

static WORD_ALIGNED_ATTR SPIFrame_t master_tx_buffer;
static WORD_ALIGNED_ATTR SPIFrame_t master_rx_buffer; 

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
    //memset(&cmd, 0, sizeof(cmd));
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
        dist = master_rx_buffer.payload.distance;
        st = master_rx_buffer.payload.status;
        return true;
    }    
    return false;
}


bool SPIMaster::requestSensorData() {
    if (!initialized) return false;
    extern QueueHandle_t xSensorQueue;
    SensorData_t sensorData;
    uint16_t dist = 0;
    uint8_t stat = 0;
    // 1. Preservar datos TOF existentes
    if (xQueuePeek(xSensorQueue, &sensorData, 0) == pdTRUE) {
        memset(&sensorData, 0, sizeof(SensorData_t));
        // Guardamos los TOF que ya teníamos
        uint16_t tofFront = sensorData.tofFront;
        uint16_t tofLeft = sensorData.tofLeft;
        uint16_t tofRight = sensorData.tofRight;
        // 2. Leer sensores del Slave
        if (getCleanSensorData(dist, stat)) {
            // Actualizar solo lo que corresponde al Slave
            sensorData.sonarDistance = dist;
            sensorData.lastSonarUpdate = millis();
            sensorData.sensorStatus |= (1 << 0); // Bit 0 = Sonar OK
            // El Slave también puede reportar una emergencia interna en 'stat'
        if (stat & 0x80) sensorData.emergency = true;
            // Restaurar TOFs
            sensorData.tofFront = tofFront;
            sensorData.tofLeft = tofLeft;
            sensorData.tofRight = tofRight;
            
            // Actualizar cola
            xQueueOverwrite(xSensorQueue, &sensorData);
            
            // Evaluar emergencias
            evaluarEmergenciaInmediata(sensorData);
            return true;
        } else {
            // Marcar error de sensor
            sensorData.sensorStatus &= ~(1 << 0); // Clear bit 0
            xQueueOverwrite(xSensorQueue, &sensorData);
            return false;
        }
    }
    
    return false;
}


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
        stopCmd.priority = 255; // Máxima prioridad posible
        stopCmd.timestamp = millis();
        
        // Enviamos al frente de la cola para que sea lo primero que procese SPIMaster
        extern QueueHandle_t xControlQueue;
        if (xControlQueue != NULL) {
            xQueueSendToFront(xControlQueue, &stopCmd, 0);
            // Opcional: Log para saber que la emergencia se activó por hardware
            // Serial.println("🚨 [EMERGENCIA HARDWARE] ¡Obstáculo muy cercano!");
        }
    }
}

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
/*
bool SPIMaster::sendCommandWithTimeout(const ControlCommand_t* cmd, uint32_t timeout_ms) {
    // 1. Limpiar el frame
    memset(&master_tx_buffer, 0, sizeof(SPIFrame_t));
    
    // 2. Sello de inicio
    master_tx_buffer.magic_word = SPI_MAGIC_MASTER; // Usar la constante 0xA5
    
    // 3. Copiar el comando íntegro (14 bytes)
    memcpy(&master_tx_buffer.payload, cmd, sizeof(ControlCommand_t));
    
    // 4. Calcular la suma de los 14 bytes del payload
    //master_tx_buffer.checksum = calcularChecksum(&master_tx_buffer.payload, sizeof(ControlCommand_t));
    master_tx_buffer.checksum = calcularChecksum(&master_tx_buffer, sizeof(SPIFrame_t) - 2);
    // 5. Configurar transacción por el tamaño total del FRAME (17 bytes)
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));
    trans.length = sizeof(SPIFrame_t) * 8; // 17 bytes * 8 bits = 136 bits
    trans.tx_buffer = &master_tx_buffer;
    trans.rx_buffer = &master_rx_buffer;

    return (spi_device_transmit(spi, &trans) == ESP_OK);
}
*/
bool SPIMaster::sendCommand(const ControlCommand_t *cmd) {
    if (!cmd || !initialized) return false;
    if (cmd->type == CMD_DRIVE) {
        memcpy(&last_drive_payload, cmd, sizeof(ControlCommand_t));
    }
    // 1. Preparar Frame TX
    memset(&master_tx_buffer, 0, sizeof(SPIFrame_t));
    master_tx_buffer.magic_word = SPI_MAGIC_MASTER;
    memcpy(&master_tx_buffer.payload, cmd, sizeof(ControlCommand_t));
    master_tx_buffer.checksum = calcularChecksum(&master_tx_buffer, sizeof(SPIFrame_t) - 2);

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
            return true;
        } else {
            // Opcional: Log de checksum solo si necesitas depurar ruido
            // Serial.printf("CS Error: Rec %04X, Calc %04X\n", master_rx_buffer.checksum, checkRecibido);
        }
    }

    errorCounter++;
    return false;
}

SPIResponsePayload_t SPIMaster::getLastResponse() {
    return master_rx_buffer.payload;
}

bool SPIMaster::isSlaveInEmergency() {
    return (master_rx_buffer.payload.status & 0x01) != 0;
}

bool SPIMaster::isSlaveMoving() {
    return (master_rx_buffer.payload.status & 0x02) != 0;
}

void SPIMaster::testCommunication() {
    Serial.println("\n🔧 TEST DE COMUNICACIÓN SPI");
    Serial.println("===========================");
    
    ControlCommand_t testCmd;
    
    // Test 1: STOP (debería resetear emergencia)
    Serial.println("Test 1: Enviando STOP...");
    memset(&testCmd, 0, sizeof(testCmd));
    testCmd.type = CMD_STOP;
    testCmd.targetMode = MODE_MANUAL;
    testCmd.timestamp = millis();
    
    bool success = sendCommand(&testCmd);
    Serial.println(success ? "  ✅ OK" : "  ❌ FALLÓ");
    delay(200);
    
    // Test 2: SET_MODE a MANUAL
    Serial.println("Test 2: Configurando modo MANUAL...");
    testCmd.type = CMD_SET_MODE;
    testCmd.targetMode = MODE_MANUAL;
    
    success = sendCommand(&testCmd);
    Serial.println(success ? "  ✅ OK" : "  ❌ FALLÓ");
    delay(200);
    
    // Test 3: MOVE_FORWARD
    Serial.println("Test 3: Enviando MOVE_FORWARD...");
    testCmd.type = CMD_DRIVE;
    testCmd.speed = 800;
    testCmd.angle = 0;
    testCmd.targetMode = MODE_MANUAL;
    
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


