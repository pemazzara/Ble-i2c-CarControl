#include "SPISlave.h"
#include "SPIDefinitions.h"
#include "Command.h"

// ⚠️ IMPORTANTE: Buffers de memoria alineada para DMA (4 bytes aligned)
// Deben estar fuera de la clase o ser estáticos para garantizar su posición en RAM
WORD_ALIGNED_ATTR SPIFrame dma_rx_data;
WORD_ALIGNED_ATTR SPIFrame dma_tx_data;

SPISlave::SPISlave(MotorControl &motor, SonarIntegration &sonar) : motorController(motor), sonar(sonar) {
    // Asignamos los punteros a los buffers globales
    rxBuffer = &dma_rx_data;
    txBuffer = &dma_tx_data;
    // Usamos SPI2 (HSPI) que es el estándar para periféricos en S3
    spi_host = SPI2_HOST; 
}

void SPISlave::begin() {
    Serial.println("🚀 Inicializando SPI Slave (Modo DMA Hardware)...");

    // 1. Configuración del BUS (Pines)
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1, // No usado
        .quadhd_io_num = -1, // No usado
        .max_transfer_sz = 4096, // Tamaño máximo DMA
    };

    // 2. Configuración del SLAVE
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = SPI_SS, // Pin Chip Select
        .flags = 0,
        .queue_size = 3,        // Permitir encolar hasta 3 transacciones
        .mode = 0,              // SPI Mode 0 (CPOL=0, CPHA=0)
        .post_setup_cb = NULL,
        .post_trans_cb = NULL
    };

    // 3. Inicializar Driver + DMA (Canal Auto)
    // SPI_DMA_CH_AUTO selecciona el canal DMA disponible automáticamente
    esp_err_t ret = spi_slave_initialize(spi_host, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    
    if (ret == ESP_OK) {
        initialized = true;
        Serial.println("✅ SPI Slave Hardware Configurado Correctamente.");
        Serial.printf("   Pines: CLK=%d, MOSI=%d, MISO=%d, SS=%d\n", 
                      SPI_CLK, SPI_MOSI, SPI_MISO, SPI_SS);
        
        // ⚠️ CRÍTICO: Debemos encolar la primera transacción INMEDIATAMENTE
        // para estar listos antes de que el Master envíe nada.
        queueNextTransaction();
        
    } else {
        Serial.printf("❌ ERROR iniciando SPI Slave: %d\n", ret);
    }
}

void SPISlave::queueNextTransaction() {
    if (!initialized) return;

    // 1. Preparar los datos que enviaremos (Status actual)
    prepareResponse(*txBuffer);
    
    // 2. Limpiar buffer de recepción (opcional, por limpieza)
    memset(rxBuffer, 0, sizeof(SPIFrame));

    // 3. Configurar la transacción hardware
    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.length = sizeof(SPIFrame) * 8; // Longitud en BITS
    t.tx_buffer = txBuffer;          // Datos a enviar (MISO)
    t.rx_buffer = rxBuffer;          // Donde guardar datos recibidos (MOSI)
    
    // 4. Cargar al hardware (No bloquea, solo lo deja listo)
    // portMAX_DELAY espera si la cola está llena, pero como procesamos una a una, no pasará.
    esp_err_t ret = spi_slave_queue_trans(spi_host, &t, portMAX_DELAY);
    
    if (ret != ESP_OK) {
        Serial.println("⚠️ Error encolando transacción SPI");
    }
}

void SPISlave::processSPICommunication() {
    if (!initialized) return;

    spi_slave_transaction_t *r_trans;
    
    // Consultar al hardware: "¿Ya terminó el Master de hablar?"
    // Usamos wait=0 (sin espera) para no bloquear el loop del motor
    esp_err_t ret = spi_slave_get_trans_result(spi_host, &r_trans, 0);

    if (ret == ESP_OK) {
        // --- ¡TRANSACCIÓN COMPLETADA! ---
        
        // 1. Procesar lo que nos dijo el Master
        // (r_trans->rx_buffer ya tiene los datos, que es dma_rx_data)
        if (rxBuffer->isValid()) {
            Serial.printf("📥 SPI Cmd recibido: 0x%02X\n", rxBuffer->command);
            processCommand(*rxBuffer);
        } else {
            // Opcional: log de error de checksum
            // Serial.println("CRC Error en trama SPI");
        }

        // 2. ⚠️ RECARGAR EL ARMA: Preparar inmediatamente la siguiente
        queueNextTransaction();
    }
}

void SPISlave::processCommand(const SPIFrame &rxFrame) {
    // Si recibimos cualquier comando que NO sea de emergencia, resetear
    if (rxFrame.command != CMD_EMERGENCY_STOP) {
        motorController.resetEmergency();
    }
    switch(rxFrame.command) {
        case CMD_EMERGENCY_STOP:
            motorController.emergencyStop();
            break;  
        case CMD_MANUAL_FORWARD:
            motorController.handleCommand(CMD_MANUAL_FORWARD, rxFrame.data1);
            break;
        case CMD_MANUAL_BACKWARD:
            motorController.handleCommand(CMD_MANUAL_BACKWARD, rxFrame.data1);
            break;
        case CMD_MANUAL_LEFT:
            motorController.handleCommand(CMD_MANUAL_LEFT, rxFrame.data1);
            break;
        case CMD_MANUAL_RIGHT:
            motorController.handleCommand(CMD_MANUAL_RIGHT, rxFrame.data1);
            break;
        case CMD_STOP:
            motorController.stop();
            break;
        case CMD_SET_MANUAL_SPEED:
            motorController.handleCommand(CMD_SET_MANUAL_SPEED, rxFrame.data1);
            break;
        case CMD_READ_SENSORS:
            // No hacemos nada específico, el Master solo quería leer
            // y los datos ya se enviaron en esta misma transacción (MISO)
            break;
    }
}

void SPISlave::prepareResponse(SPIFrame &txFrame) {
    static uint32_t responseCount = 0;
    uint8_t motorStatus = motorController.getMotorStatus();
        // Rellenar status
    txFrame.status = motorStatus;
    // Solo log cada 100 respuestas para no saturar
    if (responseCount++ % 100 == 0) {
        Serial.printf("📤 Slave Response #%d - Status: 0x%02X\n", 
                     responseCount, motorStatus);
    }
    
    // Leer distancia del sonar
    uint16_t sonarDistance = sonar.updateAndGetDistance(); 
    txFrame.sensor_high = (sonarDistance >> 8) & 0xFF;
    txFrame.sensor_low = sonarDistance & 0xFF;
    
    // Comandos y datos a 0 o eco (opcional)
    txFrame.command = 0; 
    txFrame.data1 = 0;
    
    // Calcular checksum final para que el Master valide
    txFrame.calculateChecksum();
}

// --- TASK ---

void spiSlaveTask(void *pvParameters) {
    Serial.println("📡 SPI Slave Task iniciada (CORE 1)");
    
    // Instanciar objetos locales a la tarea
    MotorControl motorController;
    SonarIntegration sonar;
    SPISlave spiSlave(motorController, sonar);
    
    // Iniciar hardware
    motorController.begin();
    spiSlave.begin();
    
    for(;;) {
        // 1. Revisar si hubo comunicación SPI (No bloqueante)
        spiSlave.processSPICommunication();
        
        // 2. Actualizar seguridad de motores (Watchdog, rampas, etc)
        motorController.updateSafety();
        
        // 3. Pequeño yield para alimentar al Watchdog del RTOS
        // Al usar DMA, podemos permitirnos esto sin miedo a perder bits
        vTaskDelay(1); 
    }
}