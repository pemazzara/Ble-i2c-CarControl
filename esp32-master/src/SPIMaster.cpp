#include "SPIMaster.h"
#include "esp_task_wdt.h"
#include <cstring>

SPIMaster spiMaster;

// Buffers DMA
static WORD_ALIGNED_ATTR SPIFrame master_tx_buffer;
static WORD_ALIGNED_ATTR SPIFrame master_rx_buffer;

void SPIMaster::begin() {
    Serial.println("🚀 Inicializando SPI Master...");

    // 1. Configuración del BUS SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 32,
        .flags = 0,
        .intr_flags = 0
    };

    // Inicializar bus SPI
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error inicializando bus SPI: 0x%x\n", ret);
        return;
    }

    // 2. Configurar dispositivo SPI
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 128,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 10000000,
        .input_delay_ns = 0,
        .spics_io_num = SPI_SS,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error agregando dispositivo SPI: 0x%x\n", ret);
        return;
    }
    
    initialized = true;
    Serial.println("✅ SPI Master inicializado correctamente");
}

bool SPIMaster::sendCommand(uint8_t command, uint8_t data1, uint8_t data2, uint8_t data3) {
    if (!initialized) return false;
    
    master_tx_buffer.command = command;
    master_tx_buffer.data1 = data1;
    master_tx_buffer.data2 = data2; 
    master_tx_buffer.data3 = data3;
    master_tx_buffer.calculateChecksum();

    memset(&master_rx_buffer, 0, sizeof(SPIFrame));

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    t.length = sizeof(SPIFrame) * 8;
    t.tx_buffer = &master_tx_buffer;
    t.rx_buffer = &master_rx_buffer;

    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    
    if (ret == ESP_OK && master_rx_buffer.isValid()) {
        return (master_rx_buffer.status & 0x80) == 0;
    }
    
    Serial.printf("❌ SPI Error: Cmd=0x%02X, Ret=0x%x\n", command, ret);
    return false;
}

bool SPIMaster::requestSensorData(uint16_t &distance, uint8_t &status) {
    if (sendCommand(SPI_CMD_READ_SENSORS)) {
        distance = (master_rx_buffer.sensor_high << 8) | master_rx_buffer.sensor_low;
        status = master_rx_buffer.status;
        return true;
    }
    return false;
}

// ✅ TASK SPI MASTER SIMPLIFICADA - Sin parámetros complejos
void spiMasterTask(void *pvParameters) {
    Serial.println("📡 SPI Master Task iniciada");
    
    // ✅ OBTENER COLAS DIRECTAMENTE (asumiendo que son globales)
    // En una implementación más limpia, estas deberían pasarse como parámetros
    // pero por ahora las hacemos accesibles globalmente
    
    extern QueueHandle_t xCommandQueue;  // Declarar como extern
    extern QueueHandle_t xSensorQueue;   // Declarar como extern
    
    if (xCommandQueue == NULL || xSensorQueue == NULL) {
        Serial.println("❌ ERROR: Colas no inicializadas");
        vTaskDelete(NULL);
        return;
    }
    
    spiMaster.begin();
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);
    
    // Definir estructuras locales
    typedef struct {
        uint16_t sonarDistance;
        uint16_t tofFront;
        uint16_t tofLeft; 
        uint16_t tofRight;
        bool emergency;
    } SensorData;
    
    typedef struct {
        uint8_t cmd;
        uint8_t data;
    } Command;
    
    SensorData sensorData = {0};
    Command command;
    
    esp_task_wdt_add(NULL);
    // ✅ CONTADORES PARA DEBUG
    uint32_t sensorReads = 0;
    uint8_t cycleCount = 0;

    for(;;) {
        esp_task_wdt_reset();
        
        // 1. Procesar comandos
        while(xQueueReceive(xCommandQueue, &command, 0) == pdTRUE) {
            uint8_t spiCommand;
            switch(command.cmd) {
                case 0x01: spiCommand = SPI_CMD_MOTOR_FORWARD; break;
                case 0x03: spiCommand = SPI_CMD_MOTOR_LEFT; break;
                case 0x04: spiCommand = SPI_CMD_MOTOR_RIGHT; break;
                case 0x05: spiCommand = SPI_CMD_MOTOR_STOP; break;
                case 0x10: spiCommand = SPI_CMD_SET_MANUAL_SPEED; break;
                case 0x00: spiCommand = SPI_CMD_EMERGENCY_STOP; break;
                default:   spiCommand = command.cmd; break;
            }
            
            bool success = spiMaster.sendCommand(spiCommand, command.data);
            
            if(success) {
                Serial.printf("✅ SPI Cmd: 0x%02X\n", command.cmd);
            } else {
                Serial.printf("❌ SPI Error: 0x%02X\n", command.cmd);
            }
            // ✅ RESET WATCHDOG DURANTE PROCESAMIENTO LARGO
            esp_task_wdt_reset();
        }
        
        // 2. Leer sensores
        static uint8_t cycleCount = 0;
        if(++cycleCount >= 10) {
            uint16_t distance;
            uint8_t status;
            
            if(spiMaster.requestSensorData(distance, status)) {
                sensorData.sonarDistance = distance;
                sensorData.emergency = (status & 0x01);
                
                if(xQueueOverwrite(xSensorQueue, &sensorData) != pdTRUE) {
                    Serial.println("⚠️  Error escribiendo en cola de sensores");
                }
                
                // Debug cada 50 lecturas
                static uint32_t sensorReads = 0;
                if (++sensorReads /* % 50*/ == 0) {
                    Serial.printf("📊 SPI Sensor#%d: %dmm\n", sensorReads, distance);
                }
            }
            cycleCount = 0;
            // ✅ RESET WATCHDOG DESPUÉS DE LECTURA SENSOR
            esp_task_wdt_reset();
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
/*
#include "SPIMaster.h"
#include "SensorControl.h"
#include "esp_task_wdt.h"
#include <cstring>

SPIMaster spiMaster;

// ✅ BUFFERS DMA GLOBALES ALINEADOS
static WORD_ALIGNED_ATTR SPIFrame master_tx_buffer;
static WORD_ALIGNED_ATTR SPIFrame master_rx_buffer;

void SPIMaster::begin() {
    Serial.println("🚀 Inicializando SPI Master (DMA Optimizado)...");

    // 1. Configuración del BUS SPI
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .data4_io_num = -1,
        .data5_io_num = -1,
        .data6_io_num = -1,
        .data7_io_num = -1,
        .max_transfer_sz = 32,  // ✅ BYTES - suficiente para nuestras tramas
        .flags = 0,
        .intr_flags = 0
    };

    // Inicializar bus SPI con DMA
    esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error inicializando bus SPI: 0x%x\n", ret);
        return;
    }

    // 2. Configurar dispositivo SPI (Slave)
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,              // SPI Mode 0 (CPOL=0, CPHA=0)
        .duty_cycle_pos = 128,  // 50% duty cycle
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 10000000, // 10MHz - óptimo para cables cortos
        .input_delay_ns = 0,
        .spics_io_num = SPI_SS,
        .flags = 0,
        .queue_size = 1,        // Suficiente para polling
        .pre_cb = nullptr,
        .post_cb = nullptr
    };

    // Agregar dispositivo al bus
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        Serial.printf("❌ Error agregando dispositivo SPI: 0x%x\n", ret);
        return;
    }
    
    initialized = true;
    Serial.println("✅ SPI Master DMA inicializado correctamente");
    Serial.printf("   Frecuencia: 10MHz, Pines: %d,%d,%d,%d\n", 
                 SPI_CLK, SPI_MISO, SPI_MOSI, SPI_SS);
}

bool SPIMaster::sendCommand(uint8_t command, uint8_t data1, uint8_t data2, uint8_t data3) {
    if (!initialized) {
        Serial.println("❌ SPI Master no inicializado");
        return false;
    }
    
    // ✅ USAR BUFFERS DMA GLOBALES (no locales en stack)
    master_tx_buffer.command = command;
    master_tx_buffer.data1 = data1;
    master_tx_buffer.data2 = data2; 
    master_tx_buffer.data3 = data3;
    master_tx_buffer.calculateChecksum();

    // Limpiar buffer de recepción
    memset(&master_rx_buffer, 0, sizeof(SPIFrame));

    // ✅ INICIALIZAR ESTRUCTURA DE TRANSACCIÓN
    spi_transaction_t t;
    memset(&t, 0, sizeof(t)); // 🔥 CRÍTICO: Limpiar toda la estructura
    
    t.length = sizeof(SPIFrame) * 8; // Longitud en BITS
    t.tx_buffer = &master_tx_buffer;
    t.rx_buffer = &master_rx_buffer;

    // ✅ TRANSMISIÓN POLLING (rápida y segura para RTOS)
    esp_err_t ret = spi_device_polling_transmit(spi, &t);
    
    if (ret == ESP_OK) {
        if (master_rx_buffer.isValid()) {
            // ✅ FULL-DUPLEX: Respuesta ya está en master_rx_buffer
            bool success = (master_rx_buffer.status & 0x80) == 0;
            
            // Debug opcional
            static uint32_t cmdCount = 0;
            if (++cmdCount % 50 == 0) {
                Serial.printf("📡 SPI Tx#%d: Cmd=0x%02X -> Status=0x%02X\n", 
                             cmdCount, command, master_rx_buffer.status);
            }
            
            return success;
        } else {
            Serial.printf("❌ Checksum error en respuesta SPI: Cmd=0x%02X\n", command);
        }
    } else {
        Serial.printf("❌ Error transmisión SPI: 0x%x, Cmd=0x%02X\n", ret, command);
    }
    
    return false;
}

bool SPIMaster::requestSensorData(uint16_t &distance, uint8_t &status) {
    // ✅ FULL-DUPLEX: En una sola transacción obtenemos los datos
    if (sendCommand(SPI_CMD_READ_SENSORS)) {
        distance = (master_rx_buffer.sensor_high << 8) | master_rx_buffer.sensor_low;
        status = master_rx_buffer.status;
        
        // Debug opcional
        static uint32_t sensorReads = 0;
        if (++sensorReads % 100 == 0) {
            Serial.printf("📊 SPI Sensor#%d: %dmm, Status=0x%02X\n", 
                         sensorReads, distance, status);
        }
        
        return true;
    }
    return false;
}

// ✅ TASK SPI MASTER OPTIMIZADA
void spiMasterTask(void *pvParameters) {
    Serial.println("📡 SPI Master Task iniciada");
    
    spiMaster.begin();
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    
    SensorData sensorData = {0};
    Command command;
    
    // Configurar watchdog
    esp_task_wdt_add(NULL);
    
    for(;;) {
        esp_task_wdt_reset();
        
        // 1. Procesar comandos de la cola
        while(xQueueReceive(xCommandQueue, &command, 0) == pdTRUE) {
            // Mapear comandos I2C a SPI (mantener compatibilidad)
            uint8_t spiCommand;
            switch(command.cmd) {
                case 0x01: spiCommand = SPI_CMD_MOTOR_FORWARD; break;
                case 0x03: spiCommand = SPI_CMD_MOTOR_LEFT; break;
                case 0x04: spiCommand = SPI_CMD_MOTOR_RIGHT; break;
                case 0x05: spiCommand = SPI_CMD_MOTOR_STOP; break;
                case 0x10: spiCommand = SPI_CMD_SET_MANUAL_SPEED; break;
                case 0x00: spiCommand = SPI_CMD_EMERGENCY_STOP; break;
                default:   spiCommand = command.cmd; break;
            }
            
            bool success = spiMaster.sendCommand(spiCommand, command.data);
            
            if(!success && command.cmd != SPI_CMD_READ_SENSORS) {
                Serial.printf("💥 Fallo SPI en comando: 0x%02X\n", command.cmd);
            }
        }
        
        // 2. Leer sensores periódicamente (cada 100ms)
        static uint8_t cycleCount = 0;
        if(++cycleCount >= 10) { // 10 ciclos * 10ms = 100ms
            uint16_t distance;
            uint8_t status;
            
            if(spiMaster.requestSensorData(distance, status)) {
                sensorData.sonarDistance = distance;
                sensorData.emergency = (status & 0x01);
                
                xQueueOverwrite(xSensorQueue, &sensorData);
            }
            cycleCount = 0;
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}*/