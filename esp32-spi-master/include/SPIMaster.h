#ifndef SPIMASTER_H
#define SPIMASTER_H

#include <Arduino.h>
#include <driver/spi_master.h>
#include <stdint.h>
#include "SPIDefinitions.h"
#include "system_state.h"
#include "esp_task_wdt.h"

// Pines HSPI ESP32-S3
// Configuración óptima para ESP32-S3
#define SPI_MASTER_CLK  39   
#define SPI_MASTER_MISO 40     
#define SPI_MASTER_MOSI 41   
#define SPI_MASTER_SS   42

extern uint16_t calcularChecksum(const void* data, size_t len);
#define DISTANCIA_CRITICA_STOP 150 // 15cm o 150mm - Detener inmediatamente

class SPIMaster {
public:
    SPIMaster();
    
    bool begin();
    bool sendCommandWithTimeout(const ControlCommand_t* cmd, uint32_t timeout_ms);
    bool sendCommand(const ControlCommand_t *cmd);
    int errorCounter = 0;
    uint16_t currentLeftSpeed = 200;
    uint16_t currentRightSpeed = 200;
    // Getters para estado del Slave
    SPIResponseFrame_t getLastResponse();
    bool isSlaveInEmergency();
    bool isSlaveMoving();
    bool getCleanSensorData(uint16_t &dist, uint8_t &st);
    void evaluarEmergenciaInmediata(const SensorData_t &sd);
    bool requestSensorData();
    // Utilidades
    void testCommunication();
    bool testSPI();
    bool performSpiExchange(const ControlCommand_t &cmd);
private:
    ControlCommand_t last_drive_payload; // Para recordar qué enviamos
    bool initialized;
    spi_device_handle_t spi;
    spi_transaction_t t;
    // Buffers para comunicación
    // En el .h de SPIMaster
    //SPIFrame_t master_tx_buffer __attribute__((aligned(4)));
    //SPIResponseFrame_t master_rx_buffer __attribute__((aligned(4)));

    bool processResponse(const SPIResponseFrame_t& response);
    void handleCommunicationErrors(uint8_t magic);
    bool reconnect();
    uint8_t getNextMsgId();
    bool startCalibration();
};

#endif // SPIMASTER_H