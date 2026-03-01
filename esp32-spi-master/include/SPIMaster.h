#ifndef SPIMASTER_H
#define SPIMASTER_H

#include <Arduino.h>
#include <driver/spi_master.h>
#include <stdint.h>
#include "SPIDefinitions.h"
#include "esp_task_wdt.h"

extern bool autonomousMode; // "Aviso" al compilador de que la variable vive en otro lugar
extern uint16_t calcularChecksum(const void* data, size_t len);
#define DISTANCIA_CRITICA_STOP 150 // 15cm o 150mm - Detener inmediatamente
class SPIMaster {
public:
    SPIMaster();
    
    bool begin();
    bool sendCommandWithTimeout(const ControlCommand_t* cmd, uint32_t timeout_ms);
    bool sendCommand(const ControlCommand_t *cmd);
    int errorCounter = 0;
    // Getters para estado del Slave
    SPIResponsePayload_t getLastResponse();
    bool isSlaveInEmergency();
    bool isSlaveMoving();
    bool getCleanSensorData(uint16_t &dist, uint8_t &st);
    void evaluarEmergenciaInmediata(const SensorData_t &sd);
    bool requestSensorData();
    // Utilidades
    void testCommunication();
    bool testSPI();
    
private:
    ControlCommand_t last_drive_payload; // Para recordar qué enviamos
    bool initialized;
    spi_device_handle_t spi;
    spi_transaction_t t;
    // Buffers para comunicación
    SPIFrame_t master_tx_buffer;
    SPIResponseFrame_t master_rx_buffer;
    void handleCommunicationErrors(uint8_t magic);
    bool reconnect();
    
};

#endif // SPIMASTER_H