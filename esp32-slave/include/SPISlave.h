#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <Arduino.h>
#include <driver/spi_slave.h> // ⚠️ Driver nativo de ESP-IDF
#include "SPIDefinitions.h"
#include "MotorControl.h"
#include "SonarIntegration.h"

class SPISlave {
private:
    MotorControl &motorController;
    SonarIntegration &sonar;
    bool initialized = false;

    // Buffers para DMA (deben ser accesibles por el hardware)
    // Se definen en el .cpp para asegurar alineación, aquí usamos punteros
    SPIFrame *txBuffer;
    SPIFrame *rxBuffer;
    
    // Handle para el driver
    spi_host_device_t spi_host;

public:
    SPISlave(MotorControl &motor, SonarIntegration &sonar);
    
    // Configura el hardware SPI + DMA
    void begin();
    
    // Revisa si el hardware terminó una transacción y prepara la siguiente
    void processSPICommunication();
    
private:
    // Pone los datos en el buffer DMA y le dice al hardware "Listos para recibir"
    void queueNextTransaction();
    
    // Lógica de negocio (tu código original adaptado)
    void processCommand(const SPIFrame &rxFrame);
    void prepareResponse(SPIFrame &txFrame);
};

// Task de SPI Slave
void spiSlaveTask(void *pvParameters);

#endif