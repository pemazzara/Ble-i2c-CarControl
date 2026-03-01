/*#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include <driver/spi_master.h>  // ✅ Cambiar por API ESP32
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "SPIDefinitions.h"

class SPIMaster {
private:
    spi_device_handle_t spi;
    bool initialized = false;
    
public:
    void begin();
    bool sendCommand(uint8_t command, uint8_t data1 = 0, uint8_t data2 = 0, uint8_t data3 = 0);
    bool requestSensorData(uint16_t &distance, uint8_t &status);
};

void spiMasterTask(void *pvParameters);
extern SPIMaster spiMaster;

#endif
*/
#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include <driver/spi_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "SPIDefinitions.h"

class SPIMaster {
private:
    spi_device_handle_t spi;
    bool initialized = false;
    
public:
    void begin();
    bool sendCommand(uint8_t command, uint8_t data1 = 0, uint8_t data2 = 0, uint8_t data3 = 0);
    bool requestSensorData(uint16_t &distance, uint8_t &status);
};

// ✅ MODIFICAR: Recibir colas como parámetros
void spiMasterTask(void *pvParameters);
extern SPIMaster spiMaster;

#endif