#ifndef SPI_SLAVE_H
#define SPI_SLAVE_H

#include <Arduino.h>
#include <driver/spi_slave.h> // ⚠️ Driver nativo de ESP-IDF
#include "SPIDefinitions.h"
#include "MotorControl.h"
#include "SpeedController.h"
#include "UltraSonicMeasure.h"
#include "freertos/semphr.h"


// Configuración para ESP32-S3 Slave
#define SPI_SLAVE_CLK  7 // -> 39 Master   
#define SPI_SLAVE_MISO 6 // -> 40 Master    
#define SPI_SLAVE_MOSI 5 // -> 41 Master  
#define SPI_SLAVE_SS   4 // -> 42 Master


class SPISlave {
private:
    MotorControl* motor_controller;
    UltraSonicMeasure* sensor_manager;
    SpeedController* pSpeedCtrl;
    // Handle para el driver
    spi_host_device_t spi_host;
    spi_slave_transaction_t transaction; // Transacción persistente
    bool initialized = false;

    // Buffer de recepción (alineado para DMA)
    //WORD_ALIGNED_ATTR static SPIFrame_t spi_rx_buffer;
    //WORD_ALIGNED_ATTR static SPIFrame_t spi_tx_buffer;
    static SPIResponseFrame_t* spi_tx_buffer; 
    static SPIFrame_t* spi_rx_buffer;
        // Sincronización entre tasks
    static SemaphoreHandle_t response_mutex;
    static SemaphoreHandle_t cmd_mutex;
    static SemaphoreHandle_t cmd_ready_sem;
    static SemaphoreHandle_t buffer_mutex;

    
     // Variables de estado
    static ControlCommand_t last_command;
    
    //static SpeedController* speedCtrl;
    static SPIResponseFrame_t last_response;
    // Pone los datos en el buffer DMA y le dice al hardware "Listos para recibir"
    void queueNextTransaction();
    void processReceivedCommand(const SPIFrame_t& rxFrame);
    void prepareResponse(SPIResponseFrame_t &txFrame);
    uint8_t getNextMsgId();

    //spi_slave_transaction_t transactions[3]; // Una por cada slot de la cola
    //int transaction_index = 0;
    // Buffers para DMA (deben ser accesibles por el hardware)
    // Se definen en el .cpp para asegurar alineación, aquí usamos punteros
    //SPIFrame_t *rxBuffer;
    //SPIResponseFrame_t *txBuffer;
    

public:
    SPISlave(MotorControl &motor, UltraSonicMeasure &sensor); // El constructor sigue recibiendo refs
    bool init();
    void processSPICommunication();
    void processCommand(const SPIFrame_t&);
    void prepareResponse(const SPIFrame_t& rxFrame);
    // Métodos estáticos para acceso desde tasks
    static ControlCommand_t getLastCommand();
    static void commandProcessed();
    static SPIFrame_t getReceivedFrame();
    static bool isCommandReady();
    static void signalDataProcessed();

    
    //static void setMotorController(MotorControl* mc) { motor_controller = mc; }   
    //static void setSensorManager(SensorManager* sm) { sensor_manager = sm; }
    bool getLatestSensorData(SonarSensorData_t &outData);

    // Para debugging
    static SPIResponseFrame_t getLastResponse() { return last_response; }

    void checkHealth();
    void printStats();
    void validateBuffers();

};

// Task de SPI Slave
//void spiSlaveTask(void *pvParameters);

#endif
