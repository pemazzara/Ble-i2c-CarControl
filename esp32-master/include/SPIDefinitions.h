#ifndef SPI_DEFINITIONS_H
#define SPI_DEFINITIONS_H

#include <Arduino.h>
#include <driver/spi_slave.h>

#define SPI_CLK  39   
#define SPI_MISO 40     
#define SPI_MOSI 41   
#define SPI_SS   42

// ✅ COMANDOS SPI UNIFICADOS (prefijo SPI_ para evitar conflictos)
enum SPICommands {
    SPI_CMD_EMERGENCY_STOP    = 0x00,
    SPI_CMD_MOTOR_FORWARD     = 0x01,
    SPI_CMD_MOTOR_BACKWARD    = 0x02,
    SPI_CMD_MOTOR_LEFT        = 0x03,
    SPI_CMD_MOTOR_RIGHT       = 0x04,
    SPI_CMD_MOTOR_STOP        = 0x05,
    SPI_CMD_SET_MANUAL_SPEED  = 0x10,
    SPI_CMD_SET_AUTO_SPEED    = 0x11,
    SPI_CMD_READ_SENSORS      = 0x80
};

// Trama SPI optimizada (8 bytes)
struct __attribute__((packed)) SPIFrame {
    uint8_t command;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;  
    uint8_t status;
    uint8_t sensor_high;
    uint8_t sensor_low;
    uint8_t checksum;
    
    void calculateChecksum() {
        checksum = command ^ data1 ^ data2 ^ data3 ^ status ^ sensor_high ^ sensor_low;
    }
    
    bool isValid() const {
        uint8_t calc = command ^ data1 ^ data2 ^ data3 ^ status ^ sensor_high ^ sensor_low;
        return calc == checksum;
    }
};

#endif
//////////////////////////////////////////////////////////////////////
// File: master-esp32/src/main.cpp