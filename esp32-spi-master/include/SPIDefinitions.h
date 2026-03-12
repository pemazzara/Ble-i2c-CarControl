// SPIDefinitions.h - Versión simplificada
#ifndef SPI_DEFINITIONS_H
#define SPI_DEFINITIONS_H

#include <stdint.h>
#include <Arduino.h>


// Pines HSPI ESP32-S3
// Configuración óptima para ESP32-S3
#define SPI_MASTER_CLK  39   
#define SPI_MASTER_MISO 40     
#define SPI_MASTER_MOSI 41   
#define SPI_MASTER_SS   42

// --- CONSTANTES ---
#define SPI_MAGIC_MASTER 0xA5
#define SPI_MAGIC_SLAVE  0x5A

// --- ENUMS ---
typedef enum : uint8_t {
    CMD_STOP = 0,
    CMD_DRIVE,
    CMD_HEARTBEAT,
    CMD_READ_SENSORS,
    CMD_MOVE_DISTANCE,
    CMD_SET_MODE,
    CMD_STATUS,
    CMD_CALIBRATE
} ControlCommandType;

typedef enum : uint8_t {
    MODE_MANUAL = 2,
    MODE_AUTO = 3,   
} ControlMode;

// Estructura de bits para el byte de confianza
struct ConfidenceByte {
    bool sonar_ok     : 1; // Bit 0: Sonar activo y en rango
    bool tof_f_ok     : 1; // Bit 1: TOF Frontal activo
    bool discrepancy  : 1; // Bit 2: Sonar y TOF no coinciden
    bool source       : 2; // Bits 3-4: 0=Ninguno, 1=Sonar, 2=TOF, 3=Fused
    uint8_t reserved  : 3; 
};
// Master -> Slave (14 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type;          // ControlCommandType
    int16_t speed;         
    int16_t angle;         
    int16_t distance;      
    uint32_t timestamp;
    uint8_t priority;
    uint8_t status;        
    uint8_t targetMode;    
} ControlCommand_t;

// Slave -> Master (14 bytes - Igualamos tamaño al comando)
typedef struct __attribute__((packed)) {
    uint8_t status;         // Estado del sistema (bits de error, etc)
    uint16_t distance;      // Distancia del sonar (mm)
    int16_t left_speed;     // Telemetría motores
    int16_t right_speed;    
    uint8_t reserved[7];    // Padding para llegar a 14 bytes y ser simétrico
} SPIResponsePayload_t;

// --- FRAMES (La envoltura que viaja por el cable) ---

// Estructura Master -> Slave (Total: 1+14+2 = 17 bytes)
typedef struct __attribute__((packed)) {
    uint8_t magic_word;           // 0xA5
    ControlCommand_t payload;     // 14 bytes
    uint16_t checksum;            // 2 bytes
} SPIFrame_t;

// Estructura Slave -> Master (Total: 1+14+2 = 17 bytes)
typedef struct __attribute__((packed)) {
    uint8_t magic_word;           // 0x5A
    SPIResponsePayload_t payload; // 14 bytes
    uint16_t checksum;            // 2 bytes
} SPIResponseFrame_t;

// --- ESTRUCTURAS DE APLICACIÓN (Internas del Master/Slave) ---

typedef struct {
    uint16_t sonarDistance;
    uint16_t tofLeft;
    uint16_t tofFront;
    uint16_t tofRight;   
    uint32_t lastSonarUpdate;
    uint32_t lastTofUpdate;   
    bool emergency;
    uint8_t sensorStatus;       // Bit 0: Sonar OK, Bit 1: TOFs OK
    uint8_t slaveInternalStatus; // El 'status' que viene del payload del slave
} SensorData_t;

typedef struct {
    uint16_t distance;
    uint16_t raw_distance;
    bool emergency;
    bool sensor_ok;
    uint32_t timestamp;
    uint8_t error_count;
} SonarSensorData_t;

#ifdef __cplusplus
extern "C" {
#endif

uint16_t calcularChecksum(const void* data, size_t len);

#ifdef __cplusplus
}
#endif

#endif