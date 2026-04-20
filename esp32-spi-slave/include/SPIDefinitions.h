// SPIDefinitions.h - Versión simplificada
#ifndef SPI_DEFINITIONS_H
#define SPI_DEFINITIONS_H

#include <stdint.h>
#include <Arduino.h>

// --- CONSTANTES ---
#define SPI_MAGIC_MASTER 0xA5
#define SPI_MAGIC_SLAVE  0x5A

// --- ENUMS ---
/*Android definitions
    CMD_STOP(0x00),     // Parada de emergencia
    CMD_DRIVE(0x01),              // Movimiento con velocidad+ángulo
    CMD_HEARTBEAT(0x02),
    CMD_READ_SENSORS(0x03),      // Leer sensores
    CMD_MOVE_DISTANCE(0x04),
    CMD_SET_MODE(0x05),          // Cambiar modo de operación
    CMD_STATUS(0x06),
    CMD_IDLE(0x07),
    CMD_READY(0x08),
    RESET_EMERGENCY(0x09),
    RECONNECT(0x0A),
    CMD_CALIBRATE(0x0B);
    */     
typedef enum : uint8_t {
    CMD_STOP = 0,
    CMD_DRIVE = 1,
    CMD_HEARTBEAT = 2,
    CMD_READ_SENSORS = 3,
    CMD_MOVE_DISTANCE = 4,
    CMD_SET_MODE = 5,
    CMD_STATUS = 6,
    CMD_IDLE = 7,
    CMD_READY = 8,
    RESET_EMERGENCY = 9,
    RECONNECT = 10,
    CMD_CALIBRATE = 11
} ControlCommandType;

typedef enum : uint8_t {
    JOYSTIC = 0,
    ROTATION = 1,
    MANUAL = 2,
    AUTOMATIC = 3,
    CALIBRATION = 4
} ControlMode;


typedef enum : uint8_t {
    TYPE_SENSORS = 0,
    TYPE_SYSTEM = 1,
} ResponseType;

// --- PAYLOADS (Los datos puros sin envoltura) ---

// Payload Master -> Slave (8 bytes)
typedef struct __attribute__((packed)) {
    uint8_t type;          // ControlCommandType
    int16_t speed;         
    int16_t angle;               
    uint16_t timestamp;
    uint8_t state;        // Padding para llegar a 10 bytes
} ControlCommand_t;

// Estructura Master -> Slave (Total: 2+10+2 = 14 bytes)
typedef struct __attribute__((packed)) {
    uint8_t msg_id;              // ID de mensaje para tracking
    uint8_t magic_word;           // 0xA5    
    ControlCommand_t payload;     // 8 bytes
    uint8_t padding[4]; // 👈 Esto completa los 16 bytes (1+1+8+4+2 = 16)
    uint16_t checksum;            // 2 bytes
} SPIFrame_t;


// Slave -> Master (16 bytes total)
#pragma pack(push, 1)
typedef struct {
    // --- CONTROL & ALINEACIÓN (4 bytes) ---
    uint8_t  msg_id;       // Offset 0
    uint8_t  magic_word;   // Offset 1 (0x5A)
    ResponseType  type;         // Offset 2 (0=Telemetría Motor, 1=Sistema)
    uint8_t  last_rx_id; // Offset 3 (ERRORES: Overcurrent, Temp, Stall)

    // --- PAYLOAD ALINEADO A 4-BYTES (Offset 4) ---
    union {
        struct {
            int16_t  rpm_left;   // Bytes 4-5 (Si tienes encoders)
            int16_t  rpm_right;  // Bytes 6-7
            uint16_t a_vel; // Bytes 8-9 (Velocidad de acercamiento, calculada por el Slave)
            uint16_t distance;    // Bytes 10-11 (Distancia del sonar)
            uint8_t  motor_flags; // Byte 12 (Bits de error: Overcurrent, Stall, etc)
            uint8_t reserved;   // Byte 13
        } motors;

        struct {
            uint16_t K_fixed;    // Bytes 4-5
            uint16_t tau_fixed;  // Bytes 6-7
            uint8_t  progress;   // Byte 8
            uint8_t  state;      // Byte 9
            uint8_t  calibration_valid; // Byte 10 (1=K/Tau válidos, 0=Inválidos)
            uint8_t pad[3];     // Bytes 11-13
        } system;
    } payload;

    // --- INTEGRIDAD (2 bytes) ---
    uint16_t checksum;     // Offset 14-15
} SPIResponseFrame_t; 
#pragma pack(pop)

typedef struct __attribute__((packed)) {
    uint8_t status;         // Estado del sistema (bits de error, etc)
    uint16_t distance;      // Distancia del sonar (mm)
    uint16_t a_vel;         // Telemetría motores
    int16_t left_speed;     // Telemetría motores
    int16_t right_speed;
    uint8_t slave_state;    // Estado interno del Slave (bits de error, modo, etc)
} SPIResponsePayload_t;



// --- ESTRUCTURAS DE APLICACIÓN (Internas del Master/Slave) ---

typedef struct {
    uint16_t sonarDistance;
    uint16_t a_vel;  // velocidad de acercamiento (adimen) positivo si se aleja, negativo si se acerca
    uint16_t tofLeft;
    uint16_t tofFront;  
    uint16_t tofRight;   
    uint32_t lastSonarUpdate;
    uint32_t lastTofUpdate;   
    bool emergency;
    uint8_t sensorStatus;       // Bit 0: Sonar OK, Bit 1: TOFs OK
    uint8_t slaveInternalStatus; // El 'status' que viene del payload del slave
    uint8_t lastSlaveFlags; // El 'flags' que viene del payload del slave
    uint8_t lastSlaveState; // El 'status' que viene del payload del slave
    uint16_t lastSyncTimestamp; // Última vez que se sincronizó con el Slave
    float calibrationK;                         // ganancia estática
    float calibrationTau;                       // constante de tiempo
    bool calibrationValid;
    uint8_t calibrationProgress; // 0-100% (si el Slave está en calibración)
    bool batteryLow; // (opcional, si el Slave envía este dato o lo inferimos de alguna forma)
} SensorData_t;

typedef struct {
    uint16_t distance;
    uint16_t a_vel; // Velocidad de acercamiento (mm/s)
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