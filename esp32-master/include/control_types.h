// control_types.h
#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

#include <stdint.h>

// Comandos básicos (1 byte cada uno)
typedef enum : uint8_t {
    CMD_STOP = 0,
    CMD_MOVE_FORWARD = 1,
    CMD_MOVE_BACKWARD = 2,
    CMD_TURN_LEFT = 3,
    CMD_TURN_RIGHT = 4,
    CMD_EMERGENCY_STOP = 8,
    CMD_SET_SPEED = 9,
    CMD_READ_SENSORS = 13,
    CMD_SET_MODE = 14,        // Manual/Autónomo/WallFollow
    CMD_CALIBRATE = 15,       // Calibración
    CMD_GET_STATUS = 16       // Solicitar estado
} SPIControlCommand;

/* Enumerado claro (mejor que chars)
typedef enum {
    CMD_STOP = 0,
    CMD_MOVE_FORWARD,
    CMD_MOVE_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_ROTATE_LEFT,     // Rotación en sitio
    CMD_ROTATE_RIGHT,
    CMD_SET_SPEED,       // Cambiar velocidad base
    CMD_SET_MODE,        // Manual/Autónomo/WallFollow
    CMD_EMERGENCY_STOP,  // Parada de emergencia
    CMD_CALIBRATE,       // Calibración
    CMD_GET_STATUS       // Solicitar estado
} ControlCommandType;
*/
// Modos de operación
typedef enum {
    MODE_MANUAL = 0,
    MODE_AUTONOMOUS,
    MODE_WALL_FOLLOW,
    MODE_LINE_FOLLOW,
    MODE_CALIBRATION
} ControlMode;

// Estructura de comando
typedef struct {
    SPIControlCommand type;
    int16_t param1;      // velocidad, ángulo, etc.
    int16_t param2;      // parámetro adicional
    uint32_t timestamp;
    uint8_t priority;    // 0=min, 10=max
    ControlMode targetMode; // Para CMD_SET_MODE
} ControlCommand_t;

#endif