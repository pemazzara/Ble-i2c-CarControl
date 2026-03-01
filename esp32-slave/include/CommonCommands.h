// CommonCommands.h - USAR EN AMBOS (Master y Slave)
// CommonCommands.h - PARA AMBOS
#ifndef COMMON_COMMANDS_H
#define COMMON_COMMANDS_H

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
    CMD_READ_SENSORS = 13
} SPIControlCommand;

#endif