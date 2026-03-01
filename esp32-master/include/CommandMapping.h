// CommandMapping.h - Mapeo entre Master y Slave
#ifndef COMMAND_MAPPING_H
#define COMMAND_MAPPING_H

#include "control_types.h"  // Comandos del Master
#include "CommonCommands.h" // Comandos comunes con Slave

// Mapeo de ControlCommandType (Master) a comandos Slave
static uint8_t mapMasterToSlaveCommand(ControlCommandType masterCmd, int16_t param1 = 0) {
    switch(masterCmd) {
        // MOVIMIENTO BÁSICO
        case CMD_MOVE_FORWARD:
            return CMD_MANUAL_FORWARD;  // 0x01
            
        case CMD_MOVE_BACKWARD:
            return CMD_MANUAL_BACKWARD; // 0x02
            
        case CMD_TURN_LEFT:
            return CMD_MANUAL_LEFT;     // 0x03
            
        case CMD_TURN_RIGHT:
            return CMD_MANUAL_RIGHT;    // 0x04
            
        case CMD_STOP:
            return CMD_STOP;            // 0x05
            
        // ROTACIONES (mapear a giros para compatibilidad)
        case CMD_ROTATE_LEFT:
            return CMD_MANUAL_LEFT;     // Usar mismo que TURN_LEFT
            
        case CMD_ROTATE_RIGHT:
            return CMD_MANUAL_RIGHT;    // Usar mismo que TURN_RIGHT
            
        // CONTROL
        case CMD_EMERGENCY_STOP:
            return CMD_EMERGENCY_STOP;  // 0x00
            
        case CMD_SET_SPEED:
            return CMD_SET_MANUAL_SPEED; // 0x0B
            
        // COMANDOS ESPECIALES (necesitan manejo especial)
        case CMD_SET_MODE:
            // No tiene equivalente directo en Slave
            // Se maneja internamente en Master
            return 0xFF;  // Comando especial
            
        case CMD_CALIBRATE:
            return 0x14;  // Si el Slave soporta calibración
            
        case CMD_GET_STATUS:
            return CMD_READ_SENSORS;    // 0x13 para leer sensores
            
        default:
            return 0xFF;  // Comando desconocido
    }
}

// Función para obtener data adicional según comando
static uint8_t getSlaveCommandData(ControlCommandType masterCmd, int16_t param1) {
    switch(masterCmd) {
        case CMD_MOVE_FORWARD:
        case CMD_MOVE_BACKWARD:
        case CMD_SET_SPEED:
            return constrain(param1, 0, 100);  // Porcentaje 0-100
            
        case CMD_TURN_LEFT:
        case CMD_TURN_RIGHT:
        case CMD_ROTATE_LEFT:
        case CMD_ROTATE_RIGHT:
            return constrain(param1, 0, 90);   // Ángulo 0-90°
            
        default:
            return 0;  // Sin datos adicionales
    }
}

#endif