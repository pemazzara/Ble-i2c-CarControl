#ifndef COMMAND_H
#define COMMAND_H

// DEFINICIÓN ÚNICA de todos los comandos
enum Command {
  // PRIORIDAD 1: Comandos de seguridad/emergencia
  CMD_EMERGENCY_STOP = 0,
  
  // PRIORIDAD 2: Comandos manuales (desde BLE)
  CMD_MANUAL_FORWARD = 1,
  CMD_MANUAL_BACKWARD = 2,
  CMD_MANUAL_LEFT = 3,
  CMD_MANUAL_RIGHT = 4,
  
  // PRIORIDAD 3: Comandos autónomos (desde ESP32)
  CMD_AUTO_FORWARD = 5,
  CMD_AUTO_BACKWARD = 6,
  CMD_AUTO_LEFT = 7,
  CMD_AUTO_RIGHT = 8,
  CMD_AUTO_NAVIGATE = 9,
  
  // Comandos de modos especiales (si los necesitas)
  CMD_OBSTACLE_AVOIDANCE = 10,
  CMD_WALL_FOLLOWING = 11,
  
  // PRIORIDAD 4: Configuración
  CMD_SET_MANUAL_SPEED = 12,
  CMD_SET_AUTO_SPEED = 13,
  CMD_SET_AVOIDANCE_PARAMS = 14,
  CMD_SET_WALL_FOLLOW_PARAMS = 15,
  
  // Comando STOP básico
  CMD_STOP = 16
};

#endif