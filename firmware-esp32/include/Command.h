#ifndef COMMAND_H
#define COMMAND_H

#define I2C_REQUEST_SONAR  0x20  // ESP32 solicita lectura sonar
#define I2C_AUTO_COMMAND   0x21  // ESP32 envía comando autónomo
// DEFINICIÓN ÚNICA de todos los comandos
enum Comandos {
  // PRIORIDAD 1: Comandos de seguridad/emergencia
  CMD_EMERGENCY_STOP = 0,
  // PRIORIDAD 2: Comandos manuales (desde BLE)
  CMD_MANUAL_FORWARD = 1,
  CMD_MANUAL_BACKWARD = 2,
  CMD_MANUAL_LEFT = 3,
  CMD_MANUAL_RIGHT = 4,
  CMD_STOP = 5,
  // PRIORIDAD 3: Comandos autónomos (desde ESP32)
  CMD_AUTO_FORWARD = 6,
  CMD_AUTO_BACKWARD = 7,
  CMD_AUTO_LEFT = 8,
  CMD_AUTO_RIGHT = 9,
  CMD_AUTO_NAVIGATE = 10,
  // PRIORIDAD 4: Configuración
  CMD_SET_MANUAL_SPEED = 11,
  CMD_SET_AUTO_SPEED = 12,
  CMD_SET_SONAR_STATE = 13,  // Habilitar/deshabilitar sonar
  CMD_GET_SONAR_DISTANCE = 14, // Solicitar lectura del sonar

  // Comandos de modos especiales (si los necesitas)
  CMD_SET_WALL_FOLLOW_PARAMS = 15,
  CMD_OBSTACLE_AVOIDANCE = 16,
  CMD_WALL_FOLLOWING = 17,
  CMD_SET_AVOIDANCE_PARAMS = 18
};

#endif