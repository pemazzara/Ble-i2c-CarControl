#ifndef CONFIG_H
#define CONFIG_H
// Configuración BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define COMMAND_UUID        "1c95d5e3-d8f7-413a-bf3d-7a2e5d7be87e"

#define BLE_DEVICE_NAME "CarRobot-ESP32-S3"

// Comandos BLE
#define CMD_FORWARD    'F'
#define CMD_BACKWARD   'B'
#define CMD_LEFT       'L'
#define CMD_RIGHT      'R'
#define CMD_STOP        'S'
#define CMD_SPEED_1    '1'
#define CMD_SPEED_2    '2'
#define CMD_SPEED_3    '3'
#define CMD_AUTO_MODE  'A'
#define CMD_MANUAL_MODE 'M'
#define CMD_ESCAPE_MODE 'E'



#endif