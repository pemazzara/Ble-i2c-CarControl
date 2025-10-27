// Características BLE
BLECharacteristic* cmdCharacteristic;   // Comandos: 'F','B','L','R','S','E'
BLECharacteristic* modeCharacteristic;  // Modos: 'M' manual, 'A' auto, '1','2','3' velocidades
BLECharacteristic* sensorCharacteristic; // Datos sensores: "F:125,L:80,R:110"