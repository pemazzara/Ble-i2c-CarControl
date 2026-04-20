#ifndef BLUETOOTH_LE_CONNECT_H
#define BLUETOOTH_LE_CONNECT_H

#include "NimBLEDevice.h"
#include <Arduino.h>
#include "SPIDefinitions.h"

#pragma pack(push, 1)
typedef struct {
    uint8_t msg_id;      // Offset 0
    uint8_t baseState;   // Offset 1
    uint8_t subState;    // Offset 2
    uint8_t alerts;      // Offset 3
    uint8_t progress;    // Offset 4
    uint16_t distance;   // Offset 5 (¡Sin padding gracias al pack!)
    uint16_t K_fixed;    // Offset 7
    uint16_t tau_fixed;  // Offset 9
    uint8_t needs_ack;   // Offset 11
} MasterStatusPacket_t;
#pragma pack(pop)
/*
#pragma pack(1)  // Crucial para que Android no lea basura por alineación de bytes
typedef struct {
    uint8_t msg_id;      // 0x81
    uint8_t baseState;   // SystemBaseState_t (BOOT, READY, EMERGENCY, ERROR...)
    uint8_t subState;    // ReadySubState_t (IDLE, MANUAL, AUTO)
    uint8_t alerts;      // Bitmask: [0: Obstáculo, 1: Comm Loss, 2: LowBat]
    uint8_t progress;    // 0-100% (para Calibración)
    uint16_t distance;   // Datos del Sonar (del Slave)
    
    // Parámetros de control (punto fijo)
    uint16_t K_fixed;    
    uint16_t tau_fixed;  
    
    uint8_t needs_ack;   // 1 si requiere confirmación del usuario para recuperar
} MasterStatusPacket_t;
 */
typedef struct __attribute__((packed)) {
    uint8_t type;          // ControlCommandType
    int16_t speed;         
    int16_t angle;         
    int16_t distance;      
    uint32_t timestamp;
    uint8_t priority;
    uint8_t status;        
    uint8_t targetMode;    
} BLECommand_t;

class BluetoothLeConnect {
public:
  BluetoothLeConnect();
  void begin(const char* deviceName);
  void update();
  void sendData(String data);
  bool isConnected();
  BLECommand_t getLastCommand();
  void sendBLEPacket(MasterStatusPacket_t* packet);
  bool hasNewCommand;

private:
    BLECommand_t lastCommand;  // Cambiar de String a BLECommand_t
     
  class DataCallbacks: public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        Serial.printf("🔔 ¡ALGUIEN SE SUSCRIBIÓ! Valor: %d\n", subValue);
    }
  };
  class MyServerCallbacks : public NimBLEServerCallbacks {
  public:
    MyServerCallbacks(BluetoothLeConnect* connect) : bleConnect(connect) {}
    void onConnect(NimBLEServer* pServer) override;
    void onDisconnect(NimBLEServer* pServer) override;
  
  private:
    BluetoothLeConnect* bleConnect;
  };

  class CommandCallbacks : public NimBLECharacteristicCallbacks {
  public:
    CommandCallbacks(BluetoothLeConnect* connect) : bleConnect(connect) {}
    void onWrite(NimBLECharacteristic* pCharacteristic) override;
  private:
    BluetoothLeConnect* bleConnect;
  };

  NimBLEServer* pServer;
  NimBLEService* pService;
  NimBLECharacteristic* pDataCharacteristic;
  NimBLECharacteristic* pCommandCharacteristic;
  bool deviceConnected;
  bool oldDeviceConnected;
  
};

#endif