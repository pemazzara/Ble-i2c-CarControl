#ifndef BLUETOOTH_LE_CONNECT_H
#define BLUETOOTH_LE_CONNECT_H

#include "NimBLEDevice.h"
#include <Arduino.h>


class BluetoothLeConnect {
public:
  BluetoothLeConnect();
  void begin(const char* deviceName);
  void update();
  void sendData(String data);
  bool isConnected();
  String getLastCommand();

private:
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
  String lastCommand;
};

#endif