#include "NimBLEDevice.h"
#include "BluetoothLeConnect.h"
#include "config.h"

BluetoothLeConnect::BluetoothLeConnect() {
  deviceConnected = false;
  oldDeviceConnected = false;
  lastCommand = "";
  pServer = nullptr;
  pService = nullptr;
  pDataCharacteristic = nullptr;
  pCommandCharacteristic = nullptr;
}

void BluetoothLeConnect::begin(const char* deviceName) {
  // Inicializar BLE con NimBLE
  NimBLEDevice::init(deviceName);
  
  // Configurar el servidor BLE
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks(this));

  // Crear el servicio BLE
  pService = pServer->createService(SERVICE_UUID);

  // Característica para enviar datos (notificaciones)
  pDataCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );
  pDataCharacteristic->addDescriptor(new NimBLE2904());

  // Característica para recibir comandos
  pCommandCharacteristic = pService->createCharacteristic(
    COMMAND_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks(this));

  // Iniciar el servicio
  pService->start();

  // Configurar y iniciar advertising
  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  //pAdvertising->setName("NimBLE"); // advertise the device name
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("BLE iniciado con NimBLE. Esperando conexión...");
}

void BluetoothLeConnect::update() {
  // Manejar reconexión
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // Dar tiempo para que la conexión se estabilice
    NimBLEDevice::startAdvertising();
    Serial.println("Dispositivo desconectado. Publicitando nuevamente...");
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}

void BluetoothLeConnect::sendData(String data) {
  if (deviceConnected) {
    pDataCharacteristic->setValue(data.c_str());
    pDataCharacteristic->notify();
  }
}

bool BluetoothLeConnect::isConnected() {
  return deviceConnected;
}

String BluetoothLeConnect::getLastCommand() {
  String command = lastCommand;
  lastCommand = ""; // Limpiar comando después de leer
  return command;
}

// Implementaciones de los callbacks
void BluetoothLeConnect::MyServerCallbacks::onConnect(NimBLEServer* pServer) {
  bleConnect->deviceConnected = true;
  bleConnect->oldDeviceConnected = true;
  Serial.println("Dispositivo conectado!");
}

void BluetoothLeConnect::MyServerCallbacks::onDisconnect(NimBLEServer* pServer) {
  bleConnect->deviceConnected = false;
  Serial.println("Dispositivo desconectado");
}

void BluetoothLeConnect::CommandCallbacks::onWrite(NimBLECharacteristic* pCharacteristic) {
  std::string value = pCharacteristic->getValue();
  if (value.length() > 0) {
    bleConnect->lastCommand = String(value.c_str());
    Serial.println("Comando recibido: " + bleConnect->lastCommand);
  }
}