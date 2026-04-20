#include "NimBLEDevice.h"
#include "BluetoothLeConnect.h"
#include "config.h"


BluetoothLeConnect::BluetoothLeConnect() {
  deviceConnected = false;
  oldDeviceConnected = false;
  memset(&lastCommand, 0, sizeof(BLECommand_t));  // Inicializar a cero
  pServer = nullptr;
  pService = nullptr;
  pDataCharacteristic = nullptr;
  pCommandCharacteristic = nullptr;
  hasNewCommand = false;
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
  pDataCharacteristic->setCallbacks(new DataCallbacks());
  //pDataCharacteristic->addDescriptor(new NimBLE2904());

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
/*
ControlCommand_t BluetoothLeConnect::getLastCommand() {
    hasNewCommand = false; // Al leerlo, marcamos que ya no es nuevo
    return lastCommand;
}
*/
BLECommand_t BluetoothLeConnect::getLastCommand() {
  BLECommand_t cmd;
  noInterrupts();
  memcpy(&cmd, &lastCommand, sizeof(BLECommand_t));
  memset(&lastCommand, 0, sizeof(BLECommand_t));  // Limpiar después de leer
  interrupts();
  hasNewCommand = false; // Al leerlo, marcamos que ya no es nuevo
  return cmd;
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

void BluetoothLeConnect::sendBLEPacket(MasterStatusPacket_t* packet) {
    if (deviceConnected) {
        bool hasSubscribers = pDataCharacteristic->getSubscribedCount() > 0;
        Serial.printf("Enviando... Suscriptores: %d | Distancia: %d\n", hasSubscribers, packet->distance);
        
        pDataCharacteristic->setValue((uint8_t*)packet, sizeof(MasterStatusPacket_t));
        pDataCharacteristic->notify(); 
    }
}

void BluetoothLeConnect::CommandCallbacks::onWrite(NimBLECharacteristic* pCharacteristic) {
  NimBLEAttValue value = pCharacteristic->getValue();
  
  bleConnect->hasNewCommand = true;
    // Recibimos un comando estructurado
    if (value.length() == sizeof(BLECommand_t)) {
      // Copiar datos al comando  
      memcpy(&bleConnect->lastCommand, value.data(), sizeof(BLECommand_t));
    } else {
      Serial.printf("⚠️ Tamaño de comando inválido: recibido %d, esperado %d\n", 
                  value.length(), sizeof(BLECommand_t));
    } 
    // Debug: mostrar comando recibido
    Serial.printf("📥 BLE Rx - Comando estructurado recibido:\n");
    Serial.printf("  Type: 0x%02X\n", bleConnect->lastCommand.type);
    Serial.printf("  Speed: %d\n", bleConnect->lastCommand.speed);
    Serial.printf("  Angle: %d°\n", bleConnect->lastCommand.angle);
    Serial.printf("  TargetMode: %s\n", 
                  bleConnect->lastCommand.targetMode == AUTOMATIC ? "AUTO" : "MANUAL");
    Serial.printf("  Priority: %d\n", bleConnect->lastCommand.priority);
    
}