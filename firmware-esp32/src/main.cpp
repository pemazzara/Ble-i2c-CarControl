#include <Arduino.h>
#include "ble/BluetoothLeConnect.h"
#include "MotorControl.h"
#include "SensorControl.h"
//#include "config.h"
#define BLE_DEVICE_NAME "CarRobot-ESP32-S3"
// Instancias
BluetoothLeConnect ble;
MotorControl motor;
SensorControl sensors;
bool autonomousMode = false;
unsigned long lastCommandTime = 0;
int speed = 150;


//ServoControl servo; // Ya no se necesita, el servo se controla desde Arduino
void processCommand(char cmd);
void autonomousNavigation();
const unsigned long SAFETY_TIMEOUT = 1500; // 1.5 segundos

void setup() {
  Serial.begin(115200);
  
  // Inicializar componentes
  motor.begin();
  //sensors.begin();
  
  // Inicializar BLE
  ble.begin(BLE_DEVICE_NAME);
  
  Serial.println("Sistema CarRobot con control I2C inicializado");
}

void loop() {
  // Actualizar BLE
  ble.update();
  
  // Leer sensores
  //sensors.readAll();
  
  // Procesar comandos BLE
  String command = ble.getLastCommand();
  if (command.length() > 0) {
    processCommand(command.charAt(0));
    // Si recibimos un comando, resetear la navegación autónoma
    autonomousMode = false;
    lastCommandTime = millis();
  }
    // Activar navegación autónoma solo si no hay comandos recientes
  if (millis() - lastCommandTime > SAFETY_TIMEOUT) { // 1.5 segundos sin comandos
    autonomousMode = true;
  } 
  
  // Ejecutar navegación autónoma solo si está activada
  if (autonomousMode) {
      motor.stopMotor();
      Serial.println("⏰ SAFETY TIMEOUT - Motores detenidos");
    //autonomousNavigation();
  }
  
  // Enviar datos de sensores via BLE
  if (ble.isConnected()) {
    String sensorData = "F:" + String(sensors.frontDistance) +
                       " L:" + String(sensors.leftDistance) +
                       " R:" + String(sensors.rightDistance);
    ble.sendData(sensorData);
  }
  
  delay(50);
}
void processCommand(char command) {
  autonomousMode = false;
  
  Serial.print("=== COMANDO RECIBIDO: '");
  Serial.print(command);
  Serial.println("' ===");
  
  switch(command) {
    case 'F':
      Serial.println("EJECUTANDO: Adelante");
      motor.moveForward(speed);
      break;
    case 'B':
      Serial.println("EJECUTANDO: Atrás");
      motor.moveBackward(speed);
      break;
    case 'L':
      Serial.println("EJECUTANDO: Izquierda");
      motor.softTurnLeft();
      break;
    case 'R':
      Serial.println("EJECUTANDO: Derecha");
      motor.softTurnRight();
      break;
    case 'S':
      Serial.println("EJECUTANDO: Detener");
      motor.stopMotor();
      break;
    case '1':
      speed = 100;
      Serial.println("VELOCIDAD: 100");
      break;
    case '2':
      speed = 180;
      Serial.println("VELOCIDAD: 180");
      break;
    case '3':
      speed = 255;
      Serial.println("VELOCIDAD: 255");
      break;
    case 'A':
      autonomousMode = true;
      Serial.println("MODO: Autónomo");
      break;
    default:
      Serial.print("ERROR: Comando no reconocido: ");
      Serial.println(command);
      break;
  }
  
  Serial.println("=========================");
}
