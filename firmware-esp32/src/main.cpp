#include <Arduino.h>
#include "BluetoothLeConnect.h"
#include "MotorControl.h"
#include "SensorControl.h"
#include "SensorFusion.h"  // ✅ INCLUIR ESTE HEADER
#include "Command.h"

#define BLE_DEVICE_NAME "CarRobot-ESP32-S3"


// Instancias
BluetoothLeConnect ble;
MotorControl motor;
SensorControl sensors;
SensorFusion sensorFusion(sensors);  // ✅ CREAR INSTANCIA CORRECTAMENTE

bool autonomousMode = false;
unsigned long lastCommandTime = 0;
int speed = 150;

// Estados del sistema
enum SystemState {
  STATE_I, STATE_F, STATE_L, STATE_R, STATE_READY
};

SystemState currentState = STATE_I;
const unsigned long SAFETY_TIMEOUT = 1500;

// Prototipos de funciones
void autonomousNavigation();
void handleSystemState();
void processStateCommand(char command);
void processCommand(char command);

void handleSystemState() {
  switch(currentState) {
    case STATE_I:
      Serial.println("🔵 ESTADO I: Esperando 'F' para iniciar calibración");
      break;
    case STATE_F:
      sensors.readSensor(0);
      Serial.printf("🟡 ESTADO F: Frontal = %dmm - Esperando 'L'\n", sensors.frontDistance);
      break;
    case STATE_L:
      sensors.readSensor(1);
      Serial.printf("🟠 ESTADO L: Izquierdo = %dmm - Esperando 'R'\n", sensors.leftDistance);
      break;
    case STATE_R:
      sensors.readSensor(2);
      Serial.printf("🔴 ESTADO R: Derecho = %dmm - Esperando 'F'\n", sensors.rightDistance);
      break;
    case STATE_READY:
      if (millis() % 5000 < 100) {
        Serial.println("🟢 SISTEMA LISTO - Modo operacional");
      }
      break;
  }
}

void processStateCommand(char command) {
  switch(currentState) {
    case STATE_I:
      if (command == 'F') {
        currentState = STATE_F;
        Serial.println("🎯 Transición: I → F");
      }
      break;
    case STATE_F:
      if (command == 'L') {
        currentState = STATE_L;
        Serial.println("🎯 Transición: F → L");
      }
      break;
    case STATE_L:
      if (command == 'R') {
        currentState = STATE_R;
        Serial.println("🎯 Transición: L → R");
      }
      break;
    case STATE_R:
      if (command == 'F') {
        currentState = STATE_READY;
        Serial.println("🎯 Transición: R → READY");
        Serial.println("🚗 ¡SISTEMA LISTO PARA OPERAR!");
      }
      break;
    case STATE_READY:
      break;
  }
}

void processCommand(char command) {
  processStateCommand(command);
  
  if (currentState != STATE_READY) {
    Serial.println("⏳ Comando ignorado - Sistema en calibración");
    return;
  }
  
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
      motor.turnLeft();
      break;
    case 'R':
      Serial.println("EJECUTANDO: Derecha");
      motor.turnRight();
      break;
    case 'S':
      Serial.println("EJECUTANDO: Detener");
      motor.stopMotor();
      break;
    case '1': speed = 100; Serial.println("VELOCIDAD: 100"); break;
    case '2': speed = 180; Serial.println("VELOCIDAD: 180"); break;
    case '3': speed = 255; Serial.println("VELOCIDAD: 255"); break;
    case 'A':
      autonomousMode = true;
      Serial.println("MODO: Autónomo activado");
      break;
    default:
      Serial.print("ERROR: Comando no reconocido: ");
      Serial.println(command);
      break;
  }
  
  Serial.println("=========================");
}

void autonomousNavigation() {
  // ✅ AHORA sensorFusion ESTÁ DECLARADO
  if (sensorFusion.shouldEmergencyStop()) {
    Serial.println("🛑 PARADA DE EMERGENCIA AUTÓNOMA");
    motor.emergencyStop();
    return;
  }
  
  byte command = sensorFusion.calculateBestCommand();
  
  // Enviar comando al Arduino
  motor.sendAutonomousCommand(command);
  
  // Debug
  sensors.readAll();
  Serial.printf("🤖 DECISIÓN AUTÓNOMA: F:%d L:%d R:%d -> CMD:%d\n",
                sensors.frontDistance, sensors.leftDistance, 
                sensors.rightDistance, command);
}

void setup() {
  Serial.begin(115200);
  
  // Inicializar componentes
  motor.begin();
  sensors.begin();
  ble.begin(BLE_DEVICE_NAME);
  
  Serial.println("Sistema CarRobot con navegación autónoma inicializado");
}

void loop() {
  ble.update();
  
  // Manejar estado del sistema
  static unsigned long lastStatePrint = 0;
  if (millis() - lastStatePrint > 2000) {
    handleSystemState();
    lastStatePrint = millis();
  }
  
  // Leer sensores
  if (currentState == STATE_READY) {
    sensors.readAll();
  }
  /* Solicitar datos del sonar periódicamente
    static unsigned long lastSonarRequest = 0;
    if (millis() - lastSonarRequest > 200) { // Cada 200ms
        motor.requestSonarData();
        lastSonarRequest = millis();
        
        // Usar datos del sonar para decisiones
        int sonarDist = motor.getSonarDistance();
        if (sonarDist > 0 && sonarDist < 100) {
            Serial.printf("📡 Sonar detecta obstáculo: %dcm\n", sonarDist);
        }
    }*/
  // Procesar comandos BLE
  String command = ble.getLastCommand();
  if (command.length() > 0) {
    processCommand(command.charAt(0));
    lastCommandTime = millis();
    autonomousMode = (command.charAt(0) == 'A');
  }
  
  // ✅ Navegación autónoma con sensorFusion declarado
  if (currentState == STATE_READY && autonomousMode) {
    autonomousNavigation();
  }
  // ✅ MODIFICADO: Enviar STOP solo una vez al timeout
    static bool stopSent = false;
    if (currentState == STATE_READY && 
        millis() - lastCommandTime > SAFETY_TIMEOUT && 
        !autonomousMode) {
        if (!stopSent) {
            motor.stopMotor();
            Serial.println("⏰ Timeout - Enviando STOP");
            stopSent = true;
        }
    } else {
        stopSent = false; // Resetear cuando hay actividad
    }
  

  
  // Enviar datos via BLE
  if (ble.isConnected() && currentState == STATE_READY) {
    static unsigned long lastBLESend = 0;
    if (millis() - lastBLESend > 1000) {
      String sensorData = "F:" + String(sensors.frontDistance) +
                       " L:" + String(sensors.leftDistance) +
                       " R:" + String(sensors.rightDistance) +
                       " M:" + (autonomousMode ? "AUTO" : "MANUAL");
      ble.sendData(sensorData);
      lastBLESend = millis();
    }
  }
  
  delay(50);
}

// Configurar velocidades
void setManualSpeed(int speed) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(CMD_SET_MANUAL_SPEED);
    Wire.write(speed);
    Wire.endTransmission();
}

void setAvoidanceSpeed(int speed, int turnSpeed) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(CMD_SET_AVOIDANCE_PARAMS);
    Wire.write(speed);
    Wire.write(turnSpeed);
    Wire.endTransmission();
}
/*
#include <Arduino.h>
#include "ble/BluetoothLeConnect.h"
#include "MotorControl.h"
#include "SensorControl.h"
#include "Command.h"
#include "SensorFusion.h"
//#include "config.h"

#define BLE_DEVICE_NAME "CarRobot-ESP32-S3"
#define ARDUINO_I2C_ADDRESS 0x08
#define CMD_MANUAL_FORWARD 1

//TwoWire I2C_TOF = TwoWire(0);    // Bus 0 para sensores
//TwoWire I2C_ARDUINO = TwoWire(1); // Bus 1 para Arduino
// no funcionó, lo moví a SensorControl.cpp

// Instancias
BluetoothLeConnect ble;
MotorControl motor(ARDUINO_I2C_ADDRESS); //, &I2C_ARDUINO);
SensorControl sensors;
SensorFusion sensorFusion(sensors);
bool autonomousMode = false;
unsigned long lastCommandTime = 0;
int speed = 150;

// Estados del sistema
enum SystemState {
  STATE_I, // Estado inicial - Esperando "F"
  STATE_F, // Test sensor frontal - Esperando "L"  
  STATE_L, // Test sensor izquierdo - Esperando "R"
  STATE_R, // Test sensor derecho - Esperando "F" para empezar
  STATE_READY // Sistema listo para operar
};

SystemState currentState = STATE_I;

void autonomousNavigationBasic();
void handleSystemState();
void printSystemStatus();
const unsigned long SAFETY_TIMEOUT = 1500;

void processCommand(char cmd);
void autonomousNavigation();

void handleSystemState() {
  switch(currentState) {
    case STATE_I:
      // Estado inicial - Esperando "F" por BLE
      Serial.println("🔵 ESTADO I: Esperando comando 'F' para iniciar calibración");
      Serial.println("   Envía 'F' por BLE para testear sensor frontal");
      break;
      
    case STATE_F:
      // Test sensor frontal
      sensors.readSensor(0); // Leer solo frontal
      Serial.printf("🟡 ESTADO F: Sensor frontal = %dmm - Esperando 'L'\n", sensors.frontDistance);
      Serial.println("   Envía 'L' por BLE para testear sensor izquierdo");
      
      // Verificar que el sensor frontal funcione
      if (sensors.frontDistance < 100 || sensors.frontDistance > 1200) {
        Serial.println("   ⚠️  ¡POSIBLE PROBLEMA EN SENSOR FRONTAL!");
      }
      break;
      
    case STATE_L:
      // Test sensor izquierdo  
      sensors.readSensor(1); // Leer solo izquierdo
      Serial.printf("🟠 ESTADO L: Sensor izquierdo = %dmm - Esperando 'R'\n", sensors.leftDistance);
      Serial.println("   Envía 'R' por BLE para testear sensor derecho");
      
      if (sensors.leftDistance < 100 || sensors.leftDistance > 1200) {
        Serial.println("   ⚠️  ¡POSIBLE PROBLEMA EN SENSOR IZQUIERDO!");
      }
      break;
      
    case STATE_R:
      // Test sensor derecho
      sensors.readSensor(2); // Leer solo derecho
      Serial.printf("🔴 ESTADO R: Sensor derecho = %dmm - Esperando 'F' para empezar\n", sensors.rightDistance);
      Serial.println("   Envía 'F' por BLE para comenzar operación normal");
      
      if (sensors.rightDistance < 100 || sensors.rightDistance > 1200) {
        Serial.println("   ⚠️  ¡POSIBLE PROBLEMA EN SENSOR DERECHO!");
      }
      break;
      
    case STATE_READY:
      // Sistema listo - operación normal
      if (millis() % 5000 < 100) { // Cada 5 segundos
        Serial.println("🟢 SISTEMA LISTO - Modo operacional activo");
        Serial.println("   Comandos: F=Adelante, B=Atrás, L=Izquierda, R=Derecha");
        Serial.println("   S=Stop, A=Autónomo, 1/2/3=Velocidad");
      }
      break;
  }
}

void processStateCommand(char command) {
  switch(currentState) {
    case STATE_I:
      if (command == 'F') {
        currentState = STATE_F;
        Wire.beginTransmission(0x08);
        Wire.write(CMD_SET_MANUAL_SPEED);  // Comando 10
        Wire.write(180);
        Wire.endTransmission(); 
        Wire.write(CMD_MANUAL_FORWARD);
        Serial.println("🎯 Transición: I → F (Test sensor frontal)");
      }
      break;
      
    case STATE_F:
      if (command == 'L') {
        currentState = STATE_L;
        Wire.beginTransmission(0x08);
        Wire.write(CMD_MANUAL_FORWARD);    // Comando 1
        Wire.endTransmission(); 
        Serial.println("🎯 Transición: F → L (Test sensor izquierdo)");
      }
      break;
      
    case STATE_L:
      if (command == 'R') {
        currentState = STATE_R;
        Serial.println("🎯 Transición: L → R (Test sensor derecho)");
      }
      break;
      
    case STATE_R:
      if (command == 'F') {
        currentState = STATE_READY;
        Serial.println("🎯 Transición: R → READY (Sistema operacional)");
        Serial.println("🚗 ¡SISTEMA LISTO PARA OPERAR!");
      }
      break;
      
    case STATE_READY:
      // En estado READY, todos los comandos funcionan normalmente
      break;
  }
}
void processCommand(char command) {
  // Primero procesar comandos de estado
  processStateCommand(command);
  
  // Solo procesar comandos de movimiento si estamos en estado READY
  if (currentState != STATE_READY) {
    Serial.println("⏳ Comando ignorado - Sistema en calibración");
    return;
  }
  
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

void setup() {
  Serial.begin(115200);
  
  // Inicializar componentes
  motor.begin();
  delay(1000);
  
  sensors.begin();
  // Ejecutar diagnóstico de sensores
  sensors.diagnoseSensors();
  // InicializarBLE
  ble.begin(BLE_DEVICE_NAME);
  
  Serial.println("Sistema CarRobot con control I2C inicializado");
}

void loop() {
  ble.update();

  // Manejar el estado del sistema (muestra info cada 2 segundos)
  static unsigned long lastStatePrint = 0;
  if (millis() - lastStatePrint > 2000) {
    handleSystemState();
    lastStatePrint = millis();
  }
  
  // Leer sensores según el estado
  if (currentState == STATE_READY) {
    sensors.readAll();
  }
  
  // Procesar comandos BLE
  String command = ble.getLastCommand();
  if (command.length() > 0) {
    processCommand(command.charAt(0));
    lastCommandTime = millis();
    autonomousMode = (command.charAt(0) == 'A'); // Activar auto con 'A'
  }
  
  if (currentState == STATE_READY && 
        millis() - lastCommandTime > SAFETY_TIMEOUT && 
        !autonomousMode) {
        motor.stopMotor(); // Parar si no hay actividad
    }

  
  // Enviar datos de sensores via BLE
  if (ble.isConnected() && currentState == STATE_READY) {
    static unsigned long lastBLESend = 0;
    if (millis() - lastBLESend > 1000) {
      String sensorData = "F:" + String(sensors.frontDistance) +
                         " L:" + String(sensors.leftDistance) +
                         " R:" + String(sensors.rightDistance);
      ble.sendData(sensorData);
      lastBLESend = millis();
    }
  }
  
  delay(100);
}
void autonomousNavigation() {
    if (sensorFusion.shouldEmergencyStop()) {
        Serial.println("🛑 PARADA DE EMERGENCIA AUTÓNOMA");
        motor.emergencyStop();
        return;
    }
    
    byte command = sensorFusion.calculateBestCommand();
    
    // Enviar comando al Arduino
    motor.sendAutonomousCommand(command);
    
    // Debug de la decisión
    sensors.readAll();
    Serial.printf("🤖 DECISIÓN AUTÓNOMA: F:%d L:%d R:%d -> CMD:%d\n",
                  sensors.frontDistance, sensors.leftDistance, 
                  sensors.rightDistance, command);
}

// Configurar diferentes velocidades por modo
void setAvoidanceSpeed(int speed, int turnSpeed) {
    Wire.beginTransmission(8);
    Wire.write(CMD_SET_AVOIDANCE_PARAMS);
    Wire.write(speed);
    Wire.write(turnSpeed);
    Wire.endTransmission();
}

void setManualSpeed(int speed) {
    Wire.beginTransmission(ARDUINO_I2C_ADDRESS);
    Wire.write(CMD_SET_MANUAL_SPEED); // Cambiar por el comando correcto
    Wire.write(speed);
    Wire.endTransmission();
}
*/
