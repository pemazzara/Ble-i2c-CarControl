/*
#include <Wire.h>
#include <VL53L0X.h>


#define I2C_PORT I2C_NUM_0


#define SDA_PIN 8
#define SCL_PIN 9

VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("🔍 Test VL53L0X individual");
  
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);
  
  Serial.println("Inicializando sensor...");
  if (sensor.init()) {
    Serial.println("✅ Sensor inicializado");
    sensor.setTimeout(500);
    sensor.startContinuous();
  } else {
    Serial.println("❌ Falló inicialización");
    return;
  }
}

void loop() {
  uint16_t distance = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    Serial.println("⏰ Timeout");
  } else {
    Serial.printf("Distancia: %d mm\n", distance);
  }
  delay(100);
}
*/
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
        Serial.println("🎯 Transición: I → F (Test sensor frontal)");
      }
      break;
      
    case STATE_F:
      if (command == 'L') {
        currentState = STATE_L; 
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
  // Inicializar I2C
  Wire.begin(I2C_SENSORES_SDA_PIN, I2C_SENSORES_SCL_PIN);
  delay(1000);
  sensors.begin();
  // Ejecutar diagnóstico de sensores
  sensors.diagnoseSensors();
  // Inicializar BLE
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
    
    if (currentState == STATE_READY) {
      autonomousMode = false;
      lastCommandTime = millis();
    }
  }
  
  // Navegación autónoma solo en estado READY
  if (currentState == STATE_READY && 
      millis() - lastCommandTime > SAFETY_TIMEOUT) {
    autonomousMode = true;
  } 
  
  if (autonomousMode && currentState == STATE_READY) {
    autonomousNavigationBasic();
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





void autonomousNavigationBasic() {
  // Leer sensores reales
  sensors.readAll();
  sensors.printDistances();  // Debug
  
  // Lógica de navegación inteligente
  uint16_t safeDistance = 300;  // 30cm mínimo seguro
  uint16_t turnDistance = 200;  // 20cm para giro urgente
  
  Serial.println("🤖 MODO AUTÓNOMO ACTIVO");
  
  if (sensors.frontDistance > safeDistance) {
    // Camino despejado adelante
    if (sensors.leftDistance < turnDistance && sensors.rightDistance > safeDistance) {
      motor.softTurnRight();
      Serial.println("↪️ Esquivando obstáculo izquierdo");
    }
    else if (sensors.rightDistance < turnDistance && sensors.leftDistance > safeDistance) {
      motor.softTurnLeft();
      Serial.println("↩️ Esquivando obstáculo derecho");
    }
    else {
      motor.moveForward(120);
      Serial.println("🟢 Adelante - Camino despejado");
    }
  }
  else {
    // Obstáculo frontal - decidir mejor giro
    if (sensors.leftDistance > sensors.rightDistance) {
      motor.softTurnLeft();
      Serial.println("🔄 Giro izquierda - Más espacio a la izquierda");
      delay(800);
    }
    else {
      motor.softTurnRight();
      Serial.println("🔄 Giro derecha - Más espacio a la derecha");
      delay(800);
    }
  }
}
 /* 
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
*/