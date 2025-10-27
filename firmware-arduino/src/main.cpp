// main.cpp
#include <Arduino.h>
#include <Wire.h>
#include "MotorController.h"
#include "ObstacleAvoider.h"

// PINES L298N
const int ENA = 9;   // PWM motor izquierdo
const int IN1 = 7;   
const int IN2 = 6;   
const int ENB = 10;  // PWM motor derecho  
const int IN3 = 5;   
const int IN4 = 4;

// Pines sonar (si lo usas)
#define TRIG_PIN 8   // Cambié para evitar conflicto
#define ECHO_PIN 11  // Cambié para evitar conflicto

#define I2C_ADDR 0x08



void handleI2CCommand(int howMany);

// Instancias actualizada con pines L298N
MotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4);
ObstacleAvoider obstacleAvoider(TRIG_PIN, ECHO_PIN, motorController);


void setup() {
    Serial.begin(9600);
    
    // Inicializar control de motores
    motorController.begin();
    obstacleAvoider.begin();  
    // Configurar I2C
    Wire.begin(I2C_ADDR);
    Wire.onReceive(handleI2CCommand);
    
    Serial.println("🚗 Sistema Arduino listo - Prioridades activas");
}

void handleI2CCommand(int howMany) {
    if (howMany >= 1) {
        byte command = Wire.read();
        byte data = 0;
        
        // Solo leer data si es un comando que la necesita
        if ((command == CMD_SET_MANUAL_SPEED || command == CMD_SET_AUTO_SPEED) && 
            howMany >= 2) {
            data = Wire.read();
        }
        
        motorController.handleCommand(command, data);
    }
}

void loop() {
    // 1. Seguridad continua del motor controller
    motorController.updateSafety();
    
    // 2. Solo ejecutar evasión de obstáculos si estamos en modo AUTO
    // y no hay emergencia
    if (motorController.getCurrentMode() == MODE_AUTONOMOUS && 
        !motorController.isInEmergency()) {
        obstacleAvoider.update();
    }
    
    delay(50);
}


/*
#include <Arduino.h>
#include <Wire.h>
#include "ObstacleAvoider.h"
#include "MotorController.h"

// Pines para sensores y motores
#define TRIG_PIN 9
#define ECHO_PIN 10
#define MOTOR_LEFT_1 5
#define MOTOR_LEFT_2 6
#define MOTOR_RIGHT_1 10
#define MOTOR_RIGHT_2 11

#define I2C_ADDR 0x08

// ==============================================
//  DEFINICIÓN DE PINES
// ==============================================

// PINES DEL MOTOR A (IZQUIERDO)
const int ENA = 9;   // Habilitar (Velocidad) - PWM
const int IN1 = 7;   // Dirección 1
const int IN2 = 6;   // Dirección 2

// PINES DEL MOTOR B (DERECHO)
const int ENB = 10;  // Habilitar (Velocidad) - PWM
const int IN3 = 5;   // Dirección 1
const int IN4 = 4;   // Dirección 2

OperationMode currentMode = MODE_MANUAL;

ObstacleAvoider obstacleAvoider(TRIG_PIN, ECHO_PIN, 
                               MOTOR_LEFT_1, MOTOR_LEFT_2,
                               MOTOR_RIGHT_1, MOTOR_RIGHT_2);

// ==============================================
//  PROTOTIPOS DE FUNCIONES
// ==============================================
void detener();
void moverAdelante(int speed);
void moverAtras(int speed);
void girarIzquierda(int speed);
void girarDerecha(int speed);
void giroSuaveIzquierda(int velocidadAdelante);
void giroSuaveDerecha(int velocidadAdelante);
void receiveEvent(int howMany);
void requestEvent(); // AGREGAR ESTE PROTOTIPO
void handleCommand(byte command); // AGREGAR ESTE PROTOTIPO
void rampaMotores(int speedA, int speedB, int duracion = 100);

// ==============================================
//  CONFIGURACIÓN
// ==============================================
void setup() {
  // Configura todos los pines de control como SALIDAS
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  detener(); // Asegura que los motores estén detenidos al inicio
  
  // Inicializar I2C como esclavo
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  
  obstacleAvoider.begin();
  Serial.begin(9600);
  Serial.println("Arduino listo para recibir comandos I2C - Control mejorado");
}

// ==============================================
//  FUNCIONES MEJORADAS DE MOVIMIENTO
// ==============================================

void moverAdelante(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void moverAtras(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void girarIzquierda(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  
  Serial.print("Giro en sitio izquierda - Velocidad: ");
  Serial.println(speed);
}

void girarDerecha(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  
  Serial.print("Giro en sitio derecha - Velocidad: ");
  Serial.println(speed);
}

void giroSuaveIzquierda(int velocidadAdelante) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, velocidadAdelante * 0.4);
  analogWrite(ENB, velocidadAdelante);
  
  Serial.print("Giro suave izquierda - Vel: ");
  Serial.print(velocidadAdelante * 0.4);
  Serial.print("/");
  Serial.println(velocidadAdelante);
}

void giroSuaveDerecha(int velocidadAdelante) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, velocidadAdelante);
  analogWrite(ENB, velocidadAdelante * 0.4);
  
  Serial.print("Giro suave derecha - Vel: ");
  Serial.print(velocidadAdelante);
  Serial.print("/");
  Serial.println(velocidadAdelante * 0.4);
}

void detener() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void rampaMotores(int speedA, int speedB, int duracion) {
  int currentA = analogRead(ENA);
  int currentB = analogRead(ENB);
  int steps = 10;
  
  for (int i = 0; i <= steps; i++) {
    int newA = currentA + (speedA - currentA) * i / steps;
    int newB = currentB + (speedB - currentB) * i / steps;
    
    analogWrite(ENA, newA);
    analogWrite(ENB, newB);
    delay(duracion / steps);
  }
}

// ==============================================
//  MANEJADOR DE COMANDOS I2C MEJORADO
// ==============================================

void receiveEvent(int howMany) {
  if (howMany >= 1) {
    byte command = Wire.read();
    handleCommand(command);
    Serial.print("Comando recibido: ");
    Serial.println(command);
  }
}

void handleCommand(byte command) {
  switch (command) {
    case CMD_STOP:
      currentMode = MODE_MANUAL;
      detener(); // Usar detener() en lugar de obstacleAvoider.stop()
      break;
      
    case CMD_FORWARD:
      currentMode = MODE_MANUAL;
      moverAdelante(150);
      break;
      
    case CMD_BACKWARD:
      currentMode = MODE_MANUAL;
      moverAtras(150);
      break;
      
    case CMD_LEFT:
      currentMode = MODE_MANUAL;
      girarIzquierda(100);
      break;
      
    case CMD_RIGHT:
      currentMode = MODE_MANUAL;
      girarDerecha(100);
      break;
      
    case CMD_OBSTACLE_AVOIDANCE:
      currentMode = MODE_OBSTACLE_AVOIDANCE;
      obstacleAvoider.setActive(true);
      break;
      
    case CMD_WALL_FOLLOWING:
      currentMode = MODE_WALL_FOLLOWING;
      obstacleAvoider.setActive(true);
      break;
      
    case CMD_EMERGENCY_STOP:
      currentMode = MODE_MANUAL;
      obstacleAvoider.setActive(false);
      detener();
      break;
      
    case CMD_SET_AVOIDANCE_PARAMS:
      if (Wire.available() >= 3) {
        int minDist = Wire.read();
        int avoidSpeed = Wire.read();
        int turnSpd = Wire.read();
        obstacleAvoider.setAvoidanceParameters(minDist, avoidSpeed, turnSpd);
      }
      break;
      
    case CMD_SET_WALL_FOLLOW_PARAMS:
      if (Wire.available() >= 2) {
        int speed = Wire.read();
        int idealDistance = Wire.read();
        obstacleAvoider.setWallFollowParameters(speed, idealDistance);
      }
      break;
      
    case CMD_SET_MANUAL_SPEED:
      if (Wire.available() >= 1) {
        int speed = Wire.read();
        obstacleAvoider.setManualSpeed(speed);
      }
      break;
  }
}

void requestEvent() {
  int distance = obstacleAvoider.getDistance();
  Wire.write((byte)(distance & 0xFF));
  Wire.write((byte)((distance >> 8) & 0xFF));
  Wire.write((byte)currentMode);
}
void loop() {
    // 1. Verificar comandos I2C (máxima prioridad)
    checkI2CCommands();
    
    // 2. Seguridad local con sonar (solo si no en emergencia)
    if (!motorController.isInEmergency()) {
        obstacleAvoider.updateSafety();
    }
    
    // 3. Ejecutar movimiento actual
    motorController.updateMovement();
    
    delay(20); // 50Hz update rate
}*/
/*
void loop() {
  switch (currentMode) {
    case MODE_OBSTACLE_AVOIDANCE:
      obstacleAvoider.update();
      break;
    case MODE_WALL_FOLLOWING:
      obstacleAvoider.wallFollowing();
      break;
    case MODE_MANUAL:
    default:
      break;
  }
  delay(50);
}*/