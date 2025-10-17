#include <Arduino.h>
#include <Wire.h>

// Dirección I2C
#define I2C_ADDR 0x08
// Comandos I2C
#define CMD_STOP       0x00
#define CMD_FORWARD    0x01
#define CMD_BACKWARD   0x02
#define CMD_LEFT       0x03
#define CMD_RIGHT      0x04
#define CMD_SPEED      0x05
#define CMD_SOFT_LEFT  0x06  // Nuevo: Giro suave izquierda
#define CMD_SOFT_RIGHT 0x07  // Nuevo: Giro suave derecha

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

// Variables para control de velocidad y dirección
int velocidad_base = 150; // Velocidad media
int velocidad_giro = 100; // Velocidad específica para giros
bool isMovingForward = false;

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

/**
 * Giro EN EL SITIO - Ambos motores en direcciones opuestas
 * (Más brusco, para giros de 90° o 180°)
 */
void girarIzquierda(int speed) {
  // Motor izquierdo atrás, motor derecho adelante
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
  // Motor izquierdo adelante, motor derecho atrás
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  
  Serial.print("Giro en sitio derecha - Velocidad: ");
  Serial.println(speed);
}

/**
 * GIRO SUAVE - Un motor más lento que el otro
 * (Para curvas suaves mientras avanza)
 */
void giroSuaveIzquierda(int velocidadAdelante) {
  // Motor izquierdo más lento, derecho a velocidad normal
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, velocidadAdelante * 0.4);  // 40% de velocidad
  analogWrite(ENB, velocidadAdelante);        // 100% de velocidad
  
  Serial.print("Giro suave izquierda - Vel: ");
  Serial.print(velocidadAdelante * 0.4);
  Serial.print("/");
  Serial.println(velocidadAdelante);
}

void giroSuaveDerecha(int velocidadAdelante) {
  // Motor izquierdo a velocidad normal, derecho más lento
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, velocidadAdelante);        // 100% de velocidad
  analogWrite(ENB, velocidadAdelante * 0.4);  // 40% de velocidad
  
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

/**
 * Rampa suave para cambios de velocidad progresivos
 */
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
  if (Wire.available()) {
    uint8_t command = Wire.read();
   
    switch (command) {
      case CMD_STOP:
        detener();
        Serial.println("Comando: Detener");
        break;
       
      case CMD_FORWARD:
        moverAdelante(velocidad_base);
        isMovingForward = true;
        Serial.println("Comando: Adelante");
        break;
       
      case CMD_BACKWARD:
        moverAtras(velocidad_base * 0.7); // Más lento en reversa
        isMovingForward = false;
        Serial.println("Comando: Atrás");
        break;
       
      case CMD_LEFT:
        // Giro en sitio (brusco)
        girarIzquierda(velocidad_giro);
        Serial.println("Comando: Giro izquierda (sitio)");
        break;
       
      case CMD_RIGHT:
        // Giro en sitio (brusco)
        girarDerecha(velocidad_giro);
        Serial.println("Comando: Giro derecha (sitio)");
        break;
      
      case CMD_SOFT_LEFT:
        // Giro suave (curva)
        giroSuaveIzquierda(velocidad_base);
        Serial.println("Comando: Giro suave izquierda");
        break;
      
      case CMD_SOFT_RIGHT:
        // Giro suave (curva)
        giroSuaveDerecha(velocidad_base);
        Serial.println("Comando: Giro suave derecha");
        break;
             
      case CMD_SPEED:
        if (Wire.available()) {
          uint8_t speed = Wire.read();
          velocidad_base = speed;
          velocidad_giro = speed * 0.8; // Giros al 80% de la velocidad base
          Serial.print("Velocidad base establecida: ");
          Serial.println(speed);
        }
        break;
    }
  }
}

void loop() {
  delay(10); 
}