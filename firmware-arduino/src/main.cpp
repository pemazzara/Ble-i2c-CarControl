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


// ==============================================
//  DEFINICIÓN DE PINES
// ==============================================

// PINES DEL MOTOR A (IZQUIERDO, por ejemplo)
const int ENA = 9;   // Habilitar (Velocidad) - PWM
const int IN1 = 7;   // Dirección 1
const int IN2 = 6;   // Dirección 2

// PINES DEL MOTOR B (DERECHO, por ejemplo)
const int ENB = 10;  // Habilitar (Velocidad) - PWM
const int IN3 = 5;   // Dirección 1
const int IN4 = 4;   // Dirección 2

// Variables para control de velocidad y dirección
int velocidad_base = 150; // Velocidad media. Velocidad actual (0-255) 
bool isMovingForward = false; // Dirección actual
void detener();
void moverAdelante(int speed);
void moverAtras(int speed);
void girarIzquierda(int speed);
void girarDerecha(int speed);
void receiveEvent(int howMany);

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
  Serial.println("Arduino listo para recibir comandos I2C");

}

// ==============================================
//  FUNCIONES DE MOVIMIENTO DEL ROBOT
// ==============================================

/**
 * Mueve el robot hacia adelante.
 * @param speed La velocidad (0 a 255).
 */
void moverAdelante(int speed) {
  // Motor A (Izquierdo) - Gira Adelante
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  // Motor B (Derecho) - Gira Adelante
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

/**
 * Mueve el robot hacia atrás.
 * @param speed La velocidad (0 a 255).
 */
void moverAtras(int speed) {
  // Motor A (Izquierdo) - Gira Atrás
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  // Motor B (Derecho) - Gira Atrás
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

/**
 * Gira el robot a la izquierda (Motor A en reversa, Motor B adelante).
 * @param speed La velocidad (0 a 255).
 */
void girarIzquierda(int speed) {
  // Motor A (Izquierdo) - Reversa (para un giro en su sitio)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  // Motor B (Derecho) - Adelante
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

/**
 * Gira el robot a la derecha (Motor A Adelante, Motor B en reversa).
 * @param speed La velocidad (0 a 255).
 */
void girarDerecha(int speed) {
  // Motor A (Izquierdo) - Adelante (para un giro en su sitio)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  // Motor B (Derecho) - Atrás
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

/**
 * Detiene ambos motores.
 */
void detener() {
  // Poner la velocidad a 0 detiene los motores.
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  
  // Opcional: También puedes poner las direcciones en LOW para un "freno suave"
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); 
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); 
}

void turnInPlace(bool clockwise, int speed) {
  if (clockwise) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, speed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, speed);
  }
}

// ==============================================
//  LOOP PRINCIPAL (Pequeño delay para estabilidad)
// ==============================================
void loop() {
  
delay(10); 

}

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
        moverAtras(100); // Más lento
        isMovingForward = false;
        Serial.println("Comando: Atrás lento");
        break;
       
      case CMD_LEFT:
         girarIzquierda(velocidad_base);
        Serial.println("Comando: Izquierda");
        break;
       
      case CMD_RIGHT:
        girarDerecha(velocidad_base);
        Serial.println("Comando: Derecha");
        break;
             
      case CMD_SPEED:
        if (Wire.available()) {
          uint8_t speed = Wire.read();
          analogWrite(ENA, speed);
          analogWrite(ENB, speed);
          velocidad_base = speed;
          Serial.print("Velocidad establecida: ");
          Serial.println(speed);
        }
        break;
    }
  }
}
