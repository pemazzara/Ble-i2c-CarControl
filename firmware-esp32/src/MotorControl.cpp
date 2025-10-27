// MotorControl.cpp
#include <Arduino.h>
#include "MotorControl.h"
#include "Command.h" // Incluir los nuevos comandos



void MotorControl::begin() {
    // Inicializar I2C como maestro
    //i2cBus = &Wire;
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(100);
    
    // Verificar conexión con Arduino
    Wire.beginTransmission(ARDUINO_ADDR);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
        Serial.println("Conexión I2C con Arduino establecida correctamente");
    } else {
        Serial.println("Error en conexión I2C con Arduino");
    }
    
    // Inicializar motor detenido
    stopMotor();
    currentManualSpeed = 0; 
}

void MotorControl::sendI2CCommand(uint8_t command, uint8_t value) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(command);
    
    if (command == CMD_SET_MANUAL_SPEED) {
        Wire.write(value);
    }
    
    Wire.endTransmission();
}


void MotorControl::moveForward(int speed) {
    // Primero configurar la velocidad si es diferente
    if (currentManualSpeed != speed) {
        setManualSpeed(speed);
        currentManualSpeed = speed;
    }
    // Luego enviar comando de movimiento
    sendI2CCommand(CMD_MANUAL_FORWARD);
}

void MotorControl::moveBackward(int speed) {
    if (currentManualSpeed != speed) {
        setManualSpeed(speed);
        currentManualSpeed = speed;
    }
    sendI2CCommand(CMD_MANUAL_BACKWARD);
}

void MotorControl::turnLeft() {
    sendI2CCommand(CMD_MANUAL_LEFT);
}

void MotorControl::turnRight() {
    sendI2CCommand(CMD_MANUAL_RIGHT);
}

void MotorControl::stopMotor() {
    sendI2CCommand(CMD_STOP);
}

void MotorControl::enableObstacleAvoidance() {
    sendI2CCommand(CMD_OBSTACLE_AVOIDANCE);
}

void MotorControl::enableWallFollowing() {
    sendI2CCommand(CMD_WALL_FOLLOWING);
}

void MotorControl::emergencyStop() {
    sendI2CCommand(CMD_EMERGENCY_STOP);
}

void MotorControl::setManualSpeed(int speed) {
    sendI2CCommand(CMD_SET_MANUAL_SPEED, speed);
}

void MotorControl::setAvoidanceParameters(int minDistance, int speed, int turnSpeed) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(CMD_SET_AVOIDANCE_PARAMS);
    Wire.write(minDistance);
    Wire.write(speed);
    Wire.write(turnSpeed);
    Wire.endTransmission();
}

void MotorControl::setWallFollowParameters(int speed, int idealDistance) {
    Wire.beginTransmission(ARDUINO_ADDR);
    Wire.write(CMD_SET_WALL_FOLLOW_PARAMS);
    Wire.write(speed);
    Wire.write(idealDistance);
    Wire.endTransmission();
}

// Eliminar las funciones de giro suave o reimplementarlas
void MotorControl::softTurnLeft() {
    // Implementar giro suave como una combinación de comandos
    // o eliminar esta función si no está en los comandos del Arduino
    turnLeft(); // Usar giro normal por ahora
}

void MotorControl::softTurnRight() {
    turnRight(); // Usar giro normal por ahora
}
void MotorControl::enableAutonomousMode() {
    sendI2CCommand(CMD_AUTO_NAVIGATE);
}
void MotorControl::sendAutonomousCommand(byte command) {
    // Validar que el comando sea autónomo
    if (command >= CMD_AUTO_FORWARD && command <= CMD_AUTO_NAVIGATE) {
        sendI2CCommand(command);
    } else {
        Serial.println("❌ Comando autónomo no válido");
    }
}

/*
#include "MotorControl.h"

void MotorControl::begin() {
  // Inicializar I2C como maestro
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(100);
  
  // Verificar conexión con Arduino
  Wire.beginTransmission(ARDUINO_ADDR);
  byte error = Wire.endTransmission();
  
  if (error == 0) {
    Serial.println("Conexión I2C con Arduino establecida correctamente");
  } else {
    Serial.println("Error en conexión I2C con Arduino");
  }
  
  // Inicializar motor detenido
  stopMotor();
  currentSpeed = 0;
  isMovingForward = false;
}

void MotorControl::sendI2CCommand(uint8_t command, uint8_t value) {
  Wire.beginTransmission(ARDUINO_ADDR);
  Wire.write(command);
  if (command == CMD_SPEED || value > 0) {
    Wire.write(value);
  }
  byte error = Wire.endTransmission();

  if (error != 0) {
    Serial.printf("Error enviando comando I2C: %d\n", error);
  } else {
    Serial.print("Comando enviado via I2C: ");
    Serial.print(command);
    if (command == CMD_SPEED || value > 0) {
      Serial.print(" Valor: ");
      Serial.println(value);
    } else {
      Serial.println();
    }
  }
}
   

void MotorControl::moveForward(int speed) {
  sendI2CCommand(CMD_FORWARD);
  sendI2CCommand(CMD_SPEED, speed);
  currentSpeed = speed;
  isMovingForward = true;
  Serial.printf("Moviendo hacia adelante a velocidad: %d\n", speed);
}

void MotorControl::moveBackward(int speed) {
  sendI2CCommand(CMD_BACKWARD);
  sendI2CCommand(CMD_SPEED, speed);
  currentSpeed = speed;
  isMovingForward = false;
  Serial.printf("Moviendo hacia atrás a velocidad: %d\n", speed);
}

void MotorControl::stopMotor() {
  sendI2CCommand(CMD_STOP);
  currentSpeed = 0;
  Serial.println("Motor detenido");
}

void MotorControl::turnLeft() {
  sendI2CCommand(CMD_LEFT);
  Serial.println("Girando a la izquierda");
}

void MotorControl::turnRight() {
  sendI2CCommand(CMD_RIGHT);
  Serial.println("Girando a la derecha");
}

void MotorControl::softTurnLeft() {
  sendI2CCommand(CMD_SOFT_LEFT);
  Serial.println("Giro suave a la izquierda");
}

void MotorControl::softTurnRight() {
  sendI2CCommand(CMD_SOFT_RIGHT);
  Serial.println("Giro suave a la derecha");
}

void MotorControl::softStop(int delayTime) {
  int initialSpeed = currentSpeed;
  int steps = 10;
  
  for (int i = steps; i > 0; i--) {
    int newSpeed = (initialSpeed * i) / steps;
    if (isMovingForward) {
      moveForward(newSpeed);
    } else {
      moveBackward(newSpeed);
    }
    delay(delayTime / steps);
  }
  
  stopMotor();
}
/*
void MotorControl::centerSteering() {
  sendI2CCommand(CMD_CENTER);
  Serial.println("Centrando dirección");
}

void setupI2C() {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Serial.println("I2C inicializado");
    delay(100); // Esperar a que el bus I2C se estabilice 
}
// Función para enviar comandos al Arduino via I2C
void sendI2CCommand(uint8_t command, uint8_t value = 0) {
  Wire.beginTransmission(ARDUINO_ADDR);
  Wire.write(command);
  if (command == CMD_SPEED) {
    Wire.write(value);
  }
  Wire.endTransmission();
  
  Serial.print("Comando enviado via I2C: ");
  Serial.print(command);
  if (command == CMD_SPEED) {
    Serial.print(" Valor: ");
    Serial.println(value);
  } else {
    Serial.println();
  }
}

void MotorControl::rampSpeed(int targetSpeed, int rampTime) {
  int steps = 20;
  int current = currentSpeed;
  int increment = (targetSpeed - current) / steps;
  
  for (int i = 0; i < steps; i++) {
    current += increment;
    if (isMovingForward) {
      moveForward(constrain(current, 0, 255));
    } else {
      moveBackward(constrain(current, 0, 255));
    }
    delay(rampTime / steps);
  }
}*/
