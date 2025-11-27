#include "ServoControl.h"

#define SERVO_PIN 20

void ServoControl::begin() {
  // Permitir asignación de timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Configurar servo
  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  centerSteering();
  
  Serial.println("Servo inicializado");
}

void ServoControl::centerSteering() {
  steeringServo.write(90);
}

void ServoControl::turnLeft() {
  steeringServo.write(60);
}

void ServoControl::turnRight() {
  steeringServo.write(120);
}

void ServoControl::smoothLeft() {
  steeringServo.write(70);
}

void ServoControl::smoothRight() {
  steeringServo.write(110);
}