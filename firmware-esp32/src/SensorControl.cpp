#include "SensorControl.h"

// Direcciones I2C únicas
#define FRONT_ADDRESS    0x30
#define LEFT_ADDRESS     0x31
#define RIGHT_ADDRESS    0x32

// Pines XSHUT
#define FRONT_XSHUT_PIN  5
#define LEFT_XSHUT_PIN   6
#define RIGHT_XSHUT_PIN  7

void SensorControl::begin() {
  initSensors();
}

void SensorControl::initSensors() {
  // Configurar pines XSHUT
  pinMode(FRONT_XSHUT_PIN, OUTPUT);
  pinMode(LEFT_XSHUT_PIN, OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);
  
  // Apagar todos los sensores
  digitalWrite(FRONT_XSHUT_PIN, LOW);
  digitalWrite(LEFT_XSHUT_PIN, LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);
  delay(10);

  // Inicializar sensor frontal
  digitalWrite(FRONT_XSHUT_PIN, HIGH);
  delay(10);
  sensorFront.setAddress(FRONT_ADDRESS);
  if (!sensorFront.init()) {
    Serial.println("Error al iniciar sensor frontal");
    while (1);
  }
  sensorFront.setTimeout(500);

  // Inicializar sensor izquierdo
  digitalWrite(LEFT_XSHUT_PIN, HIGH);
  delay(10);
  sensorLeft.setAddress(LEFT_ADDRESS);
  if (!sensorLeft.init()) {
    Serial.println("Error al iniciar sensor izquierdo");
    while (1);
  }
  sensorLeft.setTimeout(500);

  // Inicializar sensor derecho
  digitalWrite(RIGHT_XSHUT_PIN, HIGH);
  delay(10);
  sensorRight.setAddress(RIGHT_ADDRESS);
  if (!sensorRight.init()) {
    Serial.println("Error al iniciar sensor derecho");
    while (1);
  }
  sensorRight.setTimeout(500);

  // Configurar modo de alta precisión
  sensorFront.setMeasurementTimingBudget(200000);
  sensorLeft.setMeasurementTimingBudget(200000);
  sensorRight.setMeasurementTimingBudget(200000);

  Serial.println("Sensores VL53L0X inicializados correctamente");
}

void SensorControl::readAll() {
  // Leer distancias de los sensores
  frontDistance = sensorFront.readRangeSingleMillimeters();
  leftDistance = sensorLeft.readRangeSingleMillimeters();
  rightDistance = sensorRight.readRangeSingleMillimeters();
  
  // Filtrar valores erróneos
  if (sensorFront.timeoutOccurred() || frontDistance > 8000) frontDistance = 8000;
  if (sensorLeft.timeoutOccurred() || leftDistance > 8000) leftDistance = 8000;
  if (sensorRight.timeoutOccurred() || rightDistance > 8000) rightDistance = 8000;
}