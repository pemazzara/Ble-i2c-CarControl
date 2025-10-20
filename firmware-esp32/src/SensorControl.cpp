#include "SensorControl.h"


void SensorControl::begin() {
  Serial.println("Inicializando sensores VL53L0X con multiplexación XSHUT...");
  
  // Inicializar bus I2C
  i2cBus = &Wire1;
  i2cBus->begin(I2C_SENSORES_SDA_PIN, I2C_SENSORES_SCL_PIN);
  
  // Configurar pines XSHUT
  pinMode(FRONT_XSHUT_PIN, OUTPUT);
  pinMode(LEFT_XSHUT_PIN, OUTPUT);
  pinMode(RIGHT_XSHUT_PIN, OUTPUT);
  
  // Apagar todos inicialmente
  disableAllSensors();
  
  Serial.println("Sensores listos para multiplexación XSHUT");
}

void SensorControl::disableAllSensors() {
  digitalWrite(FRONT_XSHUT_PIN, LOW);
  digitalWrite(LEFT_XSHUT_PIN, LOW);
  digitalWrite(RIGHT_XSHUT_PIN, LOW);
  delay(10);
}

void SensorControl::enableSensor(uint8_t sensorIndex) {
  disableAllSensors();
  
  switch(sensorIndex) {
    case 0: // Frontal
      digitalWrite(FRONT_XSHUT_PIN, HIGH);
      break;
    case 1: // Izquierdo
      digitalWrite(LEFT_XSHUT_PIN, HIGH);
      break;
    case 2: // Derecho
      digitalWrite(RIGHT_XSHUT_PIN, HIGH);
      break;
  }
  
  delay(10); // Esperar a que el sensor se active
  
  // Re-inicializar el sensor con dirección por defecto
  sensor.setBus(i2cBus);
  if (!sensor.init()) {
    Serial.printf("❌ Error iniciando sensor %d\n", sensorIndex);
    return;
  }
  sensor.setAddress(DEFAULT_ADDRESS);
  sensor.setTimeout(500);
  sensor.startContinuous();
}

void SensorControl::readSensor(uint8_t sensorIndex) {
  enableSensor(sensorIndex);
  delay(5);
  
  uint16_t distance = sensor.readRangeContinuousMillimeters();
  
  if (sensor.timeoutOccurred()) {
    distance = 1200;
  } else if (distance > 1200) {
    distance = 1200;
  }
  
  switch(sensorIndex) {
    case 0: frontDistance = distance; break;
    case 1: leftDistance = distance; break;
    case 2: rightDistance = distance; break;
  }
}


void SensorControl::readAll() {
  // Leer cada sensor secuencialmente
  readSensor(0); // Frontal
  delay(5); // Pequeña pausa entre lecturas
  
  readSensor(1); // Izquierdo
  delay(5);
  
  readSensor(2); // Derecho
}

void SensorControl::printDistances() {
  Serial.printf("📍 Sensores - F: %dmm, L: %dmm, R: %dmm\n", 
                frontDistance, leftDistance, rightDistance);
}

void SensorControl::diagnoseSensors() {
  Serial.println("\n=== DIAGNÓSTICO SENSORES VL53L0X ===");
  
  for (int i = 0; i < 3; i++) {
    Serial.printf("\nProbando sensor %d...\n", i);
    enableSensor(i);
    
    // Test de comunicación I2C
    i2cBus->beginTransmission(DEFAULT_ADDRESS);
    byte error = i2cBus->endTransmission();
    
    if (error == 0) {
      Serial.printf("✅ Sensor %d responde en I2C\n", i);
      
      // Test de lectura
      uint16_t testDist = sensor.readRangeSingleMillimeters();
      if (!sensor.timeoutOccurred() && testDist < 1200) {
        Serial.printf("✅ Sensor %d lectura OK: %dmm\n", i, testDist);
      } else {
        Serial.printf("❌ Sensor %d error en lectura\n", i);
      }
    } else {
      Serial.printf("❌ Sensor %d sin respuesta I2C (error: %d)\n", i, error);
    }
    
    delay(100);
  }
  Serial.println("====================================\n");
}