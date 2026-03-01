// Esp32Slave - MotorControl.h
/*El archivo MotorControl.h declara la interfaz y los parámetros de bajo nivel
 para controlar un par de motores con un L298N desde un ESP32 en modo esclavo 
 I2C. Empieza con las guardas de inclusión y las dependencias esenciales 
 (<Arduino.h>, <Wire.h>), luego define direcciones y pines (dirección I2C, pines 
 ENA/ENB/IN1..IN4 y pines SDA/SCL). También incluye macros para comandos I2C: 
 tanto comandos de movimiento y parada como comandos de configuración 
 (velocidades, estado del sonar). Una observación práctica: hay una mezcla 
 de literales tipo carácter ('1','2','3') y valores hexadecimales para 
 comandos; mantener un estilo consistente facilita la lectura.

La clase MotorControl encapsula el estado y comportamiento del controlador 
de motores. En el ámbito privado se definen constantes y variables de estado: 
MAX_ACCELERATION para limitar aceleraciones bruscas, currentLeftSpeed/currentRightSpeed 
para los valores PWM actuales, manualSpeed/autoSpeed como límites por defecto,
 emergencyStopActive para la parada de emergencia y lastCommandTime/SAFETY_TIMEOUT 
 para la lógica de seguridad por tiempo. Esto sugiere que la clase aplica smoothing de velocidad y tiene un temporizador de seguridad que detiene los motores si no llegan comandos en un intervalo determinado.

Las funciones privadas declaradas describen responsabilidades claras: 
moveForward/moveBackward/turnLeft/turnRight son las acciones básicas; 
executeManualCommand interpreta comandos manuales; setMotorSpeeds y 
smoothSpeed aplican los cambios reales y la limitación de aceleración; 
resetSafetyTimer e isSafetyTimeout gestionan el temporizador interno. 
En la interfaz pública están el constructor, begin para inicialización 
del hardware, handleCommand para recibir e interpretar comandos 
I2C (probablemente llamado desde la tarea de motor), emergencyStop y stop 
para detener los motores, updateSafety para comprobar el timeout y 
getMotorStatus para exponer estado a quien lo solicite (por ejemplo el ISR I2C).
Puntos a tener en cuenta y recomendaciones: 
1) Concurrencia: el objeto MotorControl se usa desde la tarea de motor y 
desde otras tareas (sensorTask llama a emergencyStop) y además getMotorStatus 
se invoca desde el handler I2C (ISR). Asegúrate de que los métodos usados 
desde ISR o desde otras tareas sean seguros (no usen llamadas no reentrantes 
ni APIs bloqueantes) o protege el acceso a variables compartidas 
(mutexes o variables atómicas) para evitar carreras. 
2) Constantes: en C++ es preferible usar constexpr o static constexpr para 
constantes en clases en vez de const int no estático, y uniformizar la 
definición de comandos para evitar confusión. 3) Inicialización del 
temporizador: si begin() no llama a resetSafetyTimer(), lastCommandTime 
quedará en 0 y el timeout podría dispararse inmediatamente; garantiza que 
begin() inicialice el temporizador. En general la cabecera está bien 
organizada y delega la lógica real a la implementación (.cpp), pero 
conviene revisar la seguridad ante ISR/FreeRTOS y la consistencia en las 
definiciones.*/
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include "Command.h"

// Pines L298N
const int ENA = 13;   // PWM motor izquierdo
const int IN1 = 9;   
const int IN2 = 10;   
const int ENB = 14;   // PWM motor derecho  
const int IN3 = 11;   
const int IN4 = 12;

// Configuración PWM
#define MOTOR_PWM_FREQ      5000  // 5 kHz de frecuencia
#define MOTOR_PWM_RES       10    // 10 bits de resolución (0-1023)
#define MAX_DUTY_CYCLE      1023  // 2^10 - 1 = 1023

// Canales LEDC - Deben ser únicos y usar el rango 0-7 para Arduino.
#define LEDC_CH_A           0     // Canal 0 para Motor Izquierdo
#define LEDC_CH_B           1     // Canal 1 para Motor Derecho






// =========================================================
// CLASE MOTOR CONTROLLER (Adaptada de MotorController.cpp)
// =========================================================

class MotorControl {
private:
// ✅ AGREGAR protección contra cambios bruscos
    const int MAX_ACCELERATION = 50; // Incremento máximo por ciclo
    int currentLeftSpeed = 0;
    int currentRightSpeed = 0;
    int manualSpeed = 150;
    int autoSpeed = 120;
    bool emergencyStopActive = false;
    unsigned long lastCommandTime = 0;
    const unsigned long SAFETY_TIMEOUT = 1000; // 1 segundo
    
    // Funciones internas
    void moveForward(int speed);
    void moveBackward(int speed);
    void turnLeft(int speed);
    void turnRight(int speed);
    void executeManualCommand(byte command);
    void resetSafetyTimer() { lastCommandTime = millis(); }
    bool isSafetyTimeout() { return (millis() - lastCommandTime) > SAFETY_TIMEOUT; }
    void setMotorSpeeds(int leftSpeed, int rightSpeed);
    int smoothSpeed(int current, int target);
public:
    MotorControl();
    void begin(); 
    void handleCommand(byte command, byte data = 0);
    void emergencyStop();
    void resetEmergency();
    void stop();
    void updateSafety();
    uint8_t getMotorStatus();
};
#endif  