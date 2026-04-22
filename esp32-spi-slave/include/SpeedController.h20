#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

/* 
* Esta implementación sigue un patrón de diseño profesional 
* (separación de responsabilidades entre sensado, control y actuadores). 
* Estamos usando un controlador de velocidad incremental, 
* lo cual es excelente para evitar saltos bruscos en los motores.
*/

#include <Arduino.h>
#include "SPIDefinitions.h"

class SpeedController {
private:
    float target_avel = 0;      // Velocidad adimensional deseada
    float new_target = 0;       // Para detectar cambios bruscos
    float avel_current = 0;     // Velocidad actual medida
    float error_integral = 0;
    float last_error = 0;
    SemaphoreHandle_t data_mutex;
    // Constantes del PID (ajustar experimentalmente)
    const float Kp = 0.4;    // Proporcional
    const float Ki = 0.08;    // Integral  
    const float Kd = 0.02;   // Derivativa
    int base_angle = 90;        // Ángulo deseado para ir recto (90 = adelante, <90 = izquierda, >90 = derecha)
    int current_pwm = 300;   // Valor inicial de PWM (ajustar según tu hardware)
public:
    void begin();
    float getCurrentAvel();
    int16_t getCurrentPWM();
    float getLastError();
    // Llamada desde la tarea de control del Slave (ej. 50Hz)
    void updateControl();
    // Llamada cuando llega un comando SPI del Master
    void setTargetFromMaster(const ControlCommand_t& cmd); 
    //int pwmFromAvel(float target_avel);
    //uint8_t getStatus();
};

#endif