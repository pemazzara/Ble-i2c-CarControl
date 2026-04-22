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

#define CONTROL_PERIOD_MS 20
#define CONTROL_PERIOD_S (CONTROL_PERIOD_MS / 1000.0f)
#define MAX_VELOCITY_MM_S 50

class SpeedController {
private:
    float target_avel = 0;      // Velocidad adimensional deseada
    float new_target = 0;       // Para detectar cambios bruscos
    float avel_current = 0;     // Velocidad actual medida
    float error_integral = 0;
    float last_error = 0;
    SemaphoreHandle_t data_mutex;
    // Constantes del PID (ajustar experimentalmente)
    float Kp = 0.4;    // Proporcional
    float Ki = 0.08;    // Integral  
    float Kd = 0.02;   // Derivativa
    float K = 0.3; // Ganancia feedforward (ajustar // Valor de calibración (ajustar según necesario)
    float tau = 0.5; // Constante de tiempo (ajustar // Valor
    int base_angle = 90;        // Ángulo deseado para ir recto (90 = adelante, <90 = izquierda, >90 = derecha)
    int current_pwm = 300;   // Valor inicial de PWM (ajustar según tu hardware)
    void loadCalibration(float K, float tau);
public:
    void begin();
    float getCurrentAvel();
    int16_t getCurrentPWM();
    float K_ff;
    float getLastError();
    // Llamada desde la tarea de control del Slave (ej. 50Hz)
    void updateControl();
    void setGains(float kp, float ki) { Kp = kp; Ki = ki; }
    // Llamada cuando llega un comando SPI del Master
    void setTargetFromMaster(const ControlCommand_t& cmd);
    void setTarget(float target, int angle);
    void setCalibration(float K, float tau); 
    void setFeedforwardGain(float k) {
        K_ff = k;
    }
    
};
    //int pwmFromAvel(float target_avel);
    //uint8_t getStatus();

#endif