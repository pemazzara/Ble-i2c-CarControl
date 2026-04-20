
#include "SpeedController.h"
#include "sonar_integration.h"
#include "MotorControl.h"

extern UltraSonicMeasure sonar;
extern MotorControl motorController;

void SpeedController::begin() {
     
    data_mutex = xSemaphoreCreateMutex();
    if (!data_mutex) {
        Serial.println("❌ SpeedController: Error creando mutex");
        return;
    }
}

void SpeedController::updateControl() {
        // 1. Medir velocidad actual con el sonar
        SonarSensorData_t sonarData;
        sonar.sonarUpdate();
        sonar.getLatestSonarData(sonarData);
        
        avel_current = sonarData.a_vel; // Δtiempo/Δt
        
        // 2. Calcular error
        float error = target_avel - avel_current;
        
        // 3. PID
        error_integral += error * Ki;
        float derivative = (error - last_error) * Kd;
        float correction = Kp * error + error_integral + derivative;
        
    
        // El valor base puede ser el último PWM exitoso
        //static int current_pwm = 300;

        // 4. Calcular PWM (partiendo de un valor base)
        if (target_avel > 0) {
            current_pwm += (int)correction;
            // Si el PID pide movimiento, forzamos que esté en el rango útil (300-1023)
            current_pwm = constrain(current_pwm, 300, 1023);
        } else {
            // Si no hay objetivo, apagamos motores y reseteamos el acumulador
            current_pwm = 0;
            error_integral = 0;
        }
        
        // 5. Aplicar a motores (con el ángulo recibido del Master)
        //motorController.setTargetAngle(base_angle);
        //motorController.update(current_pwm);
        motorController.setPWM(current_pwm, base_angle, false);
        // 6. Guardar último error para la próxima iteración       
        last_error = error;
    }
    
    int16_t SpeedController::getCurrentPWM() {
        return current_pwm;
    }
    float SpeedController::getCurrentAvel() {
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            float avel = avel_current;
            xSemaphoreGive(data_mutex);
            return avel;
        }
        return 0; // o algún valor de error
    }
    float SpeedController::getLastError() {
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            float error = last_error;
            xSemaphoreGive(data_mutex);
            return error;
        }
        return 0; // o algún valor de error
    }
    // Llamada cuando llega un comando SPI del Master
    void SpeedController::setTargetFromMaster(const ControlCommand_t& cmd) {
        float incoming_target = cmd.speed / 1023.0f; // Normalizar
    
        // Reset integral si el cambio es muy brusco (prevención de integral windup)
        if (abs(incoming_target - target_avel) > 0.2f) {
            error_integral = 0;
        }
    
        target_avel = incoming_target; 
        /* Solo actualizamos el gradiente si el robot realmente giró
        if (abs(x3_actual - x3_anterior) > 2.0) { 
            x4_gradiente = (x1_filtrada - x1_anterior) / (x3_actual - x3_anterior);
        }*/
        base_angle = cmd.angle; // Este es tu x3* que viene del Master
    }


/* Rutina de calibración
void calibratePWMtoAvel() {
    for(int pwm = 100; pwm <= 1023; pwm += 50) {
        motorController.setDrive(pwm, 90, false); // Adelante
        
        delay(2000); // Esperar a estabilizar
        
        // Medir avel (promedio de varias lecturas)
        float avel_sum = 0;
        for(int i=0; i<10; i++) {
            avel_sum += sonar.sonarApproachRateRMT(); // tu función que calcula Δtiempo/Δt
            delay(100);
        }
        float avel = avel_sum / 10;
        
        Serial.printf("PWM: %d → avel: %.3f\n", pwm, avel);
        
        // Guardar en tabla (EEPROM o constante)
        pwm_to_avel_table[pwm/50] = avel;
    }
}*/