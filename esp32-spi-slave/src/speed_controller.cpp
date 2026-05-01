
#include "speed_controller.h"
#include "sonar_integration.h"
#include "MotorControl.h"
#include "slave_fsm.h"


//extern UltraSonicMeasure sonar;
extern MotorControl motorController;
extern CalibrationParams calibParams;
extern SonarSensorData_t data;

    // Constantes del PID (ajustar experimentalmente)
    const float Kp = 0.4;    // Proporcional
    const float Ki = 0.08;    // Integral  
    const float Kd = 0.02;   // Derivativa
    int base_angle = 90;        // Ángulo deseado para ir recto (90 = adelante, <90 = izquierda, >90 = derecha)
    int current_pwm = 300;   // Valor inicial de PWM (ajustar según tu hardware)

void SpeedController::begin() {
     
    data_mutex = xSemaphoreCreateMutex();
    if (!data_mutex) {
        Serial.println("❌ SpeedController: Error creando mutex");
        return;
    }
}
// O un método unificado:
void SpeedController::setCalibration(float K, float tau) {
        // Calcular tau_deseado (puede ser fijo o configurable)
        const float tau_deseado = tau / 2.0;
        Kp = tau / (K * tau_deseado);
        Ki = (1.0 / (K * tau_deseado)) * CONTROL_PERIOD_S;  // T debe ser conocido
        K_ff = K;    
}
void SpeedController::loadCalibration(float K, float tau) {
    setCalibration(K, tau);
}
void SpeedController::updateControl() {
    
    //sonar.sonarUpdate();
    //sonar.getLastSonarData(data); // data es extern
    float avel_normalized = data.a_vel / MAX_VELOCITY_MM_S;
    float error = target_avel - avel_normalized;
    
    // Integral con anti-windup
    error_integral += error * CONTROL_PERIOD_S;
    error_integral = constrain(error_integral, -1000, 1000); // Límite anti-windup
    
    // PID
    //setCalibration(calibParams.K, calibParams.tau); // Asegurar que los parámetros estén actualizados
    float p_term = Kp * error;
    float i_term = Ki * error_integral;
    float correction = p_term + i_term;
    
    // Feedforward basado en el modelo
    float pwm_ff = target_avel / K_ff;  // PWM necesario teórico
    
    int current_pwm = (int)(pwm_ff + correction);
    current_pwm = constrain(current_pwm, 0, 1023);
    
    //motorController.applyKinematics(current_pwm, base_angle);
    motorController.setPWM(current_pwm, base_angle, false);
    
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

    void SpeedController::setTarget(float target, int angle) {
        if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            target_avel = target;
            base_angle = angle;
            xSemaphoreGive(data_mutex);
        }
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
        //float incoming_target = (float)cmd.speed;
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


