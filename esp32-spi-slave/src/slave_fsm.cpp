#include "slave_fsm.h"
#include "motor_controller.h"   // Para control de motores
#include "sonar_integration.h" // para obtener distancia
#include "speed_controller.h"    // Para control PID de velocidad|
#include "Preferences.h"         // Para almacenamiento en EEPROM
#include "spi_protocol.h"       // Para las estructuras de datos de comunicación

void SlaveFSM::onExitIdle() {}
void SlaveFSM::onExitReady() {
    motorController->emergencyStop();
    movingFlag = false;
}
void SlaveFSM::onExitEmergency() { /* Opcional: Log o limpieza */ }
void SlaveFSM::onExitCalibration() {
    calibrationRunning = false;
    // Detener motores al salir de calibración
    motorController->setPWM(0, 90, true);
    // Guardar calibración en EEPROM
    saveCalibrationToEEPROM();
}

// Singleton instance
SlaveFSM& SlaveFSM::getInstance() {
    static SlaveFSM instance;
    return instance;
}
    
void SlaveFSM::begin() {
    fsmMutex = xSemaphoreCreateMutex();
    currentState = SLAVE_STATE_IDLE;
    previousState = SLAVE_STATE_IDLE;
    stateStartTime = millis();
    calibrationRunning = false;
    calibParams.valid = false;
    lastDistance = 0;
    emergencyFlag = false;
    errorFlag = false;
    movingFlag = false;
    statusFlags = 0;
    loadCalibrationFromEEPROM();
    if (calibParams.valid) {
    Serial.println("Calibración previa cargada");
    } else {
    Serial.println("Sin calibración válida, se requerirá calibración");
    }
    Serial.println("🔧 SlaveFSM inicializada en IDLE");
}

void SlaveFSM::update() {
    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) != pdTRUE) return; 
    statusFlags = motorController->getMotorStatus();
    // Actualizar flags de estado 
    if (emergencyFlag) statusFlags |= 0x01;
    if (movingFlag)    statusFlags |= 0x02;
    if (safetyTimeout) statusFlags |= 0x04;  // definido externamente
    if(errorFlag)      statusFlags |= 0x08;     // definido externamente

    // Máquina de estados
    switch (currentState) {
        case SLAVE_STATE_IDLE:
            // No hacer nada, esperar comandos
            break;

        case SLAVE_STATE_CALIBRATION:
            updateCalibration();
            break;

        case SLAVE_STATE_READY:
            // Leer sensores y actualizar datos de estado
            //lastDistance = sonar->sonarApproachRateRMT(); // distancia actual
            SonarSensorData_t data;
            if(sonar->getLatestSonarData(data)){
                lastDistance = data.distance;
                emergencyFlag = data.emergency;
                movingFlag = data.a_vel > 0.05f; // umbral de movimiento
                errorFlag = !data.sensor_ok;
            }; // Actualizar datos del sonar
            statusFlags = motorController->getMotorStatus();
            speedController.updateControl();
            // En READY, solo monitorear emergencias
            if (lastDistance < DISTANCIA_CRITICA_STOP) {
                transitionTo(SLAVE_STATE_EMERGENCY);
            }
            break;

        case SLAVE_STATE_EMERGENCY:
            // En emergencia, asegurar motores parados
            motorController->emergencyStop();
            break;
    }

    xSemaphoreGive(fsmMutex);
}

void SlaveFSM::handleCommand(uint8_t commandType, int16_t speed, int16_t angle) {
    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) != pdTRUE) return;

    switch (commandType) {
        case CMD_START_CALIB:
            if (currentState == SLAVE_STATE_IDLE) {
                transitionTo(SLAVE_STATE_CALIBRATION);
            }
            break;

        case CMD_ACTIVE:
            if (currentState == SLAVE_STATE_READY && calibParams.valid) {
                // Ya está en READY, no necesita transición
            } else if (currentState == SLAVE_STATE_IDLE && calibParams.valid) {
                transitionTo(SLAVE_STATE_READY);
            }
            break;

        case CMD_IDLE_TRANSITION:
            if (currentState == SLAVE_STATE_READY) {
                transitionTo(SLAVE_STATE_IDLE);
            }
            break;

        case CMD_EMERGENCY:
            // 1. Hardware primero: Frenar en seco
            motorController->setPWM(0, 90, true); 
            movingFlag = false;
            // 2. Lógica después: Cambiar estado
            transitionTo(SLAVE_STATE_EMERGENCY);
            break;

        case CMD_MOTOR_DRIVE:
            // Solo permitimos movimiento si estamos sanos y calibrados
            if (currentState == SLAVE_STATE_READY && calibParams.valid) {                   
                // Establecer referencia de velocidad (adimensional 0-1)
                float targetAvel = speed / 1023.0f;
                speedController.setTarget(targetAvel, angle);
                movingFlag = (speed != 0);
            }
            break;

        case CMD_INMEDIATE_STOP:
            motorController->setPWM(0, 90, true);  
            movingFlag = false;
            break;
    }

    xSemaphoreGive(fsmMutex);
}

void SlaveFSM::transitionTo(SlaveState_t newState) {
    if (currentState == newState) return;

    // Llamar callbacks de salida del estado actual
    switch (currentState) {
        case SLAVE_STATE_IDLE:       onExitIdle(); break;
        case SLAVE_STATE_CALIBRATION: onExitCalibration(); break;
        case SLAVE_STATE_READY:      onExitReady(); break;
        case SLAVE_STATE_EMERGENCY:  onExitEmergency(); break;
    }

    // Cambiar estado
    previousState = currentState;
    currentState = newState;
    stateStartTime = millis();

    // Llamar callbacks de entrada al nuevo estado
    switch (newState) {
        case SLAVE_STATE_IDLE:       onEnterIdle(); break;
        case SLAVE_STATE_CALIBRATION: onEnterCalibration(); break;
        case SLAVE_STATE_READY:      onEnterReady(); break;
        case SLAVE_STATE_EMERGENCY:  onEnterEmergency(); break;
    }

    Serial.printf("🔄 SlaveFSM: %d → %d\n", previousState, currentState);
}

void SlaveFSM::onEnterIdle() {
    motorController->emergencyStop();
    movingFlag = false;
    emergencyFlag = false;
    Serial.println("Slave: IDLE");
}

void SlaveFSM::onEnterCalibration() {
    calibrationRunning = true;
    calibrationProgress = 0;
    calibrationIndex = 0;
    calibrationStepStart = millis();
    motorController->setPWM(CALIB_PWM, 90, true);   // PWM fijo para avanzar
    Serial.println("Slave: Iniciando CALIBRACIÓN");
}

void SlaveFSM::updateCalibration() {
    static enum { MOVING, ANALYZING, DONE } step = MOVING;

    switch (step) {
        case MOVING:
            if (millis() - calibrationStepStart < 2000) {
                if (calibrationIndex < 100) {
                    calibrationMeasurements[calibrationIndex++] = sonar->sonarApproachRateRMT();
                    calibrationProgress = 10 + (calibrationIndex * 40 / 100);
                }
            } else {
                motorController->emergencyStop();
                step = ANALYZING;
                calibrationStepStart = millis();
            }
            break;

        case ANALYZING:
            if (millis() - calibrationStepStart > 500) {
                calculateCalibrationParams();
                step = DONE;
                calibrationProgress = 90;
                calibrationStepStart = millis();
            }
            break;

        case DONE:
            if (millis() - calibrationStepStart > 1000) {
                transitionTo(SLAVE_STATE_READY);
                step = MOVING;
            }
            break;
    }
}

void SlaveFSM::calculateCalibrationParams() {
    // Calcular K y τ a partir de las mediciones
    float avel_final = calibrationMeasurements[99];
    calibParams.K = avel_final / CALIB_PWM;  // 400 fue el PWM usado
    calibParams.valid = true;

    // Buscar τ: tiempo para alcanzar 63% de avel_final
    float target = 0.63f * avel_final;
    int tau_samples = 0;
    for (int i = 0; i < 100; i++) {
        if (calibrationMeasurements[i] >= target) {
            tau_samples = i;
            break;
        }
    }
    calibParams.tau = tau_samples * 0.02f;  // 20ms por muestra
}

void SlaveFSM::onEnterReady() {
    motorController->emergencyStop();  // Asegurar parada inicial
    movingFlag = false;
    Serial.println("Slave: READY (listo para recibir comandos)");
}

void SlaveFSM::onEnterEmergency() {
    motorController->emergencyStop();
    emergencyFlag = true;
    movingFlag = false;
    Serial.println("🚨 Slave: EMERGENCIA activada");
}
void SlaveFSM::setEmergencyFlag() {
    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        emergencyFlag = true;
        xSemaphoreGive(fsmMutex);
    }
}
void SlaveFSM::setIdleFlag() {
    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        movingFlag = false;
        xSemaphoreGive(fsmMutex);
    }
}
void SlaveFSM::setMotorController(MotorControl* controller) {
    this->motorController = controller;
}
void SlaveFSM::setSonar(UltraSonicMeasure* sonar) {
    this->sonar = sonar;
}
void SlaveFSM::setSpeedController(SpeedController* speedCtrl) {
    this->speedController = *speedCtrl;
}

// Getters (thread-safe)
SlaveState_t SlaveFSM::getCurrentState() const {
    SlaveState_t state;
    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        state = currentState;
        xSemaphoreGive(fsmMutex);
    } else {
        state = SLAVE_STATE_EMERGENCY;  // Valor por defecto en caso de error
    }
    return state;
}


uint8_t SlaveFSM::getProgressOrFlags() const {
    // Si estamos en medio de una calibración, devolvemos el %
    if (currentState == SLAVE_STATE_CALIBRATION) {
        return calibrationProgress;
    } 
    
    // Si no, devolvemos los bits de estado (flags)
    uint8_t flags = 0;
    if (emergencyFlag)  flags |= 0x01; // Bit 0: Emergencia
    if (movingFlag)     flags |= 0x02; // Bit 1: En movimiento
    if (calibParams.valid) flags |= 0x04; // Bit 2: Calibración válida cargada
    
    return flags;
}
uint8_t SlaveFSM::getProgress() const {
    uint8_t prog = 0;
    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        prog = calibrationProgress;
        xSemaphoreGive(fsmMutex);
    }
    return prog;
}

uint8_t SlaveFSM::getStatusFlags() const {
    uint8_t flags = 0;
    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        flags = statusFlags;
        xSemaphoreGive(fsmMutex);
    }
    return flags;
}


void SlaveFSM::saveCalibrationToEEPROM() {
    Preferences prefs;
    calibParams = getCalibrationParams();  // actualizar estructura global
    prefs.begin("calib", false);
    prefs.putFloat("K", calibParams.K);
    prefs.putFloat("tau", calibParams.tau);
    prefs.end();
}
void SlaveFSM::loadCalibrationFromEEPROM() {
    Preferences prefs;
    prefs.begin("calib", false);
    calibParams.K = prefs.getFloat("K", 0.0);
    calibParams.tau = prefs.getFloat("tau", 0.0);
    calibParams.valid = (calibParams.K > 0 && calibParams.tau > 0);
    prefs.end();
}   

CalibrationParams SlaveFSM::getCalibrationParams() const {
    CalibrationParams params = {0, 0, false};
    if (xSemaphoreTake(fsmMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        params = calibParams;
        xSemaphoreGive(fsmMutex);
    }
    return params;
}

uint16_t SlaveFSM::getDistance() const {
    return lastDistance;  // Asumiendo que lastDistance se actualiza en update()
}

bool SlaveFSM::isEmergency() const {
    return (currentState == SLAVE_STATE_EMERGENCY) || emergencyFlag;
}

bool SlaveFSM::isMoving() const {
    return movingFlag;
}


