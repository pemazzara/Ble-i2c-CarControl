// system_state.cpp
#include "system_state.h"

// Inicialización de miembros estáticos
SystemBaseState_t SystemState::currentBaseState = SYSTEM_STATE_BOOT;
ReadySubState_t SystemState::currentReadySubState = READY_SUBSTATE_IDLE;
SystemBaseState_t SystemState::previousBaseState = SYSTEM_STATE_BOOT;
ReadySubState_t SystemState::previousReadySubState = READY_SUBSTATE_IDLE;

static uint16_t syncSuccessCount;
uint32_t SystemState::stateStartTime = 0;
SemaphoreHandle_t SystemState::stateMutex = NULL;

// Inicialización de Callbacks
StateCallback SystemState::onEnterEmergency = nullptr;
StateCallback SystemState::onExitEmergency = nullptr;
StateCallback SystemState::onEnterReady = nullptr;
StateCallback SystemState::onExitReady = nullptr;
StateCallback SystemState::onEnterCalibrate = nullptr;
StateCallback SystemState::onExitCalibrate = nullptr;

void SystemState::begin() {
    if (stateMutex == NULL) {
        stateMutex = xSemaphoreCreateMutex();
    }
    syncSuccessCount = 0;
    stateStartTime = millis();
}

// --- Gestión de Transiciones ---

bool SystemState::transitionToBaseState(SystemBaseState_t newState) {
    if (stateMutex == NULL || xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false; 
    }

    if (currentBaseState == newState) {
        xSemaphoreGive(stateMutex);
        return true;
    }

    // 1. Callbacks de SALIDA
    if (currentBaseState == SYSTEM_STATE_READY && onExitReady) onExitReady();
    if (currentBaseState == SYSTEM_STATE_EMERGENCY && onExitEmergency) onExitEmergency();
    if (currentBaseState == SYSTEM_STATE_CALIBRATE && onExitCalibrate) onExitCalibrate();

    // 2. Actualización de estado
    previousBaseState = currentBaseState;
    currentBaseState = newState;
    stateStartTime = millis();

    // Reset de subestado si salimos de READY para evitar comportamientos fantasma
    if (newState != SYSTEM_STATE_READY) {
        currentReadySubState = READY_SUBSTATE_IDLE;
    }

    // 3. Callbacks de ENTRADA
    if (newState == SYSTEM_STATE_READY && onEnterReady) onEnterReady();
    if (newState == SYSTEM_STATE_EMERGENCY && onEnterEmergency) onEnterEmergency();
    if (newState == SYSTEM_STATE_CALIBRATE && onEnterCalibrate) onEnterCalibrate();

    xSemaphoreGive(stateMutex);
    printStateTransition();
    return true;
}

bool SystemState::transitionToReadySubState(ReadySubState_t newSubState) {
    if (stateMutex == NULL || xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        return false;
    }

    // Solo permitimos cambiar subestado si estamos en READY
    if (currentBaseState != SYSTEM_STATE_READY) {
        xSemaphoreGive(stateMutex);
        return false;
    }

    previousReadySubState = currentReadySubState;
    currentReadySubState = newSubState;
    xSemaphoreGive(stateMutex);
    return true;
}

// --- Getters Thread-Safe ---

SystemBaseState_t SystemState::getBaseState() {
    SystemBaseState_t state = SYSTEM_STATE_ERROR;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        state = currentBaseState;
        xSemaphoreGive(stateMutex);
    }
    return state;
}

ReadySubState_t SystemState::getReadySubState() {
    ReadySubState_t sub = READY_SUBSTATE_IDLE;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        sub = currentReadySubState;
        xSemaphoreGive(stateMutex);
    }
    return sub;
}

uint32_t SystemState::getTimeInCurrentState() {
    return millis() - stateStartTime;
}

// --- Helpers de Depuración ---

const char* SystemState::baseStateToString() {
    switch (getBaseState()) {
        case SYSTEM_STATE_BOOT:      return "BOOT";
        case SYSTEM_STATE_CALIBRATE: return "CALIBRATE";
        case SYSTEM_STATE_READY:     return "READY";
        case SYSTEM_STATE_EMERGENCY: return "EMERGENCY";
        case SYSTEM_STATE_LOW_BATTERY: return "LOW_BATTERY";
        default:                     return "ERROR";
    }
}

void SystemState::printStateTransition() {
    Serial.printf("[FSM] Transition: %s -> %s\n", 
                  "PREV", // Podrías mapear previousBaseState a string también
                  baseStateToString());
}

// Inicialización del miembro estático
uint32_t SystemState::lastSyncTime = 0;


bool SystemState::isCommActive() {
    return (millis() - lastSyncTime < MAX_SYNC_DELAY_MS);
}

void SystemState::notifySyncSuccess() {
    lastSyncTime = millis();
    if (syncSuccessCount < RECOVERY_THRESHOLD) {
        syncSuccessCount++;
    }
}

void SystemState::checkHealth() {
    uint32_t now = millis();
    SystemBaseState_t current = getBaseState();

    // ESCENARIO A: Pérdida de comunicación
    if (current != SYSTEM_STATE_ERROR && current != SYSTEM_STATE_BOOT) {
        if (now - lastSyncTime > MAX_SYNC_DELAY_MS) {
            syncSuccessCount = 0; // Resetear contador de estabilidad
            Serial.println("[FSM] ERROR: Comm lost.");
            transitionToBaseState(SYSTEM_STATE_ERROR);
            return;
        }
    }

    // ESCENARIO B: Intento de Recuperación
    if (current == SYSTEM_STATE_ERROR) {
        // Si la comunicación ha vuelto y es estable
        if (isCommActive() && syncSuccessCount >= RECOVERY_THRESHOLD) {
            Serial.println("[FSM] INFO: Comm restored. Re-calibrating...");
            
            // IMPORTANTE: Nunca saltar a READY directamente.
            // Volvemos a CALIBRATE o BOOT para asegurar que el hardware esté OK.
            transitionToBaseState(SYSTEM_STATE_CALIBRATE);
        }
    }
}
