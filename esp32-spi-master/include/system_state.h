#ifndef SYSTEM_STATE_H
#define SYSTEM_STATE_H

#include <Arduino.h>
#include <functional> // <--- Agregar esto

typedef enum {
    SLAVE_STATE_IDLE = 0,
    SLAVE_STATE_CALIBRATION,
    SLAVE_STATE_READY,
    SLAVE_STATE_EMERGENCY
} SlaveState_t;

// Estados base
typedef enum : uint8_t {
    SYSTEM_STATE_BOOT = 0,
    SYSTEM_STATE_CALIBRATE = 1,
    SYSTEM_STATE_READY = 2,
    SYSTEM_STATE_EMERGENCY = 3,
    SYSTEM_STATE_ERROR = 4,
    SYSTEM_STATE_LOW_BATTERY = 5
} SystemBaseState_t;

// Subestados para READY
typedef enum : uint8_t {
    READY_SUBSTATE_IDLE = 0,
    READY_SUBSTATE_MANUAL = 1,
    READY_SUBSTATE_AUTONOMOUS = 2
} ReadySubState_t;






// Callbacks para entrada/salida de estados
//typedef void (*StateCallback)();
using StateCallback = std::function<void()>;

class SystemState {
private:
    static SystemBaseState_t currentBaseState;
    static ReadySubState_t currentReadySubState;
    static SystemBaseState_t previousBaseState;
    static ReadySubState_t previousReadySubState;
    
    static uint32_t stateStartTime;
    static SemaphoreHandle_t stateMutex;
    
    // Callbacks estáticos
    static StateCallback onEnterEmergency;
    static StateCallback onExitEmergency;
    static StateCallback onEnterReady;
    static StateCallback onExitReady;
    static StateCallback onEnterCalibrate;
    static StateCallback onExitCalibrate;
    
    // Función central de transición base
    static bool transitionToBaseState(SystemBaseState_t newState);
    
    // Función central de transición de subestado
    static bool transitionToReadySubState(ReadySubState_t newSubState);
    static uint32_t lastSyncTime;
    static const uint32_t MAX_SYNC_DELAY_MS = 100; // 100ms de tolerancia

    static const uint16_t RECOVERY_THRESHOLD = 10; // Necesitamos 10 paquetes seguidos
public:
    // Inicialización
    static void begin();
    
    // Registro de callbacks
    static void setOnEnterEmergency(StateCallback cb) { onEnterEmergency = cb; }
    static void setOnExitEmergency(StateCallback cb) { onExitEmergency = cb; }
    static void setOnEnterReady(StateCallback cb) { onEnterReady = cb; }
    static void setOnExitReady(StateCallback cb) { onExitReady = cb; }
    static void setOnEnterCalibrate(StateCallback cb) { onEnterCalibrate = cb; }
    static void setOnExitCalibrate(StateCallback cb) { onExitCalibrate = cb; }
    
    // Transiciones públicas
    static bool setBoot() { return transitionToBaseState(SYSTEM_STATE_BOOT); }
    static bool setCalibrate() { return transitionToBaseState(SYSTEM_STATE_CALIBRATE); }
    static bool setReady() { return transitionToBaseState(SYSTEM_STATE_READY); }
    static bool setEmergency() { return transitionToBaseState(SYSTEM_STATE_EMERGENCY); }
    static bool setError() { return transitionToBaseState(SYSTEM_STATE_ERROR); }
    static bool setLowBattery() { return transitionToBaseState(SYSTEM_STATE_LOW_BATTERY); }
    
    // Transiciones de subestado (solo válidas en READY)
    static bool setManualMode() { 
        return transitionToReadySubState(READY_SUBSTATE_MANUAL); 
    }
    static bool setAutonomousMode() { 
        return transitionToReadySubState(READY_SUBSTATE_AUTONOMOUS); 
    }
    static bool setIdleMode() { 
        return transitionToReadySubState(READY_SUBSTATE_IDLE); 
    }
    static bool setBaseState(SystemBaseState_t newState) { return transitionToBaseState(newState); }
    static bool setReadySubState(ReadySubState_t newSubState) { return transitionToReadySubState(newSubState); }
    // Getters
    static SystemBaseState_t getBaseState();
    static ReadySubState_t getReadySubState();
    
    // Consultas de estado
    static bool isReady() { return getBaseState() == SYSTEM_STATE_READY; }
    static bool isManual() { 
        return isReady() && getReadySubState() == READY_SUBSTATE_MANUAL; 
    }
    static bool isAutonomous() { 
        return isReady() && getReadySubState() == READY_SUBSTATE_AUTONOMOUS; 
    }
    static bool isEmergency() { return getBaseState() == SYSTEM_STATE_EMERGENCY; }
    
    static uint32_t getTimeInCurrentState();
    static const char* baseStateToString();
    static const char* readySubStateToString();

    static void notifySyncSuccess();      // Se llama cada vez que un paquete SPI es válido
    static void checkHealth();            // Se llama en el loop o tarea de lógica
    static bool isCommActive();           // Consulta rápida de salud de red
    // Helper para depuración
    static void printStateTransition();
};

#endif
