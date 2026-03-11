// ObstacleAvoidance.h
#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <cstdint>

struct ControlOutput {
    int16_t leftSpeed;   // -1023 a 1023
    int16_t rightSpeed;  // -1023 a 1023
    uint8_t priority;    // Para comandos especiales (ej. STOP con prioridad alta)
};

class ObstacleAvoidance {
public:
    ObstacleAvoidance();
    
    // Estrategias disponibles
    enum Strategy {
        SIMPLE_TURN = 0,
        BUG_ALGORITHM = 1,
        WALL_FOLLOWING = 2
    };
    
    // Método principal que recibe distancias y devuelve velocidades para cada motor
    ControlOutput calculateCommand(uint16_t sonarDist, uint16_t tofFront,
                                   uint16_t tofLeft, uint16_t tofRight);
    
    // Cambiar estrategia
    void setStrategy(Strategy strat) { currentStrategy = strat; }
    
    // Parámetros configurables
    void setSafetyDistance(uint16_t distance) { MIN_SAFE_DISTANCE = distance; }
    void setTurnSpeed(int16_t speed) { turnSpeed = speed; }

private:
    Strategy currentStrategy;
    uint16_t MIN_SAFE_DISTANCE;
    uint16_t PREFERRED_DISTANCE;
    uint16_t TURN_THRESHOLD;
    int16_t turnSpeed;
    int16_t baseSpeed;
    
    // Implementaciones específicas
    ControlOutput simpleTurn(uint16_t front, uint16_t left, uint16_t right);
    ControlOutput bugAlgorithm(uint16_t front, uint16_t left, uint16_t right);
    ControlOutput wallFollowing(uint16_t front, uint16_t left, uint16_t right);
    
    // Utilidades
    uint16_t getEffectiveFront(uint16_t tofFront, uint16_t sonar);
};
#endif