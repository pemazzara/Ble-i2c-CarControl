// ObstacleAvoidance.cpp
#include "ObstacleAvoidance.h"
#include <algorithm>

ObstacleAvoidance::ObstacleAvoidance() 
    : currentStrategy(SIMPLE_TURN),
      MIN_SAFE_DISTANCE(150),
      PREFERRED_DISTANCE(300),
      TURN_THRESHOLD(50),
      turnSpeed(600),
      baseSpeed(400) {}

ControlOutput ObstacleAvoidance::calculateCommand(uint16_t sonarDist, uint16_t tofFront,
                                                 uint16_t tofLeft, uint16_t tofRight) {
    switch(currentStrategy) {
        case SIMPLE_TURN:
            return simpleTurn(tofFront, tofLeft, tofRight);
        case BUG_ALGORITHM:
            return bugAlgorithm(tofFront, tofLeft, tofRight);
        case WALL_FOLLOWING:
            return wallFollowing(tofFront, tofLeft, tofRight);
        default:
            return simpleTurn(tofFront, tofLeft, tofRight);
    }
}

ControlOutput ObstacleAvoidance::simpleTurn(uint16_t front, uint16_t left, uint16_t right) {
    ControlOutput output;
    output.priority = 2;  // Prioridad normal
    
    uint16_t effectiveFront = front;  // Podríamos combinar con sonar aquí
    
    if (effectiveFront < MIN_SAFE_DISTANCE) {
        // ¡Obstáculo muy cerca! Parada de emergencia
        output.leftSpeed = 0;
        output.rightSpeed = 0;
        output.priority = 9;  // Alta prioridad (como STOP)
    }
    else if (effectiveFront < PREFERRED_DISTANCE) {
        // Necesitamos evadir
        if (left > right + TURN_THRESHOLD) {
            // Más espacio a la izquierda
            output.leftSpeed = -turnSpeed;
            output.rightSpeed = turnSpeed;  // Giro izquierda (diferencial)
        }
        else if (right > left + TURN_THRESHOLD) {
            // Más espacio a la derecha
            output.leftSpeed = turnSpeed;
            output.rightSpeed = -turnSpeed;  // Giro derecha
        }
        else {
            // Igual de cerca por ambos lados, retroceder
            output.leftSpeed = -baseSpeed;
            output.rightSpeed = -baseSpeed;  // Retroceso
        }
    }
    else {
        // Camino despejado
        output.leftSpeed = baseSpeed;
        output.rightSpeed = baseSpeed;  // Avanzar
        
        // Velocidad proporcional a la distancia
        if (effectiveFront > 600) {
            output.leftSpeed = 850;
            output.rightSpeed = 850;
        }
    }
    
    return output;
}

ControlOutput ObstacleAvoidance::bugAlgorithm(uint16_t front, uint16_t left, uint16_t right) {
    // Implementación simplificada del algoritmo Bug0
    ControlOutput output;
    output.priority = 2;
    
    static enum { GOING_TO_GOAL, FOLLOWING_WALL } state = GOING_TO_GOAL;
    
    if (front < MIN_SAFE_DISTANCE) {
        state = FOLLOWING_WALL;
    }
    
    if (state == GOING_TO_GOAL) {
        output.leftSpeed = baseSpeed;
        output.rightSpeed = baseSpeed;
    } else {
        // Seguir pared (simple: girar a la izquierda)
        output.leftSpeed = -turnSpeed/2;
        output.rightSpeed = turnSpeed/2;
        
        // Si encontramos espacio al frente, volver a goal
        if (front > PREFERRED_DISTANCE * 1.5) {
            state = GOING_TO_GOAL;
        }
    }
    
    return output;
}

ControlOutput ObstacleAvoidance::wallFollowing(uint16_t front, uint16_t left, uint16_t right) {
    ControlOutput output;
    output.priority = 2;
    
    // Mantener distancia constante a la pared izquierda
    const uint16_t DESIRED_WALL_DIST = 200;
    
    if (front < MIN_SAFE_DISTANCE) {
        // Obstáculo frontal, girar
        output.leftSpeed = -turnSpeed;
        output.rightSpeed = turnSpeed;
    } else {
        // Seguir pared izquierda
        int16_t error = DESIRED_WALL_DIST - left;
        int16_t correction = error * 2;  // Ganancia simple
        
        output.leftSpeed = baseSpeed - correction;
        output.rightSpeed = baseSpeed + correction;
        
        // Limitar velocidades
        output.leftSpeed = std::max(-1023, std::min(1023, (int)output.leftSpeed));
        output.rightSpeed = std::max(-1023, std::min(1023, (int)output.rightSpeed));
    }
    
    return output;
}

uint16_t ObstacleAvoidance::getEffectiveFront(uint16_t tofFront, uint16_t sonar) {
    return std::min(tofFront, sonar);
}