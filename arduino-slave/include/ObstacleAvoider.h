// ObstacleAvoider.h
#ifndef OBSTACLE_AVOIDER_H
#define OBSTACLE_AVOIDER_H

#include <NewPing.h>
#include "MotorController.h"  // Incluir MotorController

class ObstacleAvoider {
private:
    NewPing* sonar;
    int trigPin, echoPin;
    MotorController* motorController;  // Puntero a MotorController
    int minDistance = 15;
    bool isActive = false;
    
public:
    // Recibe MotorController por referencia
    ObstacleAvoider(int trig, int echo, MotorController& motorCtrl);
    void begin();
    void setActive(bool active);
    void update();
    int getDistance();
    
private:
    void avoidObstacle();
    int scanLeft();
    int scanRight();
};

#endif