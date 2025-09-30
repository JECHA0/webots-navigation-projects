#include "Obstacle_Avoidance.hpp"

#include <iostream>
using namespace webots;
using namespace std;
int main(int argc, char **argv) {
    OAmyRobot *robot = new OAmyRobot;
    //double ps[2]; 
    while (robot->step(TIME_STEP)!= -1) {
       robot->STATE_OA();
       //robot ->getPositions(ps);
       //robot->transition(ps);
       //robot->Forward();
       //robot->calculateCenterOfMass();
          
    
    }
    //delete robot;
    return 0;
}