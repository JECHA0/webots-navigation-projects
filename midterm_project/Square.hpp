#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <complex>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <random>

#define MAX_SPEED 6.28      // rad/s
#define TIME_STEP 64        // ms
#define FORWARD_ANGLE 4     // ~0.164 m
#define TURN_ANGLE 2.3     // ~90 degrees
#define S_STOP 8            // Stop state

#define S1 1
#define S2 2 
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7
#define S8 8 
#define S10 10  //myRobot_Obstacle_Avoidance.hpp S10---- free to go 

using namespace webots;

class myRobot : public Robot {
public:
    Motor *leftMotor, *rightMotor;
    PositionSensor *leftPositionSensor, *rightPositionSensor;
    unsigned char state;
    double refPositions[2];
    double pos[2];
public:
    myRobot() {
        leftMotor = getMotor("left wheel motor");
        rightMotor = getMotor("right wheel motor");
        
        leftPositionSensor = leftMotor->getPositionSensor();
        rightPositionSensor = rightMotor->getPositionSensor();
        
        leftPositionSensor->enable(TIME_STEP);
        rightPositionSensor->enable(TIME_STEP);
        
        leftMotor->setPosition(INFINITY);
        rightMotor->setPosition(INFINITY);
        
        state = S1;
        getPositions(pos);

        refPositions [0] = pos[0];
        refPositions [1] = pos[1];        
    }

    void Forward(){
        leftMotor->setVelocity(0.8*MAX_SPEED);
        rightMotor->setVelocity(0.8*MAX_SPEED);
    }
    void Left_turn(){
        leftMotor->setVelocity(-0.7*MAX_SPEED);
        rightMotor->setVelocity(0.7*MAX_SPEED);
    }
    void Right_turn(){
        leftMotor->setVelocity(0.7*MAX_SPEED);
        rightMotor->setVelocity(-0.7*MAX_SPEED);
    }
    void Backward(){
        leftMotor->setVelocity(-0.8*MAX_SPEED);
        rightMotor->setVelocity(-0.8*MAX_SPEED);
    }

    void getPositions(double *pos) {
        pos[0] = leftPositionSensor->getValue();
        pos[1] = rightPositionSensor->getValue();
    }

    bool reached(double *pos) {
        bool reach = false ;
            getPositions(pos);
        switch(state){
            case S1:
            case S3:
            case S5:
            case S7:{
                reach = ((abs(pos[0]-refPositions[0]))> FORWARD_ANGLE ||
                        (abs(pos[1]-refPositions[1]))> FORWARD_ANGLE );
                break;
            }
        
            case S2:
            case S4:
            case S6:
            case S8:{
                reach =((abs(pos[0]-refPositions[0]))> TURN_ANGLE ||
                        (abs(pos[1]-refPositions[1]))> TURN_ANGLE );
                break;
            }
            default:{
                reach = false;
                break;
            }
            }
            printf("dis=%f",abs(pos[0]-refPositions[0]));
            return reach;
        }


    void transition(double *pos) {
        static bool transNext = false;
        unsigned char nextState = state;
        switch (state) {
            case S1:{
                if (transNext){
                Forward();
                transNext = false;
                }
                if (reached(pos))
                    nextState = S2;
                break;}
            case S2:{
                if (transNext){
                Left_turn();
                transNext = false;
                }
                if (reached(pos))
                    nextState = S3;
                break;
            }
            case S3:{
                if (transNext){
                Forward();
                transNext = false;
                }
                if (reached(pos))
                    nextState = S4;
                break;}
            case S4:{
                if (transNext){
                Right_turn();
                transNext = false;
                }
                if (reached(pos))
                    nextState = S5;
                break;
            }
            case S5:{ 
                if (transNext){
                Backward();
                transNext = false;
                }
                if (reached(pos))
                    nextState = S6;
                break;
                }
            case S6:{
                if (transNext){
                Left_turn();
                transNext = false;
                } 
                if (reached(pos))
                    nextState = S7;
                break;
                }
            case S7:{ 
                if (transNext){
                Backward();
                transNext = false;
                }
                if (reached(pos))
                    nextState = S8;
                break;
                }
            case S8: {
                if (transNext){
                Right_turn();
                transNext = false;
                } 
                if (reached(pos))
                    nextState = S10;    //To OA hpp
                break;
                }
            default:
                break;
        }
        
        if (nextState != state){
            transNext =true ;
            refPositions[0] = pos[0];
            refPositions[1] = pos[1];
        }
        state = nextState;
        //printf("STATE:%d\n",state);

    }
};