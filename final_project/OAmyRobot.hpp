#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <complex> // Fixed: Replaced <complex.h> with <complex>
#include <ctime>   // For srand and time
#include "myRobot.hpp"


#define THRESHOLD 74        // Best 73 probability


#define S10 10              // Free to go
#define S11 11              // Left OA
#define S12 12              // Right OA
#define S13 13              // 80L10R
#define S14 14              // 80R10L
#define S15 15              // 50R50L


#define U0_front 0          // OA in front
#define U1_left 1           // OA on the left side
#define U2_right 2          // OA on the right side
#define U3_back_or_NAN 3    // OA in the back or NAN


class OAmyRobot : public myRobot {
public:
   unsigned char oa_state;
   double refPositions[2];
   double thetac;
   DistanceSensor *ps[8];
   double psValues[8];
public:
   OAmyRobot() {  
       double pos[2];
       getPositions(pos);
       refPositions[0] = pos[0];
       refPositions[1] = pos[1];
       srand(time(NULL));


       char psNames[8][4] = {
           "ps0","ps1","ps2","ps3",
           "ps4","ps5","ps6","ps7"
       };
       for (int i = 0; i < 8; i++) {
           ps[i] = getDistanceSensor(psNames[i]);
           ps[i]->enable(TIME_STEP);
       }
       oa_state = S10;
   }


   double calculateCenterOfMass() {
       double theta[8] = {0.30, 0.80, 1.57, 2.64, -2.64, -1.57, -0.80, -0.30};
       double ms[8]; // Fixed: Added semicolon
       std::complex<double> zc = 0;
       double m = 0;
       for (int i = 0; i < 8; i++) {
           psValues[i] = ps[i]->getValue();
           if (psValues[i] > THRESHOLD) {
               ms[i] = psValues[i] - THRESHOLD;
               zc += std::polar(ms[i], theta[i]);
               m += ms[i];
           }
       }
       if (m == 0)
           thetac = NAN; // All sensors return below threshold
       else {
           zc = zc / m;
           thetac = std::arg(zc);
       }
       printf("thetac:%f\n", thetac);
       return thetac;
   }


   unsigned char range() {
       calculateCenterOfMass();
       unsigned char Variable;
       if (-0.2 <= thetac && thetac <= 0.2)               
           Variable = U0_front;                                  
       else if (-1.57 <= thetac && thetac < -0.2)        
           Variable = U1_left;
       else if (0.2 < thetac && thetac <= 1.57)           
           Variable = U2_right;
       else
           Variable = U3_back_or_NAN;                      
       return Variable;
   }


   void STATE_OA() {
       unsigned char next_state = oa_state;
       switch (oa_state) {           
           case S10: { // Free to go
               Forward();
               if (range() == U0_front) {                          
                   next_state = S14;
               }
               else if (range() == U1_left) {
                   next_state = S11;
               }
               else if (range() == U2_right) {
                   next_state = S12;
               }
               else if (range() == U3_back_or_NAN) {  
                   next_state = S10;
               }
               break;
           }
           case S11: { // Right turn                  
               Backward();
               Right_turn();
               if (range() == U3_back_or_NAN) {
                   next_state = S10;
               }
               else
                   next_state = S11;
               break;
           }
           case S12: { // Left turn
               Backward();
               Left_turn();
               if (range() == U3_back_or_NAN) {
                   next_state = S10;
               }
               else
                   next_state = S12;
               break;
           }
           case S14:{   //S14 : right turn 80% : left turn 20%
              Backward();
               int random = rand() % 10 ;
               if( random == 8 || random == 2){
                  next_state = S12;
              }
               else
                  next_state = S11;
               break;
           }
           default: {
               break;
           }
       }
       oa_state = next_state;
       printf("state:%d ,oa_state:%d\n", state, oa_state);
   }
};

