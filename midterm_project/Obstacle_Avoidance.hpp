#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <complex.h>
#include <iostream>
#include "Square.hpp"

    
#define THRESHOLD 75        //best 73 probitity


#define S10 10              //free to go 
#define S11 11              //leftOA
#define S12 12              //rightOA
#define S13 13              //80L10R
#define S14 14              //80R10L
#define S15 15              //50R50L

#define U0_front 0          //OA in front
#define U1_left 1           //OA on the left side
#define U2_right 2          //OA on the right side
#define U3_back_or_NAN 3    //OA in the back or NAN

//using namespace webots;
//using namespace std;

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
        srand( time(NULL) );

        char psNames[8][4] = {
            "ps0","ps1","ps2","ps3",
            "ps4","ps5","ps6","ps7"
        };
        for (int i=0 ; i<8 ; i++){
            ps[i] = getDistanceSensor(psNames[i]);
            ps[i] ->enable(TIME_STEP);
        }
        oa_state = S10;
     }
    //center of mass
    double calculateCenterOfMass() {
        double theta[8] = {0.30,  0.80,  1.57,  2.64, -2.64, -1.57, -0.80, -0.30};
        double ms[8];
        //std::complex<double> z[8];
        std::complex<double> zc = 0;
        double m = 0;
        for (int i=0; i<8; i++){
            psValues[i] = ps[i]->getValue();
            //printf("ps %d: %f \n",i ,psValues[i]);
            if (psValues[i] > THRESHOLD ){
                 ms[i] = psValues[i] - THRESHOLD;
                 zc += std::polar(ms[i],theta[i]);
                 m += ms[i];
            }
            //else{
            //     ms[i] = 0;
            //     }
        }
         if (m == 0)
             thetac = NAN;                                  // all sensors return below threshold
         else{
             zc = zc/m;
             thetac = std::arg(zc);
          }
         printf("thetac:%f\n",thetac);
         return thetac;
    }
    //condition the 
    unsigned char range(){ 
        calculateCenterOfMass();                              //whith this,there would be no thetac in this conditional judgment equation                 
        unsigned char Variable ;
        //if( std::isnan(thetac)){                            //if thatac is NAN,but it has been lose to U3(else)
        //    Variable =U3;
        //} 
         if (-0.1 <= thetac && thetac <= 0.1)                 
            Variable = U0_front;                                    
        else if (-1.485 <= thetac && thetac < -0.1)          
            Variable = U1_left;
        else if (0.1 < thetac && thetac <= 1.485)             
            Variable = U2_right;
        else 
            Variable = U3_back_or_NAN;                        
     //printf("thetac: %f, range:%d/n", thetac, Variable);
      return Variable;
    }

    void STATE_OA(){
      unsigned char next_state = oa_state;
      myRobot :: transition(pos);

      if (state > S8){                                        //Judgment status:Can the Square FSM enter the Obstacle_Avoidance FSM 
        switch (oa_state) {             
            case S10: {                                       //S10:free to go 
                Forward();
                if (range() == U0_front ){                            
                    //Backward();       
                    next_state = S15 ; //
                }
                else if (range() == U1_left) {
                    //Backward();
                    next_state = S14;
                }
                else if (range() == U2_right) {
                    //Backward();
                    next_state = S13 ;
                }
                else if(range() == U3_back_or_NAN){    
                    next_state = S10; 
                }
                break;
            }
            case S11: {                                       //S11: right turn                    
                //Backward();                                 //if I put it here -> shaking -> can't walk fast -> lose score  
                Right_turn();
                if (range() == U3_back_or_NAN) {
                    next_state = S10;
                }
                else
                    next_state =  S11;
                break;
            }
            case S12: {                                       //S12 : left turn
                //Backward();
                Left_turn();
                if (range() == U3_back_or_NAN) {
                    next_state = S10;
                }
                else
                    next_state =  S12;
                break;
                
            }
            case S13:{                                        //S13 : right turn 20% : left turn 80%                         
                Backward();
                int random = rand() % 10 ;
                if( random == 8 || random == 2){
                   next_state = S11;
               }
                else 
                   next_state = S12;
                break;
            }
            case S14:{                                        //S14 : right turn 80% : left turn 20%
                Backward();
                int random = rand() % 10 ;
                if( random == 8 || random == 2){
                   next_state = S12;
               }
                else 
                   next_state = S11;
                break;
            }
            case S15:{                                        //S15 : right turn 50% : left turn 50%
                Backward();
                int random = rand() % 2 ;
                if( random == 0){
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
      }
        oa_state = next_state;
        printf("state:%d ,oa_state:%d\n",state,oa_state);
    }
  };