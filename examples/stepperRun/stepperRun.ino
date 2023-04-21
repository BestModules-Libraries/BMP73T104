/*************************************************
File:             Example2_stepperRun.ino
Description:      The stepper motor cycles through the following actions 
                  1.STEP1 rotate forward 2000 steps at a speed of 1000 steps/s 
                  then STEP1 has acceleration back to 0 position 
Note:               
**************************************************/
#include <BMP73T104.h>
BMP73T104 myStepper1(2,StepMotor);
//Constructing the Stepper Motor Object
//BMP73T104(object) Choose motor type. object:DCMotor or StepMotor
void setup() {
    myStepper1.begin(STEP1,HALF_STEP); 
    //STEP1 is initialized as myStepper1
    //Stepper motor rotation mode is HALF_STEP. 
    //STEPn is the channel number of stepper      
    //STEP1 = 0x03;STEP2 = 0x0C;
    //interface:FULL_STEP(0) , HALF_STEP(1) or MICRO_STEP(2) 
    myStepper1.setStepperMaxSpeed(1200);
    myStepper1.setStepperAcceleration(800);
}

void loop() {   
    myStepper1.stepperMove(2000,1000);
    delay(5000);  
    myStepper1.stepperMoveTo(0); 
    delay(5000); 
}
