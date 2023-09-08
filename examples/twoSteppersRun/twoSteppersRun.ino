/*************************************************
File:             Example_twoStepperRun.ino
Description:      The expansion board performs the following actions 
                  1.Stepper motor 1 rotates clockwise at a speed of 200 steps/s, and stepper motor 2 rotates counterclockwise at a speed of 800 steps/s for 3 seconds
                  2.Stepper motor stops for 1 second
                  3.Stepper motor 1 rotates counterclockwise at a speed of 200 steps/s, and stepper motor 2 rotates clockwise at a speed of 800 steps/s for 3 seconds
                  4.Stepper motor stops for 1 second
                 
Note:               
**************************************************/
#include <BMP73T104.h>

 
BMP73T104 myStepper1(2,StepMotor);          //Constructing the Stepper Motor Object
BMP73T104 myStepper2(2,StepMotor);          //Constructing the Stepper Motor Object
 
void setup() {
 
  myStepper1.begin(STEP1,HALF_STEP);        //Stepper motor 1 initialization
  myStepper1.setStepperMaxSpeed(1200);
  myStepper1.setStepperAcceleration(800);

  myStepper2.begin(STEP2,HALF_STEP);        //Stepper motor 2 initialization
  myStepper2.setStepperMaxSpeed(1200);
  myStepper2.setStepperAcceleration(800);
}

void loop() {
  
  myStepper1.stepperRun(CW,200); 
  myStepper2.stepperRun(CCW,800); 
  delay(3000);
  
  myStepper1.stepperStop(); 
  myStepper2.stepperStop(); 
  delay(1000);
  
  myStepper1.stepperRun(CCW,200);   
  myStepper2.stepperRun(CW,800); 
  delay(3000);
  
  myStepper1.stepperStop(); 
  myStepper2.stepperStop(); 
  delay(1000);
}
