/*************************************************
File:             Example1_dcMotorRun.ino
Description:      The DC motor cycles through the following actions
                  M1 and M2 rotates forward for 5 seconds at a speed of 80% and 60% duty cycle
                  and reverses for 5 seconds at a speed of 60 and 80 duty cycle
Note:               
**************************************************/
#include <BMP73T104.h>
BMP73T104 myM1(2,DCMotor);
BMP73T104 myM2(2,DCMotor);

//Constructing the DC Motor Object
//BMP73T104(object) Choose motor type. object:DCMotor or StepMotor
void setup() {
    myM1.begin(M1);
    //M1 is initialized as DCMotor1
    myM2.begin(M2);
    //M1 = 0x01;M2 = 0x02;M3 = 0x04;M4 = 0x08;
}

void loop() {
    myM1.dcMotorRun(80);
    myM2.dcMotorRun(60); 
    delay(5000);    
    myM1.dcMotorRun(-60);
    myM2.dcMotorRun(-80); 
    delay(5000);
}
