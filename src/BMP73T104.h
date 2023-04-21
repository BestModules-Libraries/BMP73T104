/*****************************************************************
File:             BMP73T104.h
Author:           W, BESTMODULE
Description:      Define classes and required variables
History：         
V1.0.1-- initial version；2023-01-09；Arduino IDE : ≥v1.8.15
******************************************************************/
#ifndef BMP73T104_h
#define BMP73T104_h

#include <Arduino.h>
#include "Wire.h"
#define Slave_Addr 	(0x19)
#define ENABLE  	(1)
#define DISABLE 	(0)
#define FULL_STEP  	(0)
#define HALF_STEP  	(1)
#define MICRO_STEP 	(2)

#define CHECK_OK        (0)
#define CHECK_ERROR     (1)
enum dir_type{
    CW = 0,
    CCW = 1,
};
enum driver_channel_type{
    DRV1 = 0x01,
    DRV2 = 0x02,
    DRV3 = 0x04,
    DRV4 = 0x08,
};
enum motor_channel_type{
    M1 = DRV1,
    M2 = DRV2,
    M3 = DRV3,
    M4 = DRV4,
};
enum stepper_channel_type{
    STEP1 = DRV1|DRV2,
    STEP2 = DRV3|DRV4,
};

enum Motor_Type{
    StepMotor = 1,
    DCMotor = 0,
  };

class BMP73T104
{ 
  public:  
    BMP73T104(uint8_t intPin,uint8_t object,TwoWire *theWire = &Wire);
    uint8_t begin(uint8_t motor,uint8_t interface = 0,uint8_t i2c_addr = Slave_Addr);
    
    uint8_t setOCP(uint8_t ocp,uint8_t debounce = 1,uint8_t status = ENABLE); 
    uint8_t clearOverCurrent();
    uint8_t dcMotorRun(int8_t rank);  
    uint8_t dcMotorStop();    
    uint8_t dcMotorBrake();
    uint8_t getINT();
    int8_t getDcMotorRank();

    uint8_t stepperMoveTo(int32_t absolute, uint16_t speed);
    uint8_t stepperMoveTo(int32_t absolute);
    uint8_t stepperMove(int32_t relative, uint16_t speed);
    uint8_t stepperMove(int32_t relative);
    uint8_t stepperRun(uint8_t dir, uint16_t speed);
    uint8_t stepperStop();
    uint8_t stepperRelease();
    uint8_t isRunning();
    uint16_t getStepperMaxSpeed();
    uint16_t getStepperSpeed();
    uint16_t getStepperAcceleration();
    int32_t getStepperTargetPosition();
    int32_t getStepperDistanceToGo();
    int32_t getStepperPosition();
    uint8_t setStepperMaxSpeed(uint16_t maxSpeed);
    uint8_t setStepperAcceleration(uint16_t acceleration);
    uint8_t setStepperCurrentPosition(int32_t position);
    uint8_t setStepperHoldTorque(uint8_t status = DISABLE);
    
    uint8_t setIICSlaveAddr(uint8_t i2c_addr);
    void reset();
    uint16_t getFWVer();
  private:     
    void writeBytes(uint8_t wbuf[], uint8_t wlen);
    uint8_t readBytes(uint8_t rbuf[], uint8_t rlen); 
    uint8_t _intPin;
    TwoWire *_wire = NULL; 
    uint8_t _slaveAddr;
    uint8_t _driver;
    uint8_t _interface;
    uint8_t _motor;
    int8_t _dcrank;
    uint16_t _stepperMaxSpeed;
    uint16_t _stepperAccel;
    
};
#endif
