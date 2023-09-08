/*****************************************************************
File:          BMP73T104.cpp
Author:        BESTMODULE
Description:   Stepper library/DC motor library for Arduino 
Version：      V1.0.1   --2023-01-09
******************************************************************/
#include "BMP73T104.h"
#define IIC_CMD_BEGIN           (0x01)
#define IIC_CMD_DCMOTOR_RUN     (0x02)
#define IIC_CMD_DCMOTOR_BRAKE   (0x03)
#define IIC_CMD_DCMOTOR_STOP    (0x04)
#define IIC_CMD_SET_OCP         (0x05)
#define IIC_CMD_SET_MAXSPEED    (0x06)
#define IIC_CMD_SET_ACCEL       (0x07)
#define IIC_CMD_SET_POSITION    (0x08)
#define IIC_CMD_MOVETO_S        (0x09)
#define IIC_CMD_MOVETO_A        (0x0A)
#define IIC_CMD_MOVE_S          (0x0B)
#define IIC_CMD_MOVE_A          (0x0C)
#define IIC_CMD_STEP_RUN        (0x0D)
#define IIC_CMD_STEP_STOP       (0x0E)
#define IIC_CMD_RESET           (0x0F)
#define IIC_CMD_RUNNING         (0x10)
#define IIC_CMD_GET_SPEED       (0x11)
#define IIC_CMD_GET_T_POSITION  (0x12)
#define IIC_CMD_GET_STEP_TOGO   (0x13)
#define IIC_CMD_GET_POSITION    (0x14)
#define IIC_CMD_CLEAR_OCP_STU   (0x15)
#define IIC_CMD_HOLD_TORQUE     (0x16)
#define IIC_CMD_GET_ADC         (0x17)
#define IIC_CMD_SET_IIC_ADDR    (0x18)
#define IIC_CMD_GET_FWVER       (0X19)

#define waitTime  (1)
/**********************************************************
Description:Constructor 
Parameters: intPin:INT pin
	    object:Type of Motor , Stepper:StepMotor(0x01);DC:DCMotor(0x00)
            wireSet: IIC Channel,Wire/Wire1/Wire2    
Return:               
Others:     
**********************************************************/ 
BMP73T104::BMP73T104(uint8_t intPin,uint8_t object,TwoWire *theWire)
{
    _motor = 0;   
    _dcrank = 0; 
    _stepperMaxSpeed = 1000; 
    _stepperAccel = 800; 
    _driver = object;
    _wire = theWire;
    _intPin = intPin;
}
/**********************************************************
Description:Motor initial 
Parameters: motor:Drive channel number.M1,M2,M3,M4 or STEP1,STEP2
            interface:Stepper motor rotation mode ,FULL_STEP(0)/HALF_STEP(1)/MICRO_STEP(2)
	    slaveAddr: IIC Slave Address,this motor drive extended version Address is 0x19
Return:     1:No ACK 0:ACK       
Others:     
**********************************************************/
uint8_t BMP73T104::begin(uint8_t motor,uint8_t interface,uint8_t i2c_addr)
{ 
    //uint8_t check = 0;
    uint8_t txData[7] = {0x55,0x73,0x04,IIC_CMD_BEGIN,0x00,0x00,0x00};
    uint8_t rxData[20] = {0};
    _slaveAddr = i2c_addr;
    pinMode(_intPin,INPUT_PULLUP);
    _wire->begin();
    if(_driver == 0)
    {
      if(motor == 0x01||motor == 0x02||motor == 0x04||motor == 0x08)
      {
        _motor = motor;
        _interface = 0;      
      }
      else 
      {
        _motor = 0;  
      }
    }
    if(_driver == 1)
    {
      if(motor == 0x03||motor == 0x0c)
      {
        _motor = motor;
        _interface = interface;
      }
      else 
      {
        _motor = 0;  
      }
    }
    delay(waitTime);
    txData[4] = _motor;
    txData[5] = _interface;
    txData[6] = ~(0x55+0x73+0x04+IIC_CMD_BEGIN+_motor+_interface);
    writeBytes(txData,7);
     
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(txData[3] == IIC_CMD_BEGIN)
    {
      return 0;  
    }  
    return 1;
}
/**********************************************************
Description:Set overcurrent protection ADC value
Parameters: ocp:(Iset*0.05/Vioref)*4095
            debounce:Prevent shaking,time = debounce * 5ms
            status:Switch,ENABLE open DISENABLE close 
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::setOCP(uint8_t ocp,uint8_t debounce,uint8_t status)
{
 
    uint8_t txData[9] = {0x55,0x73,0x06,IIC_CMD_SET_OCP,_motor,status,ocp,debounce,0x00};
    uint8_t rxData[20] = {0};
    
    delay(waitTime);
    txData[8] = ~(0x55+0x73+0x06+IIC_CMD_SET_OCP+_motor+debounce+ocp+status);
    writeBytes(txData,9);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_SET_OCP)
    {
      return 0;  
    }
    return 1;
}
/**********************************************************
Description:Clear overcurrent status
Parameters:      
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::clearOverCurrent()
{
    uint8_t txData[5] = {0x55,0x73,0x02,IIC_CMD_CLEAR_OCP_STU,0x00};
    uint8_t rxData[20] = {0};
    uint8_t cmd_data = IIC_CMD_CLEAR_OCP_STU;
    
    delay(waitTime);
    txData[4] = ~(0x55+0x73+0x02+cmd_data);
    writeBytes(txData,5);

    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_CLEAR_OCP_STU)
    {
      return 0;  
    }
    return 1; 	
} 
/**********************************************************
Description:Rotating DC motor  
Parameters: rank:Speed gear, range - 100 to + 100,  
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::dcMotorRun(int8_t rank)
{   
    uint8_t txData[7] = {0x55,0x73,0x04,IIC_CMD_DCMOTOR_RUN,_motor,0x00,0x00};
    uint8_t rxData[20] = {0};
    
    if (rank > 100)
    {
      rank = 100;
    }
    else if (rank < -100) 
    {
      rank = -100;
    }       
    _dcrank = rank;
    
    delay(waitTime);
    txData[5] = rank;
    txData[6] = ~(0x55+0x73+0x04+IIC_CMD_DCMOTOR_RUN+_motor+rank);
    writeBytes(txData,7);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_DCMOTOR_RUN)
    {
      return 0;  
    }
    return 1;
}
/**********************************************************
Description: DC motor enter standby mode  
Parameters:                     
Return:      1:No ACK 0:ACK        
Others:      
**********************************************************/
uint8_t BMP73T104::dcMotorStop()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_DCMOTOR_STOP,_motor,0x00};
    uint8_t rxData[20] = {0};
    _dcrank = 0;
    delay(waitTime);
    txData[5] = ~(0x55+0x73+0x03+IIC_CMD_DCMOTOR_STOP+_motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_DCMOTOR_STOP)
    {
      _dcrank = 0;
      return 0;  
    }
    return 1;     
}
/**********************************************************
Description:DC motor stop stall  
Parameters:              
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::dcMotorBrake()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_DCMOTOR_BRAKE,_motor,0x00};
    uint8_t rxData[20] = {0};
    _dcrank = 0;
    delay(waitTime);
    txData[5] = ~(0x55+0x73+0x03+IIC_CMD_DCMOTOR_BRAKE+_motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_DCMOTOR_BRAKE)
    {
      _dcrank = 0;
      return 0;  
    }
    return 1;
}
/**********************************************************
Description:get INT pin status 
Parameters: 
Return:     Overcurrent status       
Others:     
**********************************************************/
uint8_t BMP73T104::getINT()
{
    uint8_t IRQ;
    IRQ = digitalRead(_intPin);
    return IRQ;            
}
/**********************************************************
Description:Get DC motor speed  
Parameters:                       
Return:     rank:Speed gear, range - 100 to + 100, 
            positive clockwise, negative counterclockwise  
Others:     
**********************************************************/
int8_t BMP73T104::getDcMotorRank()           
{
    return _dcrank;   
}

/**********************************************************
Description:Turn to the target position at a constant speed      
Parameters: absolute:Absolute position  
            speed:the speed of the stepper motor                   
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/  
uint8_t BMP73T104::stepperMoveTo(int32_t absolute, uint16_t speed)
{
    uint8_t txData[12] = {0x55,0x73,0x09,IIC_CMD_MOVETO_S,_motor,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t a,b,c,d,e,f;
    uint8_t rxData[20] = {0};
    delay(waitTime);
    
    d =  (absolute>>24)&0xff;
    c =  (absolute>>16)&0xff;
    b =  (absolute>>8)&0xff;
    a =  absolute&0xff;
    f = (speed>>8)&0xff;
    e = speed&0xff;
 
    txData[5] = d;
    txData[6] = c;
    txData[7] = b;
    txData[8] = a;
    txData[9] = f;
    txData[10] = e;
    txData[11] = ~(0x55+0x73+0x09+IIC_CMD_MOVETO_S+_motor+a+b+c+d+e+f);
    writeBytes(txData,12);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_MOVETO_S)
    {
      return 0;  
    }
    return 1; 
}
/**********************************************************
Description:Acceleration to target position      
Parameters: absolute:Absolute position                    
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/  
uint8_t BMP73T104::stepperMoveTo(int32_t absolute)
{
    uint8_t txData[10] = {0x55,0x73,0x07,IIC_CMD_MOVETO_A,_motor,0x00,0x00,0x00,0x00,0x00};
    uint8_t a,b,c,d;
    uint8_t rxData[20] = {0};
    
    delay(waitTime);
    d =  (absolute>>24)&0xff;
    c =  (absolute>>16)&0xff;
    b =  (absolute>>8)&0xff;
    a =  absolute&0xff;

    txData[5] = d;
    txData[6] = c;
    txData[7] = b;
    txData[8] = a;
    txData[9] = ~(0x55+0x73+0x07+IIC_CMD_MOVETO_A+_motor+a+b+c+d);
    writeBytes(txData,10);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_MOVETO_A)
    {
      return 0;  
    }
    return 1;  
}
/**********************************************************
Description:How many steps do you rotate at a uniform speed    
Parameters: relative:Motor rotation steps, positive indicates clockwise, negative indicates counterclockwise 
            speed:The speed of the stepper motor                     
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/  
uint8_t BMP73T104::stepperMove(int32_t relative, uint16_t speed)
{
    uint8_t txData[12] = {0x55,0x73,0x09,IIC_CMD_MOVE_S,_motor,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t a,b,c,d,e,f;
    uint8_t rxData[20] = {0};
    
    delay(waitTime);
    d = (relative>>24)&0xff;   
    c = (relative>>16)&0xff;
    b = (relative>>8)&0xff;
    a = relative&0xff;
    f = (speed>>8)&0xff;
    e = speed&0xff;
    
    txData[5] = d;
    txData[6] = c;
    txData[7] = b;
    txData[8] = a;
    txData[9] = f;
    txData[10] = e;
    txData[11] = ~(0x55+0x73+0x09+IIC_CMD_MOVE_S+_motor+a+b+c+d+e+f);
    writeBytes(txData,12);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_MOVE_S)
    {
      return 0;  
    }
    return 1; 
}
/**********************************************************
Description:How many steps does the acceleration rotate     
Parameters: relative:Motor rotation steps, positive indicates clockwise, negative indicates counterclockwise                    
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::stepperMove(int32_t relative)
{
    uint8_t txData[10] = {0x55,0x73,0x07,IIC_CMD_MOVE_A,_motor,0x00,0x00,0x00,0x00,0x00};
    uint8_t a,b,c,d;
    uint8_t rxData[20] = {0};
    delay(waitTime);
    d = (relative>>24)&0xff;  
    c = (relative>>16)&0xff;
    b = (relative>>8)&0xff;
    a = relative&0xff;

    txData[5] = d;
    txData[6] = c;
    txData[7] = b;
    txData[8] = a;
    txData[9] = ~(0x55+0x73+0x07+IIC_CMD_MOVE_A+_motor+a+b+c+d);
    writeBytes(txData,10);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_MOVE_A)
    {
      return 0;  
    }
    return 1;    
}
/**********************************************************
Description:The stepper motor rotates continuously at a constant speed     
Parameters: dir:Stepper motor direction
            FORWARD = 1 indicates clockwise rotation and REVERSAL = 0 indicates counterclockwise rotation  
            speed:the speed of the stepper motor                      
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::stepperRun(uint8_t dir,uint16_t speed)
{
    uint8_t txData[9] = {0x55,0x73,0x06,IIC_CMD_STEP_RUN,_motor,dir,0x00,0x00,0x00};
    uint8_t a,b;
    uint8_t rxData[20] = {0};
    
    delay(waitTime);
    b = (speed>>8)&0xff;
    a = speed&0xff;

    txData[6] = b;
    txData[7] = a;
    txData[8] = ~(0x55+0x73+0x06+IIC_CMD_STEP_RUN+_motor+a+b+dir);
    writeBytes(txData,9);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_STEP_RUN)
    {
      return 0;  
    }
    return 1;   
}
/**********************************************************
Description:Stepper motor enters stop mode        
Parameters:                     
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::stepperStop()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_STEP_STOP,_motor,0x00};
    uint8_t rxData[20] = {0};
    delay(waitTime);

    txData[5] = ~(0x55+0x73+0x03 + IIC_CMD_STEP_STOP + _motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_STEP_STOP)
    {
      return 0;  
    }
    return 1;   
      
}
/**********************************************************
Description:Release all pins of the stepper motor so it free-spins. 
Parameters:                     
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/  
uint8_t BMP73T104::stepperRelease ()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_STEP_STOP,_motor,0x00};
    uint8_t rxData[20] = {0};
    delay(waitTime);
 
    txData[5] = ~(0x55+0x73+0x03 + IIC_CMD_STEP_STOP + _motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_STEP_STOP)
    {
      return 0;  
    }
    return 1;   
}
/**********************************************************
Description:Check whether the motor is running towards the target          
Parameters:                
Return:     true:Arrived  false:Not arrived      
Others:     
**********************************************************/
uint8_t BMP73T104::isRunning()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_RUNNING,_motor,0x00};
    bool isR = 1;
    uint8_t rxData[20] = {0};
    delay(waitTime);
    txData[5] = ~(0x55+0x73+0x03+IIC_CMD_RUNNING+_motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    readBytes(rxData,6);
    if(rxData[3] == IIC_CMD_RUNNING)
    {
       isR = rxData[4];
    }
    return isR;
}
/**********************************************************
Description:Get maximum speed         
Parameters:                      
Return:     Maximum speed of stepper motor       
Others:     
**********************************************************/
uint16_t BMP73T104::getStepperMaxSpeed()
{
     return _stepperMaxSpeed;
} 
/**********************************************************
Description:Get current speed         
Parameters:                                 
Return:     Speed of stepper motor          
Others:     
**********************************************************/  
uint16_t BMP73T104::getStepperSpeed()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_GET_SPEED,_motor,0x00};
    uint16_t Speed_Data = 0;
    uint8_t rxData[20] = {0};

    delay(waitTime);
    txData[5] = ~(0x55+0x73+0x03+IIC_CMD_GET_SPEED+_motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    if(readBytes(rxData,7) != 0)
    {
      return 0;
    }

    if(rxData[3] == IIC_CMD_GET_SPEED)
    {
       Speed_Data = rxData[4];
       Speed_Data = (Speed_Data<<8) + rxData[5];
    }
    if(Speed_Data == 0xffff)
    {
      Speed_Data = 0;
    }
    return Speed_Data;
}
/**********************************************************
Description: Get acceleration        
Parameters:                       
Return:      Stepper motor acceleration 
Others:      
**********************************************************/
uint16_t BMP73T104::getStepperAcceleration()
{
      return _stepperAccel;
}
/**********************************************************
Description: Get target location         
Parameters:                        
Return:      Target position of stepper motor        
Others:      
**********************************************************/
int32_t BMP73T104::getStepperTargetPosition()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_GET_T_POSITION,_motor,0x00};
    uint32_t TargetPosition=0;
    uint8_t rxData[20] = {0};
    delay(waitTime);
    txData[5] = ~(0x55+0x73+0x03+IIC_CMD_GET_T_POSITION+_motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    if(readBytes(rxData,9) != 0)
    {
      return 0;
    }
    if(rxData[3] == IIC_CMD_GET_T_POSITION)
    {
      for(uint8_t i = 4; i < 8; i++)
      {
        TargetPosition = TargetPosition<<8;
        TargetPosition = TargetPosition + rxData[i];
      }
    }
    return (int32_t)TargetPosition;
} 
/**********************************************************
Description: Gets the distance between the current position and the target position         
Parameters:         
Return:      The distance between the current position and the target position       
Others:      
**********************************************************/   
int32_t BMP73T104::getStepperDistanceToGo()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_GET_STEP_TOGO,_motor,0x00};
    uint32_t go=0;
    uint8_t rxData[20] = {0};
    
    delay(waitTime);
    txData[5] = ~(0x55+0x73+0x03+IIC_CMD_GET_STEP_TOGO+_motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    if(readBytes(rxData,9) != 0)
    {
      return 0;
    }
    if(rxData[3] == IIC_CMD_GET_STEP_TOGO)
    {
      for(uint8_t i = 4; i < 8; i++)
      {
        go = go<<8;
        go = go + rxData[i];
      }
    }
    return (int32_t)go;
}
/**********************************************************
Description: Get current location            
Parameters:                         
Return:      Current position        
Others:      
**********************************************************/
int32_t BMP73T104::getStepperPosition()
{
    uint8_t txData[6] = {0x55,0x73,0x03,IIC_CMD_GET_POSITION,_motor,0x00};
    uint32_t Position=0; 
    uint8_t rxData[20] = {0};
    delay(waitTime);
    txData[5] = ~(0x55+0x73+0x03+IIC_CMD_GET_POSITION+_motor);
    writeBytes(txData,6);
    
    delay(waitTime);
    if(readBytes(rxData,9) != 0)
    {
      return 0;
    }
    if(rxData[3] == IIC_CMD_GET_POSITION)
    {
      for(uint8_t i = 4; i < 8; i++)
      {
        Position = Position<<8;
        Position = Position + rxData[i];
      }
    }
    return (int32_t)Position; 
}
/**********************************************************
Description:Sets the maximum speed of the stepper motor  
Parameters: maxSpeed:The maximum speed of the stepper motor     
            Steps per second                  
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::setStepperMaxSpeed(uint16_t maxSpeed)
{
    uint8_t txData[8] = {0x55,0x73,0x05,IIC_CMD_SET_MAXSPEED,_motor,0x00,0x00,0x00};
    uint8_t a,b;
    uint8_t rxData[20] = {0};
    
    delay(waitTime);
    _stepperMaxSpeed = maxSpeed;
    b = (maxSpeed>>8)&0xff;  
    a = maxSpeed&0xff;

    delay(waitTime);
    txData[5] = b;
    txData[6] = a;
    txData[7] = ~(0x55+0x73+0x05+IIC_CMD_SET_MAXSPEED+_motor+a+b);
    writeBytes(txData,8);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_SET_MAXSPEED)
    {
      return 0;  
    }
    return 1; 
}
/**********************************************************
Description:Sets the acceleration of the stepper motor  
Parameters: acceleration:The acceleration of the stepper motor 
            Range 1~65535 , Step square per second                      
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::setStepperAcceleration(uint16_t acceleration)
{
    uint8_t txData[8] = {0x55,0x73,0x05,IIC_CMD_SET_ACCEL,_motor,0x00,0x00,0x00};
    uint8_t a,b;
    uint8_t rxData[20] = {0};
    
    delay(waitTime);
    _stepperAccel = acceleration;   
    b = (acceleration>>8)&0xff;   
    a = acceleration&0xff;
    delay(waitTime);
    txData[5] = b;
    txData[6] = a;
    txData[7] = ~(0x55+0x73+0x05+IIC_CMD_SET_ACCEL+_motor+a+b);
    writeBytes(txData,8);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_SET_ACCEL)
    {
      return 0;  
    }
    return 1; 
}
/**********************************************************
Description:Set the current position of the stepper motor 
Parameters: position:The current position of the stepper motor 
            Range -2^31 ~ +2^31                     
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/
uint8_t BMP73T104::setStepperCurrentPosition(int32_t position)
{
    uint8_t txData[10] = {0x55,0x73,0x07,IIC_CMD_SET_POSITION,_motor,0x00,0x00,0x00,0x00,0x00};
    uint8_t a,b,c,d;
    uint8_t rxData[20] = {0};
    delay(waitTime);
         
    d = (position>>24)&0xff;   
    c = (position>>16)&0xff;
    b = (position>>8)&0xff;
    a = position&0xff;

    txData[5] = d;
    txData[6] = c;
    txData[7] = b;
    txData[8] = a;
    txData[9] = ~(0x55+0x73+0x07+IIC_CMD_SET_POSITION+_motor+a+b+c+d);
     
    writeBytes(txData,10);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_SET_POSITION)
    {
      return 0;  
    }
    return 1; 
}
/**********************************************************
Description:Set stepper motor holding torque 
Parameters: status:ENABLE or DISABLE                   
Return:     1:No ACK 0:ACK        
Others:     
**********************************************************/  
uint8_t BMP73T104::setStepperHoldTorque(uint8_t status)
{
    uint8_t txData[7] = {0x55,0x73,0x04,IIC_CMD_HOLD_TORQUE,_motor,status,0x00};
    uint8_t rxData[20] = {0};

    delay(waitTime);
    txData[6] = ~(0x55+0x73+0x04+IIC_CMD_HOLD_TORQUE+_motor+status);
    writeBytes(txData,7);
    
    delay(waitTime);
    if(readBytes(rxData,6) != 0)
    {
      return 1;
    }
    if(rxData[3] == IIC_CMD_HOLD_TORQUE)
    {
      return 0;  
    }
    return 1;  
}
/**********************************************************
Description:change IIC slave address
Parameters: addr:slave address 0~127      
Return:     1:No ACK 0:ACK          
Others:     
**********************************************************/ 
uint8_t BMP73T104::setIICSlaveAddr(uint8_t i2c_addr)
{
    uint8_t j = 0,checkSumTx = 0,checkSumRx = 0;
    uint8_t rxData[20] = {0};
    uint8_t rxNum = 6;
    _wire->begin();
    delay(100);
    for(uint8_t i = 0; i < 128; i++)
    {
      _wire->beginTransmission(i);
      _wire->write(0x55);     //top
      _wire->write(0x73);     //slave class
      _wire->write(0x03);     //len
      _wire->write(IIC_CMD_SET_IIC_ADDR);     //cmd
      _wire->write(i2c_addr);     //data1
      checkSumTx = ~(0x55+0x73+0x03+IIC_CMD_SET_IIC_ADDR+i2c_addr);
      _wire->write(checkSumTx);    //dcs
      _wire->endTransmission();  
      delay(2);
    }
    delay(waitTime);
    _wire->requestFrom(i2c_addr,rxNum);
    while(_wire->available())
    {
      rxData[j] = _wire->read();
      checkSumRx += rxData[j];
      j++;
    }
    checkSumRx = checkSumRx + 1;
    if((rxData[3] == IIC_CMD_SET_IIC_ADDR)&&(!rxData[4])&&(!checkSumRx))
    {
      return 0;  
    }
    else 
    {
      return 1;
    }
}
/**********************************************************
Description: Reset motor driver            
Parameters:                         
Return:             
Others:      
**********************************************************/
void BMP73T104::reset()
{
    uint8_t txData[5] = {0x55,0x73,0x02,IIC_CMD_RESET,0x00};
  	uint8_t cmd_reset = IIC_CMD_RESET;
    delay(waitTime);
    txData[4] = ~(0x55+0x73+0x02+cmd_reset);
    writeBytes(txData,5);
}
/**********************************************************
Description: Get version number         
Parameters:                      
Return:      0x0101:Version number 1.01       
Others:      
**********************************************************/
uint16_t BMP73T104::getFWVer()
{
    uint8_t txData[5] = {0x55,0x73,0x02,IIC_CMD_GET_FWVER,0x00};
    uint16_t FWVer = 0;
    uint8_t get_cmd = IIC_CMD_GET_FWVER; 
    uint8_t rxData[20] = {0};
    delay(waitTime);
    txData[4] = ~(0x55+0x73+0x02+get_cmd);
    writeBytes(txData,5);
    
    delay(waitTime);
    if(readBytes(rxData,7) != 0)
    {
      return 0;
    }
    if(rxData[3] == IIC_CMD_GET_FWVER)
    {
      FWVer = rxData[4];
      FWVer = (FWVer<<8) + rxData[5];
    }
    return FWVer;
} 

void BMP73T104::writeBytes(uint8_t wbuf[], uint8_t wlen)
{
  if (_wire != NULL)
  {
    while (_wire->available() > 0)
    {
      _wire->read();
    }
    _wire->beginTransmission(_slaveAddr);
    _wire->write(wbuf, wlen);
    _wire->endTransmission();
  }

}
uint8_t BMP73T104::readBytes(uint8_t rbuf[], uint8_t rlen)
{
  uint8_t i = 0, checkSum = 0;
  _wire->requestFrom(_slaveAddr, rlen);
  for (i = 0; i < rlen; i++)
  {
    rbuf[i] = _wire->read();
    /* Check Sum */
    checkSum += rbuf[i];
  }
  checkSum += 1;
  if (!checkSum)
  {
    return CHECK_OK; // Check correct
  }
  else
  {
    return CHECK_ERROR; // Check error
  }
}
