/*
  Motor.h - Library for controlling motor w/ sabertooth board.
  PID position regulation.
  Created by Alexis DAMIENS, 01/04/2014
  Released into the public domain.
*/
#include "Arduino.h"
#include <PID_v1.h>
//#include <DueFlashStorage.h>

#ifndef Motor_h
#define Motor_h


/////////////////////////////////////////////////////////////////////
//                                                                 //
//                      Motor Class                                //
//                                                                 //
/////////////////////////////////////////////////////////////////////
class Motor
{
  public:
    Motor(byte id, double kp, double ki, double kd, int AnalogInput);      // Constructor using PID parameters, Motor Id (1 or 2) and position return ADC channel
    
    void writeSpeed(int Speed);
    
    void changePID(double kp, double ki, double kd);
    
    void changePIDdirection();
    
    int update(double setPoint);
    
    int update();
    
    int getPosition();
    
    void start();
    
    void setSampleTime(int SampleTime);
    
    int getSampleTime(void);
    
    void stop();
    
    void setLimits(int H, int L);
    
    void saveParameters();
    
    double Kp, Ki, Kd;
    double Setpoint, Input, Output;
    
  //private:
    byte _Id;
    int _Speed;
    int _AnalogInput;
    PID _myPID;
    int _LLimit;
    int _HLimit;
    int _SampleTime;
    //DueFlashStorage dfs;
};

#endif
