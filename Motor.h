/*
  Motor.h - Library for controlling motor w/ sabertooth board.
  PID position regulation.
  Created by Alexis DAMIENS, 01/04/2014
  Released into the public domain.
*/
#include "Arduino.h"
//#include <PID_v1.h>

#ifndef Motor_h
#define Motor_h


#define AUTOMATIC  1
#define MANUAL  0
#define DIRECT  0
#define REVERSE  1


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

    double Setpoint, Input, Output;
    
    void SetControllerDirection(int);
    
    void SetSampleTime(int);
    
    bool PIDCompute();

    //private:
    byte _Id;
    int _Speed;
    int _AnalogInput;
    //PID _myPID;
    int _LLimit;
    int _HLimit;
    //int _SampleTime;
    
    
//  PID SECTION
    void Initialize();

    double dispKp;				// * we'll hold on to the tuning parameters in user-entered
    double dispKi;				//   format for display purposes
    double dispKd;				//

    double Kp;                  // * (P)roportional Tuning Parameter
    double Ki;                  // * (I)ntegral Tuning Parameter
    double Kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;

    double myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double myOutput;             //   This creates a hard link between the variables and the
    double mySetpoint;           //   PID, freeing the user from having to constantly tell us
    //   what these values are.  with pointers we'll just know.

    unsigned long lastTime;
    double ITerm, lastInput;

    unsigned long SampleTime;
    double outMin, outMax;
    bool inAuto;

};

#endif
