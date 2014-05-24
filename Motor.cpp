#include "Arduino.h"
#include "Motor.h"
//#include <PID_v1.h>

#define SAMPLINGTIME 100


/*
 Motor class constructor
 Parameters :
 - byte id = Motor channel on Sabertooth board => 1 or 2
 - double kp, ki, kd = PID tuning parameters.
 - int AnalogInput = Position return ADC channel
 */

Motor::Motor(byte id, double kp, double ki, double kd, int AnalogInput)
{
  if (id == 1 || id == 2)
  {
    _Id = id;
  }
  else _Id = 1;

  _AnalogInput = AnalogInput;
  Setpoint = 0;
  Input = 0;
  Output = 0;
  outMin = -127;
  outMax = 127;

  inAuto = false;


  SampleTime = 100;							//default Controller Sample Time is 0.1 seconds

  SetControllerDirection(DIRECT);
  changePID(kp, ki, kd);

  lastTime = millis() - SampleTime;




}

void Motor::changePID(double kp, double ki, double kd)
{

  if (kp < 0 || ki < 0 || kd < 0) return;

  dispKp = kp; dispKi = ki; dispKd = kd;

  double SampleTimeInSec = ((double)SampleTime) / 1000;

  Kp = kp;
  Ki = ki * SampleTimeInSec;
  Kd = kd / SampleTimeInSec;

  if (controllerDirection == REVERSE)
  {
    Kp = (0 - kp);
    Ki = (0 - ki);
    Kd = (0 - kd);
  }
}



void Motor::changePIDdirection()
{
  if (inAuto)
  {
    Kp = (0 - Kp);
    Ki = (0 - Ki);
    Kd = (0 - Kd);
  }
  controllerDirection = !controllerDirection;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void Motor::SetControllerDirection(int Direction)
{
   if(inAuto && Direction !=controllerDirection)
   {
     Kp = (0 - Kp);
     Ki = (0 - Ki);
     Kd = (0 - Kd);
   }   
   controllerDirection = Direction;
}



void Motor::setSampleTime(int ST)
{
  SetSampleTime(ST);
}



int Motor::getSampleTime()
{
  return SampleTime;
}



int Motor::update(double setPoint)
{
  Setpoint = constrain(setPoint, _LLimit, _HLimit);
  Input = map(analogRead(_AnalogInput), 0, 1024, -127, 127);
  PIDCompute();
  writeSpeed(Output);
  return 0;
}


int Motor::update()
{
  Input = map(analogRead(_AnalogInput), 0, 1024, -127, 127);
  PIDCompute();
  writeSpeed((int)Output);
  return 0;
}


int Motor::getPosition()
{
  return map(analogRead(_AnalogInput), 0, 1024, 0, 255);
}


//////////////////////////////////////////////
// Send motor speed by serial to sabertooth //
// Speed : between -127 and +127            //
//////////////////////////////////////////////

void Motor::writeSpeed(int Speed)
{
  _Speed = Speed;
  byte Command = (_Id == 2 ? 4 : 0) + (Speed < 0 ? 1 : 0);
  byte BSpeed = (byte)abs(Speed);

  Serial1.write((byte)128);       //Address
  Serial1.write(Command);  //command
  Serial1.write(BSpeed);     //Rate
  Serial1.write((128 + Command + BSpeed) & 0b01111111); //Required checksum with mask
}

void Motor::start()
{
  if (!inAuto)
  { /*we just went from manual to auto*/
    Initialize();
  }
  inAuto = AUTOMATIC;
}

void Motor::stop()
{
  writeSpeed(0);
  inAuto = MANUAL;
}

void Motor::setLimits(int H, int L)
{
  _HLimit = constrain(H, -127, 127);
  _LLimit = constrain(L, -127, 127);
}

void Motor::saveParameters()
{

}

bool Motor::PIDCompute()
{
  if (!inAuto) 
  {
    return false;
    }
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (timeChange >= SampleTime)
  {
    double error = Setpoint - Input;      //Get the error between desired and actual position
    
    ITerm += (Ki * error);
    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;

    double dInput = (Input - lastInput);

    /*Compute PID Output*/
    Output = Kp * error + ITerm - Kd * dInput;

    if (Output > outMax) Output = outMax;
    else if (Output < outMin) Output = outMin;

    /*Remember some variables for next time*/
    lastInput = Input;
    lastTime = now;
    return true;
  }
  else return false;
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void Motor::SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio  = (double)NewSampleTime
                    / (double)SampleTime;
    Ki *= ratio;
    Kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/
void Motor::Initialize()
{
  ITerm = myOutput;
  lastInput = myInput;
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
}


