#include "Arduino.h"
#include "Motor.h"
#include <PID_v1.h>
//#include <DueFlashStorage.h>

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
  else id = 1;
  
  //dfs.read(0);
  Serial1.begin(9600);
  
  writeSpeed(0);
  Kp = kp;
  Ki = ki;
  Kd = kd;
  _AnalogInput = AnalogInput;
  Setpoint = 0;
  Input = 0;
  Output = 0;
  _myPID.InitializePID(&Setpoint, &Input, &Output, Kp, Ki, Kd, DIRECT);
  _myPID.SetOutputLimits(-127,127);
  _myPID.SetSampleTime(SAMPLINGTIME);
}

void Motor::changePID(double kp, double ki, double kd)
{
  
  Kp = kp;
  Ki = ki;
  Kd = kd;
  _myPID.SetTunings(Kp, Ki, Kd);
}

void Motor::changePIDdirection()
{
  _myPID.SetControllerDirection(!_myPID.GetDirection());
}

void Motor::setSampleTime(int ST)
{
  _SampleTime = ST;
  _myPID.SetSampleTime(ST);
}

int Motor::getSampleTime()
{
  return _SampleTime;
}



int Motor::update(double setPoint)
{
  Setpoint = constrain(setPoint, _LLimit, _HLimit);
  Input = map(analogRead(_AnalogInput),0,1024,-127,127);
  _myPID.Compute();
  writeSpeed(Output);
  return 0;
}


int Motor::update()
{
  Input = map(analogRead(_AnalogInput),0,1024,-127,127);
  _myPID.Compute();
  writeSpeed(Output);
  return 0;
}


int Motor::getPosition()
{
  return map(analogRead(_AnalogInput),0,1024,0,255);
}
  
  
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
  _myPID.SetMode(AUTOMATIC);
}
  
void Motor::stop()
{
  writeSpeed(0);
  _myPID.SetMode(MANUAL);
}

void Motor::setLimits(int H, int L)
{
  _HLimit = constrain(H, -127, 127);
  _LLimit = constrain(L, -127, 127);
}

void Motor::saveParameters()
{
  
}
  

