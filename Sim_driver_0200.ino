#include <PID_v1.h>
#include "Motor.h"


double dKp = 5; 
double dKi = 0; 
double dKd = 0;

Motor motor1(1, dKp, dKi, dKd, A0);
Motor motor2(2, dKp, dKi, dKd, A1);

enum state{
  INITIALIZATION,
  SIMULATION,
  CONFIGURATION
};

enum confstate{
  RECEPTION,
  SETSETPOINT,
  SETPID,
  MEASURE,
  SETSAMPLETIME,
  GETSAMPLETIME,
  SETLIMITS,
  GETLIMITS,
  GETPID,
  EXIT
};


state State = INITIALIZATION;
confstate cmd = RECEPTION;


void setup() 
{
  SerialUSB.begin(115200);


  while(!SerialUSB.available());

}

void loop() 
{
  if (SerialUSB.available()>=4)
  {
    while(SerialUSB.available())
    {
      int FirstByte = SerialUSB.read();
      int SecondByte = SerialUSB.read();

      switch (FirstByte+SecondByte)
      {
      case 2 : 
        {
          State = SIMULATION;

          int Consigne1 = SerialUSB.read();
          motor1.Setpoint = map(Consigne1, 0, 255, 65, 190);
          int Consigne2 = SerialUSB.read();
          motor2.Setpoint = map(Consigne2, 0, 255, 65, 190);

          break;
        }

      case 4 :
        State = CONFIGURATION;
        configurationMode();
        break;

      default :
        serialFlush();
        break;
      }
    }
  } 

  motor1.update();
  motor2.update();
}

void serialFlush()
{
  while(SerialUSB.available())
  {
    SerialUSB.read();
  }
}

void configurationMode()
{
  serialFlush();
  while(State = CONFIGURATION)
  {
    if (SerialUSB.available()>=2)
    {
      int FB = SerialUSB.read();
      int SB = SerialUSB.read();
      cmd = (confstate)((FB + SB)/2);
      switch (cmd)
      {
      case SETSETPOINT:
        setpoint();
        serialFlush();
        break;

      case SETPID:
        setPID();
        serialFlush();
        break;

      case MEASURE:
        startMeasure();
        serialFlush();
        break;

      case SETSAMPLETIME:
        setSampleTime();
        serialFlush();
        break;

      case GETSAMPLETIME:
        getSampleTime();
        serialFlush();
        break;

      case SETLIMITS:

        serialFlush();
        break;

      case GETLIMITS:

        serialFlush();
        break;

      case GETPID:

        serialFlush();
        break;

      case EXIT:
        serialFlush();
        State = SIMULATION;
        break;

      default:    
        serialFlush();
        break;
      }
    }

    motor1.update(3);
    motor2.update(3);
  }
}


///////////////////////////////////////////////////////////////////////////////
// Set a new setpoint is usefull for manual control => Min/Max limit setting //
///////////////////////////////////////////////////////////////////////////////

void setpoint()
{
  int timeout = 0;
  while (SerialUSB.available() < 2 && timeout < 500) // while there is not 2 bytes in RX buffer we wait, timeout value is 500ms
  {
    delay(1);
    timeout++;
  }

  if (timeout < 500)
  {
    int Consigne1 = SerialUSB.read();
    int Consigne2 = SerialUSB.read();

    motor1.update(Consigne1);
    motor2.update(Consigne2);

  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function to get New PID settings from serial : values should be Kp1Ki1Kd1Kp2Kd2 each xxxx ==> x,xxx //
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void setPID()                                      
{
  int timeout = 0;
  while (SerialUSB.available() < 24 && timeout < 500) // while there is not 24 bytes in RX buffer we wait, timeout value is 500ms
  {
    delay(1);
    timeout++;
  }

  if (timeout < 500)
  {
    double Kp = (double)(((byte)SerialUSB.read()-48)+((byte)SerialUSB.read()-48)/10.0+((byte)SerialUSB.read()-48)/100.0+((byte)SerialUSB.read()-48)/1000.0);
    double Ki = (double)(((byte)SerialUSB.read()-48)+((byte)SerialUSB.read()-48)/10.0+((byte)SerialUSB.read()-48)/100.0+((byte)SerialUSB.read()-48)/1000.0);
    double Kd = (double)(((byte)SerialUSB.read()-48)+((byte)SerialUSB.read()-48)/10.0+((byte)SerialUSB.read()-48)/100.0+((byte)SerialUSB.read()-48)/1000.0);

    motor1.Kp = Kp;
    motor1.Ki = Ki;
    motor1.Kd = Kd;

    Kp = (double)(((byte)SerialUSB.read()-48)+((byte)SerialUSB.read()-48)/10.0+((byte)SerialUSB.read()-48)/100.0+((byte)SerialUSB.read()-48)/1000.0);
    Ki = (double)(((byte)SerialUSB.read()-48)+((byte)SerialUSB.read()-48)/10.0+((byte)SerialUSB.read()-48)/100.0+((byte)SerialUSB.read()-48)/1000.0);
    Kd = (double)(((byte)SerialUSB.read()-48)+((byte)SerialUSB.read()-48)/10.0+((byte)SerialUSB.read()-48)/100.0+((byte)SerialUSB.read()-48)/1000.0);

    motor2.Kp = Kp;
    motor2.Ki = Ki;
    motor2.Kd = Kd;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start of a measure (step response) Message to be sent is 1 for X motor, 2 for Y motor, 3 for Both   //
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void startMeasure()
{
  int timeout = 0;
  unsigned long time = millis();
  while (!SerialUSB.available() && timeout < 500) // while there is not 24 bytes in RX buffer we wait, timeout value is 500ms
  {
    delay(1);
    timeout++;
  }

  if (timeout < 500)
  {
    switch  (SerialUSB.read())
    {
    case 1:
      motor1.update(50);
      while (millis() - time < 1000)
      {
        motor1.update();
        time = millis();
      }

      motor1.update(204);
      while (millis() - time < 1000)
      {
        motor1.update();
        time = millis();
        delay(1);
        SerialUSB.print(motor1.getPosition());
      }

      break;

    case 2:
      motor2.update(50);
      while (millis() - time < 1000)
      {
        motor2.update();
        time = millis();
      }

      motor2.update(204);
      while (millis() - time < 1000)
      {
        motor2.update();
        time = millis();
        delay(1);
        SerialUSB.print(motor1.getPosition());
      }

      break;

    case 3:
      motor1.update(50);
      motor2.update(50);
      while (millis() - time < 1000)
      {
        motor1.update();
        motor2.update();
        time = millis();
      }

      motor1.update(204);
      motor2.update(204);
      while (millis() - time < 1000)
      {
        motor1.update();
        motor2.update();
        time = millis();
        delay(1);
        SerialUSB.print(motor1.getPosition());
        SerialUSB.print(motor2.getPosition());
      }

      break;

    default:
      break;
    }
  }
}

//////////////////////////////////////////////////////////////
// Set the new sampletime two Bytes shall be sended (1 Int) //
//////////////////////////////////////////////////////////////

void setSampleTime()                                      
{
  int timeout = 0;
  while (SerialUSB.available() < 2 && timeout < 500) // while there is not 24 bytes in RX buffer we wait, timeout value is 500ms
  {
    delay(1);
    timeout++;
  }

  if (timeout < 500)
  {
    byte bst = SerialUSB.read();
    int st = ((int)bst)<<8;
    bst = SerialUSB.read();
    st = st +(int)bst;

    motor1.setSampleTime(st);
    motor2.setSampleTime(st);
  }
}

//////////////////////////////////////////////////////////////////
// Get the current sampletime two Bytes shall be sended (1 Int) //
//////////////////////////////////////////////////////////////////

void getSampleTime()                                      
{
  SerialUSB.print(motor1.setSampleTime());
  SerialUSB.print(motor2.setSampleTime());
}

















