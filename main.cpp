#include "mbed.h"
#include "PID_v1.h"
#include <cmath>

#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point

#define timeoutLimit 100

// PWM Initialization
// DC-Motor
double period = 2.0;
double duty_cycle = 0;
DigitalOut direction(PA_9);
PwmOut motor_pwm(PC_7);



//------------------------------------
// Hyperterminal configuration
// 9600 bauds, 8-bit data, no parity
//------------------------------------
// Serial Initialization
Serial pc(SERIAL_TX, SERIAL_RX);

// SPI Initialization
SPI device(SPI_MOSI, SPI_MISO, SPI_SCK);
DigitalOut chip_sel(SPI_CS);

// PID Controller
double Setpoint = 0;
int point = 1500;
double Input = 0;
double Output = 0;
//Define Tuning Parameters

//aggressive
double aggKp=0.9;
double aggKi=0.1;
double aggKd=0.02;

//conservative
double consKp=0.015;//0.015
double consKi=0.001;//0.035;
double consKd=0.015;//0.015

uint8_t stage=0;


PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, REVERSE);

uint8_t SPIWrite(uint8_t sendByte);

int encodervalue(void);

void links(void);
void rechts(void);
void stopp(void);


int main()
{
    // Initialize PID
    myPID.SetMode(AUTOMATIC);
    myPID.SetSampleTime(1);
    myPID.SetControllerDirection(REVERSE);
    //myPID.SetOutputLimits(11,100);

    // Pull up CS line
    chip_sel = 1;

    // Set PWM
    motor_pwm.period_ms(period);
    motor_pwm.write(duty_cycle);

    while (1) {
        int enc = encodervalue();
          if (abs(enc - point) > 100)// && stage!=0)
          {
            stage = 0;
            myPID.SetOutputLimits(0,100);
            myPID.SetTunings(aggKp, aggKi, aggKd);
          }
          else if (abs(enc - point) > 30) // && stage!=1)
          {
            stage = 1;
            myPID.SetOutputLimits(0,100);
            myPID.SetTunings(consKp, consKi, consKd);
          }
          else if (abs(enc - point) > 5)// && stage!=2)
          {
            stage = 2;
            myPID.SetOutputLimits(0,20);
            myPID.SetTunings(consKp, consKi, consKd);
          }
          else //if (stage!=3)
          {
            stage = 3;
            //Serial.print('Reached');
            //Serial.write("\n");
            myPID.SetOutputLimits(0,20);
            myPID.SetTunings(0.0, 0.0, 30.0);
          }

        if(enc < 1024)
        {
          duty_cycle = 0.2f;
          links();
        }
        else if(enc > 3072)
        {
          duty_cycle = 0.2f;
          rechts();
        }



        else //if(enc >= 1024 && enc <= 3072)
        {
          Input = (double)(abs(enc - point));
          myPID.Compute();

          duty_cycle = Output/255.0f;
        }

        pc.printf("Encoder value: %d \t Error value: %d \t Output: %3.2f \t Velocity: %3.2f \r\n", enc, (enc-point), Output, duty_cycle);

        if((enc - point) > 0)
        {
        rechts();
        }
        else
        {
        links();
        }

    }
}

uint8_t SPIWrite(uint8_t sendByte)
{
  //holder for the received over SPI
  uint8_t data;

  //the AMT20 requires the release of the CS line after each byte
  chip_sel = 0;
  data = device.write(sendByte);
  chip_sel = 1;

  //we will delay here to prevent the AMT20 from having to prioritize SPI over obtaining our position
  wait_us(10);

  return data;
}



int encodervalue()//sollte am besten in einer loop verwendet werden
{
    uint8_t data;               //this will hold our returned data from the AMT20
    uint8_t timeoutCounter;     //our timeout incrementer
    uint16_t currentPosition;   //this 16 bit variable will hold our 12-bit position

    int rueckgabe=0;
    timeoutCounter = 0;
    data = SPIWrite(rd_pos);

    while (data != rd_pos && timeoutCounter++ < timeoutLimit)
    {
      data = SPIWrite(nop);
    }


    if (timeoutCounter < timeoutLimit)
    {
      currentPosition = (SPIWrite(nop)& 0x0F) << 8;
      currentPosition |= SPIWrite(nop);
    }
    else
    {
      pc.printf("Error obtaining position.\r\n");
      pc.printf("Reset Arduino to restart program.\r\n");

      while(true);
    }


    //simply gives out the encoder value in you favourite data Type, we use DEC in another Part of the code
    //pc.printf("Current position: ");
    //pc.printf(currentPosition, DEC);
    //pc.printf("\t 0x");
    //pc.printf(currentPosition, HEX);
    //pc.printf("\t 0b");
    //pc.printf(currentPosition, BIN);
    //pc.printf("\n");

    wait_us(100);
    rueckgabe = currentPosition;
    return rueckgabe;
}

void links()
{
direction = 1;
motor_pwm.write(duty_cycle);
}

void rechts()//uhrzeigersinn clockwise etc
{
direction = 0;
motor_pwm.write(duty_cycle);
}

void stopp ()
{
motor_pwm.write(0.0f);
}
