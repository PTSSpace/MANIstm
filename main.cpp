/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "stm32f103c8t6.h"

#include "PID.h"
#include "QEI.h"


#define pulsesPerRev 8384
#define pulsesPerHalf 4192

//****************************************************************************/
// Defines - PID
//****************************************************************************/

// Loop rate
#define  RATE 0.01

// PID values from tuning
// Velocity control
#define  Vel_Kc 0.1
#define  Vel_Ti 0.35    //expandable
#define  Vel_Td 0.0
// Position control
#define Pos_Kc    180.0//25.0 //3.25 //0.9 // 3.25
#define Pos_Ti    0.0035//0.008 //0.45 //0.0015//expandable //0.65
#define Pos_Td    0.03//0.02 //0.0 //0.02

// Parameter input
#define SET_VELOCITY 40.0       // deg per second
#define SET_POSITION 90.0       // deg

#define  MAX_VELOCITY 3000.0    // pulses per second

float set_velocity;             // pulses per second
float set_position;             // pulses per second

Timer           timer;

//****************************************************************************/
// Defines - UART Serial
//****************************************************************************/
Serial pc(PA_2, PA_3);


//--------------------
// Working variables.
//--------------------
// Velocity control
volatile int drivePulses     = 0;
volatile int drivePrevPulses = 0;
volatile float drivePwmDuty  = 0.0;
volatile float driveVelocity = 0.0;

PwmOut DriveMotor(PA_6);
DigitalOut DriveDirection(PA_7);
QEI DriveQei(PA_11, PA_12, NC, pulsesPerRev, QEI::X4_ENCODING);     // TODO Adjust pins to not overlapp with CAN (important 5V tollerant interrupt)
PID VelocityController(Vel_Kc, Vel_Ti, Vel_Td, RATE);


// Position control
volatile int steerPulses     = 0;
volatile float steerPwmDuty  = 0.0;
volatile float steerPosition = 0.0;

PwmOut SteerMotor(PB_0);
DigitalOut SteerDirection(PB_1);
QEI SteerQei(PA_9, PA_10, NC, pulsesPerRev, QEI::X4_ENCODING);
PID PositionController(Pos_Kc, Pos_Ti, Pos_Td, RATE);


//****************************************************************************/
// Initialization
//****************************************************************************/
//Set motors to go "forward", brake off, not moving.
void initializeMotors(void);
//Set up PID controllers with appropriate limits and biases.
void initializePidControllers(void);

//****************************************************************************/
// Main
//****************************************************************************/
// main() runs in its own thread in the OS
int main()
{
    confSysClock();     //Configure the system clock (72MHz HSE clock, 48MHz USB clock)
    pc.baud(115200);

    //Initialization.
    initializeMotors();
    initializePidControllers();
    int count = 0;
    timer.start();
    pc.printf("Start PID \r\n");

    // Calculate from deg to pulses
    set_velocity = SET_VELOCITY / 360.0 * float(pulsesPerRev);
    set_position = SET_POSITION  / 360.0 * float(pulsesPerRev);

    // Apply set points
    // Velocity control
    VelocityController.setSetPoint(set_velocity >= 0 ? set_velocity : -set_velocity);
    // Position control
    PositionController.setSetPoint(set_position);
    while (true) {

        // Compute PID
        if ( timer.read() >= RATE){

            // Get Pulses, Compute PID and set drive motors
            // Velocity control
            drivePulses = DriveQei.getPulses();
            driveVelocity = float(drivePulses - drivePrevPulses) / RATE;
            VelocityController.setProcessValue(driveVelocity >= 0 ? driveVelocity : -driveVelocity);
            drivePwmDuty = VelocityController.compute();
            DriveDirection = set_velocity >= 0 ? 1.0 : 0.0;
            DriveMotor = drivePwmDuty;
            // Position control
            steerPulses = SteerQei.getPulses();
            steerPosition = float(steerPulses);
            PositionController.setProcessValue(steerPosition);
            steerPwmDuty = PositionController.compute();
            SteerDirection = steerPwmDuty >= 0 ? 1.0 : 0.0;
            steerPwmDuty = steerPwmDuty >= 0 ? steerPwmDuty : -steerPwmDuty;
            SteerMotor = steerPwmDuty;

            // Reset Timer
            timer.reset();

            // UART serial output
            count ++;
            if (count > 10) {
                pc.printf("D-Goal: %f, \t D-Current: %f, \t D-PWM: %f , \t P-Goal: %f, \t P-Current: %f, \t P-PWM: %f \r\n", set_velocity, driveVelocity, drivePwmDuty, set_position, steerPosition, steerPwmDuty);
                count = 0;
            }

            // Set previous pulses
            drivePrevPulses = drivePulses;

        }

    }

    timer.stop();
    //Stop motors.

}






void initializeMotors(void){
    // Velocity control
    DriveMotor.period(0.00005);
    DriveMotor = 0.0;
    DriveDirection = 1.0;
    // Position control
    SteerMotor.period(0.00005);
    SteerMotor = 0.0;
    SteerDirection = 1.0;

}

void initializePidControllers(void){
    // Velocity PID controller
    VelocityController.setInputLimits(0.0, MAX_VELOCITY);
    VelocityController.setOutputLimits(0.0, 0.9);
    VelocityController.setBias(0.0);
    VelocityController.setMode(AUTO_MODE);
    // Position PID controller
    PositionController.setInputLimits(-50.0, float(pulsesPerRev) + 50.0);
    PositionController.setOutputLimits(-0.2, 0.2);
    PositionController.setBias(0.0);
    PositionController.setMode(AUTO_MODE);

}
