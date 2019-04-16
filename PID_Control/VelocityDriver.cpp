#include "VelocityDriver.h"

PwmOut leftMotor(PA_1);
DigitalOut leftDirection(PA_2);
QEI leftQei(PB_11, PB_10, NC, encoder_range, QEI::X4_ENCODING);

PID leftVelocityController(Kc, Ti, Td, RATE);

//--------------------
// Working variables.
//--------------------
volatile int leftPulses     = 0;
volatile int leftPrevPulses = 0;
volatile float leftPwmDuty  = 1.0;
volatile float leftVelocity = 0.0;

void initializeMotors(void){

    leftMotor.period_us(2000);
    leftMotor = 0.0;
    leftDirection = 1.0;

}

void initializePidControllers(void){

    // Velocity PID controller
    leftVelocityController.setInputLimits(0.0, 1080.0);
    leftVelocityController.setOutputLimits(-0.5, 0.5);
    leftVelocityController.setBias(0.0);
    leftVelocityController.setMode(AUTO_MODE);

}

void computePIDLeftVelocity(float goal_velocity) {

    leftDirection = goal_velocity >= 0 ? 1.0 : 0.0;
    goal_velocity = goal_velocity >= 0 ? goal_velocity : -goal_velocity;

    leftVelocityController.setSetPoint(goal_velocity);
    leftPulses = leftQei.getPulses();
    leftVelocity = float(((leftPulses - leftPrevPulses) / float(RATE)) / float(encoder_range) * 360.0);
    leftPrevPulses = leftPulses;
    leftVelocityController.setProcessValue(leftVelocity >= 0 ? leftVelocity : -leftVelocity);
    leftPwmDuty = leftVelocityController.compute();

    leftMotor = leftPwmDuty;
}
