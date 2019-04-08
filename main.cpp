#include "mbed.h"
#include <cmath>

#define nop 0x00            //no operation
#define rd_pos 0x10         //read position
#define set_zero_point 0x70 //set zero point

#define timeoutLimit 100

// PWM Initialization
// Servo-Motor
PwmOut servo_pwm(PA_8);

int main()
{
    // Set PWM



    // Blue Bird BMS -630MG
    // pulse width band: 1100us - 1900us //900us - 2100us
    // angle rotation: -60deg - 60deg
    // period: 8000us

    // Modelcraft RS2 MG/BB
    // pulse width band: 560us - 2460us
    // angle rotation: -90deg - 90deg
    // period: 20000us


    float s_period = 20.0;
    float s_duty_cycle = 0.0;
    float s_pulse_width = 0.0;

    const float s_pulse_width_min = 0.56;
    const float s_pulse_width_max = 2.46;

    float angle = 90.0;


    servo_pwm.period_ms(s_period);

    while (1) {
        /*
        for (s_pulse_width = s_pulse_width_min; s_pulse_width <= s_pulse_width_max ; s_pulse_width = s_pulse_width + 0.01f){
            s_duty_cycle = s_pulse_width / s_period;
            servo_pwm.write(s_duty_cycle);
            wait(0.05f);
            //pc.printf("Duty cycle: %3.2f \r\n", s_duty_cycle);
        }
        */

        angle = 0.0;
        s_pulse_width = (s_pulse_width_max - s_pulse_width_min) / 180.f * angle + s_pulse_width_min;
        s_duty_cycle = s_pulse_width / s_period;
        servo_pwm.write(s_duty_cycle);
        wait(1.0f);
        angle = 180.0;
        s_pulse_width = (s_pulse_width_max - s_pulse_width_min) / 180.f * angle + s_pulse_width_min;
        s_duty_cycle = s_pulse_width / s_period;
        servo_pwm.write(s_duty_cycle);
        wait(1.0f);

    }


}

