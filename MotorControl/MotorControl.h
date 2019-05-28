#ifndef MotorControl_H
#define MotorControl_H

/**
 * Includes
 */
#include "mbed.h"
#include "defines.h"
#include "PID.h"
#include "QEI.h"
/**
 * Defines
 */
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

/**
 * PID Motor Control Interface.
 */
class MotorControl: public QEI, protected PwmOut, DigitalOut, PID{

public:
    MotorControl(PinName motPin, PinName dirPin,            //Motor PWM pin, motor direction pin
                PinName qeiPinA, PinName qeiPinB,           // Encoder pin A, encoder pin B
                float Kc, float tauI, float tauD,           // Proportional , integral, derivative tuning parameter
                float interval, bool pidType);              // PID computation interval, PID type [0 = position, 1 = velocity]
    void zero_encoder(void);                                // Zero incremental encoder
    void set_mode(bool);                                    // Set motor and PID mode to on or off
    void update_set_point(float);                           // Update the PID set point
    void pid_control_processing(void);                      // Processing PID ticker ISR
    static bool get_pid_flag(void);                         // Get pid computation flag
    bool get_reached_flag(void);                            // Get setpoint reached flag
    float get_pwm_duty(void);
    float get_process_value(void);

protected:

    bool mode_;                                             // Motor and PID status
    bool point_reached_;                                    // Setpoint reached flag
    volatile float set_point_;                              // PID setpoint [encoder pulses (per second)]

private:

    void initialize_motor(void);                            // Initiallising motor pin
    void initialize_pid_controllers(void);                  // Set up PID controller with appropriate limits and biases
    static void pid_control(void);                          // PID controller ticker interrupt flag

    static Ticker           pidTick_;                       // PID call instance
    static bool pid_;                                       // PID computation flag

    volatile int pulses_;
    volatile int prevPulses_;
    volatile float pwmDuty_ ;
    volatile float processValue_;                           // [encoder pulses (per second)]
    volatile float direction_;                              // Motor direction vaule


    bool pidType_;                                          // PID control type [0 = position, 1 = velocity]

};

#endif /* MotorControl_H */
