#ifndef MotorControl_H
#define MotorControl_H

/**
 * Includes
 */
#include "mbed.h"
//#include "rtos.h"
#include "defines.h"
#include "PID.h"
#include "QEI.h"

/**
 * PID Motor Control Interface.
 */
class MotorControl: public QEI, protected PwmOut, DigitalOut, PID{

public:
    MotorControl(PinName motPin, PinName dirPin,            //Motor PWM pin, motor direction pin
                PinName qeiPinA, PinName qeiPinB,           // Encoder pin A, encoder pin B
                float Kc, float tauI, float tauD,           // Proportional , integral, derivative tuning parameter
                float interval, bool pidType,               // PID computation interval, PID type [0 = position, 1 = velocity]
                int PULSES_PER_REV);                        // Encoder pulses per revolution
    void zero_encoder(void);                                // Zero incremental encoder
    void set_mode(bool);                                    // Set motor and PID mode to on or off
    void update_set_point(float);                           // Update the PID set point
    void pid_control_processing(void);                      // Processing PID ticker ISR
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

    volatile int pulses_;
    volatile int revolutions_;
    volatile int prevPulses_;
    volatile int prevRevolutions_;
    volatile float pwmDuty_ ;
    volatile float processValue_;                           // [encoder pulses (per second)]
    volatile float direction_;                              // Motor direction value
    int rp_;                                                 // Precision for triggering point reached flag
    int PULSES_PER_REV_;

    bool pidType_;                                          // PID control type [0 = position, 1 = velocity]

};



#endif /* MotorControl_H */
