/**
 * Includes
 */
#include "MotorControl.h"

/**
 * Functions
 */
MotorControl::MotorControl(PinName motPin, PinName dirPin, PinName qeiPinA, PinName qeiPinB,
                            float Kc, float tauI, float tauD, float interval, bool pidType,
                            int PULSES_PER_REV) : QEI(qeiPinA, qeiPinB, NC, PULSES_PER_REV,          // Important 5V tollerant interrupt pins
                            QEI::X4_ENCODING), PwmOut(motPin), DigitalOut(dirPin),
                            PID(Kc, tauI, tauD, RATE) {
    // Initialize working variables
    mode_ = 0;
    point_reached_ = 1;
    set_point_ = 0;
    pulses_ = 0;
    revolutions_ = 0;
    prevPulses_ = 0;
    prevRevolutions_ = 0;
    pwmDuty_  = 0.0;
    processValue_ = 0.0;
    direction_ = 0.0;

    pidType_ = pidType;
    initialize_motor();
    initialize_pid_controllers();

    // Adjust precision flag for point reached feedback
    if (pidType_){
        // Velocity control
        rp_ = Vel_RP;
    }
    else{
        // Position control
        rp_ = Pos_RP;
    }
}

void MotorControl::pid_control_processing(void){
    // Get pulses and revolutions and set process variable
    pulses_ = QEI::getPulses();
    revolutions_ = QEI::getRevolutions();
    if (pidType_){
        // Velocity control
        processValue_ = (float((revolutions_ - prevRevolutions_) * PULSES_PER_REV_) + float(pulses_ - prevPulses_)) / RATE;
        prevPulses_ = pulses_;                              // Set previous pulses
        prevRevolutions_ = revolutions_;                    // Set previous revolutions
    }
    // Position control
    else processValue_ = float(pulses_);
    // Check if setpoint has been reached
    if (!point_reached_ && processValue_-set_point_ < rp_ && processValue_-set_point_ > -rp_ ) point_reached_ = 1;

    // Compute PID and set motor
    if (mode_) {
        if (pidType_){
            // Velocity control
            PID::setProcessValue(processValue_ >= 0 ? processValue_ : -processValue_);
            pwmDuty_ = PID::compute();
        }
        else{
            // Position control
            PID::setProcessValue(processValue_);
            pwmDuty_ = PID::compute();
            direction_ = pwmDuty_ >= 0 ? 1.0 : 0.0;
            pwmDuty_ = pwmDuty_ >= 0 ? pwmDuty_ : -pwmDuty_;
        }
        DigitalOut::write(direction_);
        PwmOut::write(pwmDuty_);
    }
}

void MotorControl::zero_encoder(void){
    // Set encoder value to zero
    QEI::reset();
    bool stop = 0;
    int enc_val = QEI::getPulses();
    int prev_enc = enc_val + PULSES_PER_REV_;
    // Set motor direction opposite on both sides
    #if defined(FRONT_LEFT) || defined(REAR_LEFT)
    // Left motor controller
        DigitalOut::write(0.0);
        PwmOut::write(0.15);
    #endif
    #if defined(REAR_RIGHT) || defined(FRONT_RIGHT)
    // Right motor controller
        DigitalOut::write(1.0);
        PwmOut::write(0.15);
    #endif
    wait(RATE);
    // Continously check for hard stop
    while(!stop) {
        wait(RATE);
        // If hard stop is reached
        if (enc_val == prev_enc) {
            PwmOut::write(0.0);                         // Turn off motor
            stop = 1;
        } else {
            // Get encoder values
            prev_enc = enc_val;
            enc_val = QEI::getPulses();
        }
    }
    // Set Encoder value to zero
    QEI::reset();
}

void MotorControl::set_mode(bool mode){
    if (mode_ != mode){
        mode_ = mode;                                       // Set motor and PID mode flag
        // Position control
        PID::setMode(mode_);                                // Set PID control to manual or automatic
        if (!mode_) {
            PwmOut::write(0.0);                             // Turn off steering motor
        }
    }
}

void MotorControl::update_set_point(float set_point){
    set_point_ = set_point;
    // Apply set point
    if (pidType_){
        // Velocity control
        if (set_point_ > 0) direction_ = 1.0;
        else if (set_point_ < 0){
            direction_ = 0.0;
            set_point_ = -set_point_;
        }
    }
    PID::setSetPoint(set_point_);
    point_reached_ = 0;
}

void MotorControl::initialize_motor(void){
    PwmOut::period(0.00005);
    PwmOut::write(0.0);
    DigitalOut::write(1.0);
}

void MotorControl::initialize_pid_controllers(void){
    if (pidType_){
        // Velocity PID controller
        PID::setInputLimits(0.0, MaxVelEnc);
        PID::setOutputLimits(0.0, 0.9);
        PID::setBias(0.0);
        PID::setMode(AUTO_MODE);
    }
    else{
        // Position PID controller
        PID::setInputLimits(-50.0, MaxOrEnc + 50.0);
        PID::setOutputLimits(-0.2, 0.2);
        PID::setBias(0.0);
        PID::setMode(AUTO_MODE);
    }
}

bool MotorControl::get_reached_flag(void){
    return point_reached_;
}

float MotorControl::get_pwm_duty(void){
    return pwmDuty_;
}

float MotorControl::get_process_value(void){
    return processValue_;
}
