#include "main.h"


//****************************************************************************/
// Main - Implementation
//****************************************************************************/

// Main function
int main(void)
{
    // Pull down Can Standby pin
    canSTB = 0;
    //Initialization
    #if defined(DEBUG)
        initializeSerial();
    #endif

    commandThread.start(callback(&com_queue, &EventQueue::dispatch_forever));
    initialize_can_bus();

    #if defined(DEBUG)
        pc.printf("Start PID \r\n");
    #endif

    // Message variables
    int data[2] = {0,0};

    while (true) {
         if (zeroEncoder) {
            #if defined(DEBUG)
                pc.printf("Zeroing %s orientaton \r\n", wheelIndex);
                pc.printf("Start searching for hard stop\r\n");
            #endif
            PositionControl.zero_encoder();
            #if defined(DEBUG)
                pc.printf("Zeroing complete - hard stop reached\r\n");
                pc.printf("Enabling motor control - setting PID to auto\r\n");
            #endif
            zeroEncoder = 0;
        }
        //-----------------------------------------/
        // CAN Bus
        //-----------------------------------------/
        // Check for received message
        // Publish locomotion task accomplished message
        if (PositionControl.get_reached_flag() && ortFeed){
            data[0] = 1;                                 // Steer encoder pulses  [0..MaxValue]
            // Send reached orientation feedback
            can_publisher_processing(LC_FD, data, 1, 0);
            if (msg_sent) ortFeed = 0;                                                  // Clear orientation feedback flag
            #if defined(DEBUG)
                if(msg_sent) {
                        pc.printf("-------------------------------------\r\n");
                        pc.printf("CAN orientation reached sent\r\n");
                }
                else pc.printf("Transmission error\r\n");
            #endif
        }
        if (VelocityControl.get_reached_flag() && velFeed){
            data[0] = 1;
            // Send reached orientation feedback
            can_publisher_processing(LC_FD, data, 1, 0);
            if (msg_sent) velFeed = 0;                                                  // Clear orientation feedback flag
            #if defined(DEBUG)
                if(msg_sent) {
                        pc.printf("-------------------------------------\r\n");
                        pc.printf("CAN velocity reached sent\r\n");
                }
                else pc.printf("Transmission error\r\n");
            #endif
        }

        // Publish locomotion odometry message
		if (!msg_sent){
            // Get drive encoder data
            data[0] = VelocityControl.QEI::getPulses();                                 // Drive encoder pulses  [0..pulsesPerRevl]
            data[1] = VelocityControl.QEI::getRevolutions();                            // Drive encoder revolutions  [0..MaxValue]
            // Send message
			can_publisher_processing(OM_FD, data, 2, 1);
            #if defined(DEBUG)
                if(msg_sent) {
                    pc.printf("-------------------------------------\r\n");
                    pc.printf("CAN odometry sent\r\n");
                    pc.printf("Pulses = %d \r\n", data[0]);
                    pc.printf("Revolutions = %d \r\n", data[1]);
                }
                else pc.printf("Transmission error\r\n");
            #endif
        }

        //-----------------------------------------/
        // PID Position/Velocity Control
        //-----------------------------------------/
		if (MotorControl::get_pid_flag()){
            PositionControl.pid_control_processing();
    		VelocityControl.pid_control_processing();

            #if defined(DEBUG)
                    // UART serial output
                    count ++;
                    if (count >= 500) {
                        if(steerMode) pc.printf("O-Goal: %d, \t O-Current: %d, \t O-PWM: %d \r\n", int(set_orientation),
                                        int(PositionControl.get_process_value()), int(PositionControl.get_pwm_duty()*1000.0));
                        if(driveMode) pc.printf("V-Goal: %d, \t V-Current: %d, \t V-PWM: %d \r\n", int(set_velocity),
                                        int(VelocityControl.get_process_value()), int(VelocityControl.get_pwm_duty()*1000.0));
                        count = 0;
                    }
                #endif
		}
    }
}



//****************************************************************************/
// Functions - Implementation
//****************************************************************************/

//-----------------------------------------/
// UART Serial
//-----------------------------------------/
#if defined(DEBUG)
    void initializeSerial(void){
        pc.baud(115200);                                                     // Set serial speed
        led = 0;                                                             // Turn off LED for message feedback
        pc.printf("CAN Control %s board \r\n", wheelIndex);
    }
#endif

//-----------------------------------------/
// CAN Bus
//-----------------------------------------/
void initialize_can_bus(void){
    // CAN Interface
    can.mode(CAN::Normal);
    can.frequency(FREQUENCY);                                               // Set CAN bit rate
    can.filter(S_CMD, MASK, CANStandard, 0);                                // Set filter #0 to accept only standard messages with ID in MASK range
    can.attach(&can_received, CAN::RxIrq);                                  // Attach ISR to handle received messages
    pubTick.attach(&can_publisher, PUB_RATE);                               // Attach callback to ticker

}

void can_publisher(void){
    if (publisherMode) msg_sent = 0;
}

// CAN interrupt
// IMPORTANT: CAN::read must not contain mutex locking
void can_received(void)
{
    // Read message from CAN bus
    can.read(rxMsg);
    // Set message received flag
    //msg_received = 1;
    // Process message outside ISR
    com_queue.call(&can_command_processing);
}

// Processing thread outside of interrupt
void can_command_processing(void)
{
    #if defined(DEBUG)
        pc.printf("-------------------------------------\r\n");
        pc.printf("CAN received %d\r\n", rxMsg.id);
    #endif
    // Check for correct message ID
    if (rxMsg.id == S_CMD){
        // Extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        driveMode = ci.byte_to_int(rxMsg.data, 0, 1);               // Read first four bytes (driveMode  [0, 1])
        VelocityControl.set_mode(driveMode);
        steerMode = ci.byte_to_int(rxMsg.data, 1, 1);               // Read second four bytes (steerMode  [0, 1])
        PositionControl.set_mode(steerMode);
        // Extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        publisherMode = ci.byte_to_int(rxMsg.data, 2, 1);           // Read first four bytes (publisher  [0, 1])
        #if defined(DEBUG)
            pc.printf("Drive mode set to %d\r\n", driveMode);
            pc.printf("Steer mode set to %d\r\n", steerMode);
            pc.printf("Set publisher mode %d\r\n", publisherMode);
        #endif
        zeroEncoder = ci.byte_to_int(rxMsg.data, 3, 1);             // Read second four bytes (zeroEncoders  [0, 1])
    }
    else if (rxMsg.id == O_CMD) {
        // Extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        int orientation = ci.byte_to_int(rxMsg.data, 0, 4);         // Read first four bytes (velocity scaled [0..MaxValue])
        // Update set point for PID control
        set_orientation = ci.msg_to_enc(orientation, 1);            // orientation [0..EncOrMax]
        // Apply orientation set point
        PositionControl.update_set_point(set_orientation);
        ortFeed = 1;                                                //Set orientation feedback flag
        #if defined(DEBUG)
            pc.printf("Set orientation %d\r\n", orientation);
        #endif
    }
    else if (rxMsg.id == V_CMD) {
        // Extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        int velocity = ci.byte_to_int(rxMsg.data, 0, 4);            // Read first four bytes (velocity scaled [0..MaxValue])
        // Update set point for PID control
        set_velocity = ci.msg_to_enc(velocity, 0);                  // velocity [0..EncOrMax]
        // Apply velocity set point
        VelocityControl.update_set_point(set_velocity);
        velFeed = 1;                                                //Set velocity feedback flag
        #if defined(DEBUG)
            pc.printf("Set velocity %d\r\n", velocity);
        #endif
    }
}

void can_publisher_processing (uint8_t ID, int value[], int num_values, int type){
    txMsg.id = ID;                                                      // Set ID (11 bit)
    if (type == 0){
        txMsg.len = num_values;
        for (int i = 0 ; i<num_values; i++){
            if(value[i] != 0)
                txMsg.data[i] = 1;
        }
    }
    else {
        txMsg.len = num_values * 4;                                         // Length of transfered data [byte]
        ci.int_to_byte(txMsg.data, value, num_values);
    }
    // Write message to CAN bus
    msg_sent = can.write(txMsg);
}
