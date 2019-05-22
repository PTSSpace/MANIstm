#include "main.h"


//****************************************************************************/
// Main - Implementation
//****************************************************************************/

// Main function
int main(void)
{
    //confSysClock();     														//Configure the system clock (72MHz HSE clock, 48MHz USB clock)

    //Initialization
    #if defined(DEBUG)
        initializeSerial();
    #endif
    initializeCanBus();
    initializeMotors();
    initializePidControllers();
    initializePidControllers();

    Thread commandThread(osPriorityHigh);
    //Thread publisherThread(osPriorityNormal);
    //Thread pidcontrolThread(osPriorityNormal);


    commandThread.start(callback(&com_queue, &EventQueue::dispatch_forever));
    //publisherThread.start(callback(&pub_queue, &EventQueue::dispatch_forever));
    //pidcontrolThread.start(callback(&pid_queue, &EventQueue::dispatch_forever));

    setPublisherMode();

    // PID update timer
    pidTick.attach(&pid_control, RATE);
    #if defined(DEBUG)
        pc.printf("Start PID \r\n");
    #endif

    while (true) {
        //-----------------------------------------/
        // CAN Bus
        //-----------------------------------------/
        // Check for received message
        /*
        if(msg_received){
            msg_received = 0;
            can_command_processing();
        }
        */
        if (zeroing_encoder){
            #if defined(DEBUG)
                pc.printf("Zeroing %s orientaton \r\n", wheelIndex);
            #endif
            // Start zeroing
            zeroEncoders();
        }
        // Publish odometry message
		if (!msg_sent){
			can_publisher_processing();
		}
		if (pid){
    		pid_control_processing();
    		pid = 0;
		}

        //-----------------------------------------/
        // PID Position/Velocity Control
        //-----------------------------------------/


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
        pc.baud(115200);                                                        // Set serial speed
        led = 0;                                                                // Turn off LED for message feedback
        pc.printf("CAN Control %s board \r\n", wheelIndex);
    }
#endif

//-----------------------------------------/
// CAN Bus
//-----------------------------------------/
void initializeCanBus(void){
    // CAN Interface
    can.mode(CAN::Normal);
    can.frequency(500000);                                                      // Set CAN bit rate to 1Mbps
    can.filter(S_CMD, MASK, CANStandard, 0);                                   // Set filter #0 to accept only standard messages with ID in MASK range

    can.attach(&can_received, CAN::RxIrq);                                      // Attach ISR to handle received messages
}

void setPublisherMode(void){
    if (!publisherMode){
        // TODO interrupt priority
        pubTick.attach(&can_publisher, PUB_RATE);                             // Attach callback to ticker
        publisherMode = 1;
    }
    else {
        pubTick.detach();                                                     // Detetch callback from ticker
        publisherMode = 0;
    }
    #if defined(DEBUG)
        pc.printf("Publisher switched to %d \r\n", publisherMode);
    #endif
}

// Convert from message unit [0..MaxValue] to encoder unit [pulses]
float msg_to_enc(int value, bool type){
	// Scale back to decimal number [0..1]
	float dec = float(value) /MaxValue;
	float enc = .0;
    if (type){
    	// Convert orientation to [encoder pulses]
        enc = (dec + 1.0) *(MaxOrEnc/2.0);
	}
	else{
        // Convert velocity to [encoder pulses per second]
        enc = dec *MaxVelEnc;
	}
	return enc;
}

// Convert from encoder encoder unit [pulses] to message unit [0..MaxValue]
float enc_to_msg(float value, bool type){
    float nom = .0;
	if (type){
    	// Normalise orientation [encoder pulses] to [0..1]
        nom = value /(MaxOrEnc/2.0) - 1.0;
        // Check if value out of bounds
        nom = nom > 1.0 ? 1.0 : nom;
	}
	else{
        // Normalise velocity [encoder pulses per second] to [0..1]
        nom = value /MaxVelEnc;
	}
	// Scale to integer value [0..MaxValue]
	int scl = int(nom *MaxValue);
	return scl;
}

// Convert from byte to integer value
int byte_to_int(unsigned char byte_array[], int start, int len){
    int val = byte_array[start];
    for (int i = 1 ; i<len; i++){
        val = val | (int)byte_array[start+i]<<(i*8);
        }
    return val;
}

// Convert integer values to byte message
void int_to_byte(unsigned char byte_array[], int val[], int num_values){
    for (int j = 0 ; j<num_values; j++){
        for (int i = 0 ; i<4; i++){
            byte_array[i+j*4] = val[j] >> (i*8) & 0xFF;
        }
    }
    /*
    memcpy(txMsg.data, &velocity, 4);
    memcpy(txMsg.data+4, &orientation, 4)
    */
}

#if defined(DEBUG)
    // Print CAN message
    void printMsg(CANMessage& msg)
    {
        pc.printf("  ID      = 0x%.3x\r\n", msg.id);
        pc.printf("  Type    = %d\r\n", msg.type);
        pc.printf("  Format  = %d\r\n", msg.format);
        pc.printf("  Length  = %d\r\n", msg.len);
        pc.printf("  Data    =");
        for(int i = 0; i < msg.len; i++)
            pc.printf(" 0x%.2X", msg.data[i]);
        pc.printf("\r\n");
    }
#endif

void can_publisher_processing (void){
	txMsg.id = TX_ODM;            			            // Set ID (11 bit)
	txMsg.len    = 8;									// Length of transfered data [byte]
	// TODO read in encoder values
	// Convert data to message scale
	int velocity = enc_to_msg(driveVelocity, 0);			// velocity  [0..MaxValue]
	int orientation = enc_to_msg(steerOrientation, 1);	// orientation  [0..MaxValue]
	// Convert data to bytes
	int value[2] = {velocity, orientation};
	int_to_byte(txMsg.data, value, 2);
	// Write message to CAN bus
	msg_sent = can.write(txMsg);

	#if defined(DEBUG)
	    if(msg_sent) {
	        pc.printf("-------------------------------------\r\n");
	        pc.printf("CAN message sent\r\n");
	        printMsg(txMsg);
	        pc.printf("  velocity = %d \r\n", velocity);
	        pc.printf("  orientation = %d \r\n", orientation);
	    }
	    else
	        pc.printf("Transmission error\r\n");
    #endif
}

void can_publisher(void){
    msg_sent = 0;
}

// Processing thread outside of interrupt
void can_command_processing(void)
{
	#if defined(DEBUG)
	    pc.printf("-------------------------------------\r\n");
	    pc.printf("CAN message received\r\n");
	    printMsg(rxMsg);
	#endif

	// Check for correct message ID
    if (rxMsg.id == S_CMD) {
        // Extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        int drive = byte_to_int(rxMsg.data, 0, 4);          // Read first four bytes (driveMode  [0, 1])
        int steer = byte_to_int(rxMsg.data, 4, 4);          // Read second four bytes (steerMode  [0, 1])
        // Update PID mode and motor status for PID control
        if (drive != driveMode) {
            setDriveMode();
        }
        if (steer != steerMode) {
            setSteerMode();
        }
    }
    if (rxMsg.id == I_CMD) {
        // Extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        int pub = byte_to_int(rxMsg.data, 0, 4);           // Read first four bytes (publisher  [0, 1])
        int zero = byte_to_int(rxMsg.data, 4, 4);          // Read second four bytes (zeroEncoders  [0, 1])
        // Zero encoders
        if (zero && !zeroing_encoder){
            // Set zeroing encoder flag
            zeroing_encoder = 1;
        }
        // Set publisher
        if (pub != publisherMode){
            // Enable encoder feedback publisher
            setPublisherMode();
        }
    }
    if (rxMsg.id == RX_CMD) {
        // Extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        int velocity = byte_to_int(rxMsg.data, 0, 4);			// Read first four bytes (velocity scaled [0..MaxValue])
        int orientation = byte_to_int(rxMsg.data, 4, 4);		// Read second four bytes (orientation scaled [0..MaxValue])

        // Update set point for PID control
        set_velocity = msg_to_enc(velocity, 0);                 // velocity [0..EncVelMax]
        set_orientation = msg_to_enc(orientation, 1);           // orientation [0..EncOrMax]
        updateSetPoints();                                      // Update the PID set points
        #if defined(DEBUG)
	        pc.printf("Set velocity = %d\r\n", velocity);
	        pc.printf("Set orientation = %d\r\n", orientation);
	    #endif
    }
}

// CAN interrupt
// IMPORTANT: can.read must not contain mutex locking
void can_received(void)
{
    #if defined(DEBUG)
        led = !led;
    #endif
	// Read message from CAN bus
    can.read(rxMsg);
    // Set message received flag
    //msg_received = 1;
    // Process message outside ISR
    com_queue.call(&can_command_processing);
}

//-----------------------------------------/
// PID Position/Velocity Control
//-----------------------------------------/
void pid_control(void){
    pid = 1;
}

void pid_control_processing(void){
    // Compute PID
    // Get Pulses, Compute PID and set drive motors
    // Velocity control
    drivePulses = DriveQei.getPulses();
    driveVelocity = float(drivePulses - drivePrevPulses) / RATE;
    if (driveMode){
        VelocityController.setProcessValue(driveVelocity >= 0 ? driveVelocity : -driveVelocity);
        drivePwmDuty = VelocityController.compute();
        if (set_velocity > 0){
            DriveDirection = 1.0;
        }
        else if (set_velocity < 0){
            DriveDirection = 0.0;
        }
        DriveMotor = drivePwmDuty;
        #if defined(DEBUG)
            // UART serial output
            if (count >= 200) {
                pc.printf("V-Goal: %d, \t V-Current: %d, \t V-PWM: %d \r\n", int(set_velocity), int(driveVelocity), int(drivePwmDuty*1000.0));
            }
        #endif
    }
    // Position control
    steerPulses = SteerQei.getPulses();
    steerOrientation = float(steerPulses);
    if (steerMode){
        PositionController.setProcessValue(steerOrientation);
        steerPwmDuty = PositionController.compute();
        SteerDirection = steerPwmDuty >= 0 ? 1.0 : 0.0;
        steerPwmDuty = steerPwmDuty >= 0 ? steerPwmDuty : -steerPwmDuty;
        SteerMotor = steerPwmDuty;
        #if defined(DEBUG)
            // UART serial output
            if (count >= 200) {
                pc.printf("O-Goal: %d, \t O-Current: %d, \t O-PWM: %d \r\n", int(set_orientation), int(steerOrientation), int(steerPwmDuty*1000.0));
            }
        #endif
    }
    #if defined(DEBUG)
        count ++;
        if (count > 200) {
            count = 0;
        }
    #endif

    // Set previous pulses
    drivePrevPulses = drivePulses;

}

void zeroEncoders(void){
    #if defined(FRONT_LEFT) || defined(REAR_LEFT)
        // Left motor controller
        int side = 1;
    #endif

    #if defined(REAR_RIGHT) || defined(FRONT_RIGHT)
        // Right motor controller
        int side = 0;
    #endif
    // Set encoder value to zero
    SteerQei.reset();
    bool stop = 0;
    int enc_val = SteerQei.getPulses();
    int prev_enc = enc_val + pulsesPerRev;
    #if defined(DEBUG)
        pc.printf("Start searching for hard stop\r\n");
    #endif
    // Set motor direction opposite on both sides
    if (side == 0){
        SteerDirection = 1.0;
        SteerMotor = 0.15;
    }
    else{
        SteerDirection = 0.0;
        SteerMotor = 0.15;
    }
    // Continously check for hard stop
    while(!stop) {
        wait(RATE);
        // If hard stop is reached
        if (enc_val == prev_enc) {
            SteerMotor = 0.0;                   // Turn off motor
            stop = 1;
        } else {
            // Get encoder values
            prev_enc = enc_val;
            enc_val = SteerQei.getPulses();
        }
    }
    // Set Encoder value to zero
    SteerQei.reset();
    #if defined(DEBUG)
        pc.printf("Zeroing complete - hard stop reached\r\n");
        pc.printf("Enabling motor control - setting PID to auto\r\n");
    #endif
    // Scrap zeroing encoder flag
    zeroing_encoder = 0;
}

void setDriveMode(void){
    driveMode = !driveMode;                         // Set driving mode flag
    // Velocity control
    //VelocityController.setMode(driveMode);          // Set velocity PID control to manual or automatic
    if (!driveMode) {
        DriveMotor = 0.0;                           // Turn off driving motor
    }
    #if defined(DEBUG)
        pc.printf("Drive mode set to %d\r\n", driveMode);
    #endif
}

void setSteerMode(void){
    steerMode = !steerMode;                         // Set steering mode flag
    // Position control
    //PositionController.setMode(steerMode);          // Set position PID control to manual or automatic
    if (!steerMode) {
        SteerMotor = 0.0;                           // Turn off steering motor
    }
    #if defined(DEBUG)
        pc.printf("Steer mode set to %d\r\n", steerMode);
    #endif
}

void updateSetPoints(void){
    // Apply set points
    // Velocity control
    VelocityController.setSetPoint(set_velocity >= 0 ? set_velocity : -set_velocity);
    // Position control
    PositionController.setSetPoint(set_orientation);
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
    VelocityController.setInputLimits(0.0, MaxVelEnc);
    VelocityController.setOutputLimits(0.0, 0.9);
    VelocityController.setBias(0.0);
    VelocityController.setMode(AUTO_MODE);
    // Position PID controller
    PositionController.setInputLimits(-50.0, MaxOrEnc + 50.0);
    PositionController.setOutputLimits(-0.2, 0.2);
    PositionController.setBias(0.0);
    PositionController.setMode(AUTO_MODE);
}
