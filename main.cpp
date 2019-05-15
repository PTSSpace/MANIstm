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

    // PID update timer
    timer.start();
    #if defined(DEBUG)
        int count = 0;
        pc.printf("Start PID \r\n");
    #endif

    while (true) {
        //-----------------------------------------/
        // CAN Bus
        //-----------------------------------------/
        // Publish odometry message
		if (!msg_sent){
			can_publisher_processing();
		}

        //-----------------------------------------/
        // PID Position/Velocity Control
        //-----------------------------------------/
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
            steerOrientation = float(steerPulses);
            PositionController.setProcessValue(steerOrientation);
            steerPwmDuty = PositionController.compute();
            SteerDirection = steerPwmDuty >= 0 ? 1.0 : 0.0;
            steerPwmDuty = steerPwmDuty >= 0 ? steerPwmDuty : -steerPwmDuty;
            SteerMotor = steerPwmDuty;

            // Reset Timer
            timer.reset();
            #if defined(DEBUG)
                // UART serial output
                count ++;
                if (count > 10) {
                    pc.printf("D-Goal: %f, \t D-Current: %f, \t D-PWM: %f , \t P-Goal: %f, \t P-Current: %f, \t P-PWM: %f \r\n", set_velocity, driveVelocity, drivePwmDuty, set_orientation, steerOrientation, steerPwmDuty);
                    count = 0;
                }
            #endif

            // Set previous pulses
            drivePrevPulses = drivePulses;

        }

    }

    timer.stop();
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
    can.filter(RX_CMD, 0xFFF, CANStandard, 0);                                  // Set filter #0 to accept only standard messages with ID == RX_ID

    Thread commandThread(osPriorityLow);
    commandThread.start(callback(&com_queue, &EventQueue::dispatch_forever));
    can.attach(&can_received, CAN::RxIrq);                                      // Attach ISR to handle received messages

    // TODO interrupt priority
    publisher.attach(&can_publisher, PUB_RATE);
}
// Convert from message unit [0..MaxValue] to encoder unit [pulses]
float msg_to_enc(int value, bool type){
	// Scale back to decimal number [0..1]
	float dec = float(value) /MaxValue;
	float enc = 0;
    if (type){
    	// Convert orientation to [encoder pulses]
        enc = dec *MaxOrEnc;
	}
	else{
        // Convert velocity to [encoder pulses per second]
        enc = dec *MaxVelEnc;
	}
	return enc;
}

// Convert from encoder encoder unit [pulses] to message unit [0..MaxValue]
float enc_to_msg(float value, bool type){
    float nom = 0;
	if (type){
    	// Normalise orientation [encoder pulses] to [0..1]
        nom = value /MaxOrEnc;
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
	// Convert data to degrees
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
	        pc.printf("  velocity = %d rps\r\n", velocity);
	        pc.printf("  orientation = %d deg\r\n", orientation);

	    }
	    else
	        pc.printf("Transmission error\r\n");
	    }
    #endif

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
    if (rxMsg.id == RX_CMD) {
        // Extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        int velocity = byte_to_int(rxMsg.data, 0, 4);			// Read first four bytes (velocity  [0.001 rps])
        int orientation = byte_to_int(rxMsg.data, 4, 4);		// Read second four bytes (orientation  [0.001 rps])

        // Update set point for PID control
        set_velocity = msg_to_enc(velocity, 0);                 // velocity  [0..EncVelMax]
        set_orientation = msg_to_enc(orientation, 1);           // velocity  [0..EncOrMax]
        updateSetPoints();                                      // Update the PID set points
        #if defined(DEBUG)
	        pc.printf("  Set velocity = %d rps\r\n", velocity);
	        pc.printf("  Set orientation = %d deg\r\n", orientation);
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
    // Process message outside ISR
    com_queue.call(&can_command_processing);
}

//-----------------------------------------/
// PID Position/Velocity Control
//-----------------------------------------/
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
    PositionController.setInputLimits(-50.0, MaxOrientation + 50.0);
    PositionController.setOutputLimits(-0.2, 0.2);
    PositionController.setBias(0.0);
    PositionController.setMode(AUTO_MODE);
}