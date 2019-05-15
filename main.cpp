#include "stm32f103c8t6.h"
#include "mbed.h"


#define DEBUG					1				// Debug output

// Motor control
#define pulsesPerRev            8384
#define pulsesPerHalf           4192

// Set motor parameters
#define MaxVelocity             3                       // Maximal wheel velocity [rps]
const float MaxVelEnc = MaxVelocity * pulsesPerRev;     // Maximal wheel velocity [encoder pulses per second]
#define MaxOrientation          180                     // Maximal wheel orientation [deg]
const float MaxOrEnc = MaxOrientation * pulsesPerRev;   // Maximal wheel orientation [encoder pulses]


// Protocol parameters
#define MaxValue                2147483647      // Maximal signed value for 4 bytes


// CAN communication
#define PUB_RATE                5.0     		// Rate to publish odometry data [s]
#define FRONT_LEFT           	1       	    // Wheel index ['front_left', 'rear_left', 'rear_right', 'front_right']

#if defined(DEBUG)
    // Declare LED pin
    #define LED_PIN     LED1
#endif


/*
Messages with lower numeric values for their ID's
have higher priority on the CAN network
*/

// Message header ID's:
#if defined(FRONT_LEFT)
    // Front left motor controller
    const uint8_t RX_CMD = 0x0C1;		    // Locomotion command
    const uint8_t TX_ODM = 0x0D1;		    // Odometry update
    #if defined(DEBUG)
        const char wheelIndex[12] = "front_left";  // Wheel index
    #endif
#endif
#if defined(REAR_LEFT)
    // Rear left motor controller
    const uint8_t RX_CMD = 0x0C2;		    // Locomotion command
    const uint8_t TX_ODM = 0x0D2;      	    // Odometry update
    #if defined(DEBUG)
        const char wheelIndex[12] = "rear_left";  // Wheel index
    #endif
#endif
#if defined(REAR_RIGHT)
    // Rear right motor controller
    const uint8_t RX_CMD = 0x0C3;      	    // Locomotion command
    const uint8_t TX_ODM = 0x0D3;      	    // Odometry update
    #if defined(DEBUG)
        const char wheelIndex[12] = "rear_right";  // Wheel index
    #endif
#endif
#if defined(FRONT_RIGHT)
    // Front right motor controller
    const uint8_t RX_CMD = 0x0C4;      	    // Locomotion command
    const uint8_t TX_ODM = 0x0D4;      	    // Odometry update
    #if defined(DEBUG)
        const char wheelIndex[12] = "front_right";  // Wheel index
    #endif
#endif


// UART Serial output
#if defined(DEBUG)
	Serial              pc(PB_6, PB_7);		// UART Tx pin name, UART Rx pin name
	DigitalOut          led(LED_PIN);
#endif

// CAN interface
CAN                 can(PB_8, PB_9);  		// CAN Rdx pin name, CAN Tdx pin name
CANMessage       	rxMsg;
CANMessage          txMsg;
volatile bool 		msg_received = 0;
volatile bool 		msg_sent = 1;


// Interrupt event queue
EventQueue 			com_queue;           	// Command queue
EventQueue 			pub_queue;           	// Publisher queue

// Odometry publisher ticker
Ticker              publisher;



// Working variables
float 				enc_velocity;           // Velocity [pulses per second]
float 				enc_orientation;        // Orientation [pulses per second]
float 				set_velocity;           // [pulses per second]
float 				set_orientation;        // [pulses per second]


// Functions
int byte_to_int(unsigned char, int, int);
void int_to_byte(unsigned char, int, int);

void printMsg(CANMessage&);
void can_publisher_processing (void);
void can_publisher(void);
void can_command_processing(void);
void can_received(void);




// Main function
int main(void)
{
    //confSysClock();     														//Configure the system clock (72MHz HSE clock, 48MHz USB clock)

    // CAN Interface
    can.mode(CAN::Normal);
    can.frequency(500000); 														// Set CAN bit rate to 1Mbps
    can.filter(RX_CMD, 0xFFF, CANStandard, 0); 					// Set filter #0 to accept only standard messages with ID == RX_ID

    Thread commandThread(osPriorityLow);
    commandThread.start(callback(&com_queue, &EventQueue::dispatch_forever));
    can.attach(&can_received, CAN::RxIrq);                						// Attach ISR to handle received messages


    // TODO interrupt priority
    publisher.attach(&can_publisher, PUB_RATE);


    #if defined(DEBUG)
    	pc.baud(115200);        // Set serial speed
    	led = 0;                // Turn on LED for message feedback
    	pc.printf("CAN Control %s board \r\n", wheelIndex);
    #endif

    while(1) {
        // Publish odometry message
		if (!msg_sent){
			can_publisher_processing();
		}
    }
    //wait(osWaitForever);

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

void can_publisher_processing (void){
	txMsg.id = TX_ODM;            			            // Set ID (11 bit)
	txMsg.len    = 8;									// Length of transfered data [byte]
	// TODO read in encoder values
	// Convert data to degrees
	int velocity = enc_to_msg(enc_velocity, 0);			// velocity  [0..MaxValue]
	int orientation = enc_to_msg(enc_orientation, 1);	// orientation  [0..MaxValue]
	// Convert data to bytes
	int value[2] = {velocity, orientation};
	int_to_byte(txMsg.data, value, 2);
	// Write message to CAN bus
	msg_sent = can.write(txMsg);

	#if defined(DEBUG)
	    if(msg_sent) {
	        pc.printf("-------------------------------------\r\n");
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

        #if defined(DEBUG)
	        pc.printf("  Set velocity = %d rps\r\n", velocity);
	        pc.printf("  Set orientation = %d deg\r\n", orientation);
	    #endif
    }
}

// CAN interrupt
// can.read must not contain mutex locking
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
