#include "stm32f103c8t6.h"
#include "mbed.h"
#include "PID.h"
#include "QEI.h"

//****************************************************************************/
// Defines - Flags
//****************************************************************************/

//#define DEBUG					1				    // Debug output
#define FRONT_LEFT              1                   // Wheel index ['front_left', 'rear_left', 'rear_right', 'front_right']

// Protocol parameters
#define MaxValue                2147483647          // Maximal signed value for 4 bytes

//****************************************************************************/
// Defines
//****************************************************************************/

//-----------------------------------------/
// Motor/Encoder Parameters
//-----------------------------------------/

// Motor control
#define pulsesPerRev            8384
#define pulsesPerHalf           4192

// Set motor parameters
#define MaxVelocity             3                                       // Maximal wheel velocity [rps]
const float MaxVelEnc = MaxVelocity * pulsesPerRev;                     // Maximal wheel velocity [encoder pulses per second]
#define MaxOrientation          3.14159265358979323846                  // Maximal wheel orientation [rad]
const float MaxOrEnc = float(pulsesPerHalf);                            // Maximal wheel orientation [encoder pulses]


//-----------------------------------------/
// UART Serial
//-----------------------------------------/

// UART Serial output
#if defined(DEBUG)
    // Declare LED pin
    #define LED_PIN     LED1
    Serial              pc(PB_6, PB_7);             // UART Tx pin name, UART Rx pin name
    DigitalOut          led(LED_PIN);
#endif


//-----------------------------------------/
// CAN Bus
//-----------------------------------------/

// CAN communication
#define PUB_RATE                5.0     		    // Rate to publish odometry data [s]

/*
Messages with lower numeric values for their ID's
have higher priority on the CAN network
*/

// Message header ID's:
#if defined(FRONT_LEFT)
    // Front left motor controller
    const uint8_t S_CMD = 0x0A1;                    // Start/stop command
    const uint8_t I_CMD = 0x0B1;                    // Initialization command
    const uint8_t RX_CMD = 0x0C1;		            // Locomotion command
    const uint8_t TX_ODM = 0x0D1;		            // Odometry update
    #if defined(DEBUG)
        const char wheelIndex[12] = "front_left";   // Wheel index
    #endif
#endif
#if defined(REAR_LEFT)
    // Rear left motor controller
    const uint8_t S_CMD = 0x0A2;                    // Start/stop command
    const uint8_t I_CMD = 0x0B2;                    // Initialization command
    const uint8_t RX_CMD = 0x0C2;		            // Locomotion command
    const uint8_t TX_ODM = 0x0D2;      	            // Odometry update
    #if defined(DEBUG)
        const char wheelIndex[12] = "rear_left";    // Wheel index
    #endif
#endif
#if defined(REAR_RIGHT)
    // Rear right motor controller
    const uint8_t S_CMD = 0x0A3;                    // Start/stop command
    const uint8_t I_CMD = 0x0B3;                    // Initialization command
    const uint8_t RX_CMD = 0x0C3;      	            // Locomotion command
    const uint8_t TX_ODM = 0x0D3;      	            // Odometry update
    #if defined(DEBUG)
        const char wheelIndex[12] = "rear_right";   // Wheel index
    #endif
#endif
#if defined(FRONT_RIGHT)
    // Front right motor controller
    const uint8_t S_CMD = 0x0A4;                    // Start/stop command
    const uint8_t I_CMD = 0x0B4;                    // Initialization command
    const uint8_t RX_CMD = 0x0C4;      	            // Locomotion command
    const uint8_t TX_ODM = 0x0D4;      	            // Odometry update
    #if defined(DEBUG)
        const char wheelIndex[12] = "front_right";  // Wheel index
    #endif
#endif

// CAN interface
const uint8_t MASK = 0x08F;		        // CAN filter mask

// Declaration of unlocked CAN class with disabled mutex locks
// to be used in the Interrupt Irq
class UnlockedCAN : public CAN {
public:
    UnlockedCAN(PinName rd, PinName td) : CAN(rd, td) { }
    virtual void lock() { }
    virtual void unlock() { }
};


UnlockedCAN                 can(PB_8, PB_9);        // CAN Rdx pin name, CAN Tdx pin name
CANMessage          rxMsg;
CANMessage          txMsg;
volatile bool       msg_received = 0;
volatile bool       msg_sent = 1;


// Interrupt event queue
EventQueue          com_queue;                      // Command queue
EventQueue          pub_queue;                      // Publisher queue
EventQueue          pid_queue;                      // PID control queue


// Odometry publisher ticker
Ticker          pubTick;
int publisherMode = 0;                              // Publisher status on/off
int zeroing_encoder = 0;                            // Zeroing encoder status

//-----------------------------------------/
// PID Position/Velocity Control
//-----------------------------------------/

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


// TODO check
//#define  MAX_VELOCITY 3000.0                      // pulses per second

Ticker           pidTick;                          // PID call instance
volatile bool pid = 0;
#if defined(DEBUG)
    int count = 0;
#endif
//****************************************************************************/
// Declaration - Working variables
//****************************************************************************/

int driveMode = 0;                                  // Drive motor and velocity PID status
int steerMode = 0;                                  // Steer motor and position PID status

float set_velocity = 0;                                 // [pulses per second]
float set_orientation = 0;                              // [pulses per second]

// Velocity PID control
volatile int drivePulses     = 0;
volatile int drivePrevPulses = 0;
volatile float drivePwmDuty  = 0.0;
volatile float driveVelocity = 0.0;                 // Velocity [pulses per second]

PwmOut DriveMotor(PA_6);
DigitalOut DriveDirection(PA_7);
QEI DriveQei(PA_11, PA_12, NC, pulsesPerRev, QEI::X4_ENCODING);     // TODO Adjust pins to not overlapp with CAN (important 5V tollerant interrupt)
PID VelocityController(Vel_Kc, Vel_Ti, Vel_Td, RATE);

// Position control
volatile int steerPulses     = 0;
volatile float steerPwmDuty  = 0.0;
volatile float steerOrientation = 0.0;              // Orientation [pulses]

PwmOut SteerMotor(PB_0);
DigitalOut SteerDirection(PB_1);
QEI SteerQei(PA_9, PA_10, NC, pulsesPerRev, QEI::X4_ENCODING);
PID PositionController(Pos_Kc, Pos_Ti, Pos_Td, RATE);


//****************************************************************************/
// Function - Initialization
//****************************************************************************/

//-----------------------------------------/
// UART Serial
//-----------------------------------------/
#if defined(DEBUG)
    void initializeSerial(void);
#endif

//-----------------------------------------/
// CAN Bus
//-----------------------------------------/
#if defined(DEBUG)
    void printMsg(CANMessage&);                         // Serial print CAN message
#endif
void initializeCanBus(void);                            // Initialization of can objects and interrupts
void setPublisherMode(void);                            // Turn on/off wheel encoder odometry publisher
int byte_to_int(unsigned char, int, int);               // Convert byte message to integer
void int_to_byte(unsigned char, int, int);              // Convert integer to byte message
void can_publisher_processing (void);                   // Processer for publisher flag
void can_publisher(void);                               // Ticker interrupt handler to set send flag
void can_command_processing(void);                      // Processer thread for message interrupts
void can_received(void);                                // CAN message interrupt handler


//-----------------------------------------/
// PID Position/Velocity Control
//-----------------------------------------/
void zeroEncoders(void);                                // Zero incremental encoders
void setDriveMode(void);                                // Set drive mode to on or off
void setSteerMode(void);                                // Set steer mode to on or off
void initializeMotors(void);                            // Initiallising motor pins
void initializePidControllers(void);                    // Set up PID controllers with appropriate limits and biases
void updateSetPoints(void);                             // Update the PID set points
void pid_control(void);                                 // PID controller
void pid_control_processing(void);
