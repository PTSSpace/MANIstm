#include "stm32f103c8t6.h"
#include "mbed.h"
#include "CurrentSensor.h"
#include "CANInterface.h"

//****************************************************************************/
// Declaration - Static variables
//****************************************************************************/

//-----------------------------------------/
// UART Serial
//-----------------------------------------/

// UART Serial output
#if defined(DEBUG)
    // Wheel index
    #if defined(EPS)
        const char nodeIndex[4] = "eps";
    #endif
    // Declare LED pin
    #define LED_PIN     LED1
    Serial              pc(PB_6, PB_7);                             // UART Tx pin name, UART Rx pin name
    DigitalOut          led(LED_PIN);
#endif


//-----------------------------------------/
// CAN Bus
//-----------------------------------------/

// Initialise variables
bool        msg_received_ = 0;                                      // Message receiver flag
bool        msg_sent = 1;                                           // Message sender flag
bool        publisherMode = 0;                                      // Publisher status on/off
bool        pwrFeed = 0;                                            // Motor power set feedback to be sent


Ticker              pubTick;                                        // Current sensor publisher ticker

CANMessage          rxMsg;
CANMessage          txMsg;

// CAN interface
UnlockedCAN                 can(PB_8, PB_9);                        // CAN rdx pin name, CAN tdx pin name
CANInterface                ci;
DigitalOut          canSTB(PA_8);


// Interrupt event queue
EventQueue          com_queue;                                      // Command queue
Thread commandThread(osPriorityHigh);


//-----------------------------------------/
// Electrical Power Supply Monitoring
//-----------------------------------------/

int error[5] = {0, 0, 0, 0, 0};
int crit[5] = {0, 0, 0, 0, 0};
float current[5] = {0, 0, 0, 0, 0};

// Current sensors
CurrentSensor batterySen (PC_15, PA_0, float(ACS_25A), float(ACS_PWR));         // Full current drawn from battery
CurrentSensor flMotorSen (PA_1, PA_2, float(ACS_25A), float(ACS_PWR));          // Current drawn by front left motor
CurrentSensor rlMotorSen (PA_3, PA_4, float(ACS_25A), float(ACS_PWR));          // Current drawn by rear left motor
CurrentSensor rrMotorSen (PA_5, PA_6, float(ACS_25A), float(ACS_PWR));          // Current drawn by rear right motor
CurrentSensor frMotorSen (PA_7, PB_0, float(ACS_25A), float(ACS_PWR));          // Current drawn by front right motor

// Relays
DigitalOut MotorPower(PC_14) = 0;                                               // Motor power switch (initialised as off)

//****************************************************************************/
// Declaration - Working variables
//****************************************************************************/

//-----------------------------------------/
// PID Position/Velocity Control
//-----------------------------------------/

bool driveMode = 0;                                                 // Drive motor and velocity PID status
bool steerMode = 0;                                                 // Steer motor and position PID status
bool zeroingEncoder = 0;                                            // Zeroing encoder status

volatile float set_velocity = 0;                                    // [pulses per second]
volatile float set_orientation = 0;                                 // [pulses]



volatile bool pid = 0;                                              // PID computation flag
#if defined(DEBUG)
    int count = 0;
#endif



//****************************************************************************/
// Function - Initialization
//****************************************************************************/

//-----------------------------------------/
// UART Serial
//-----------------------------------------/
#if defined(DEBUG)
    void initializeSerial(void);
#endif

void initialize_can_bus(void);                                      // Initialization of can objects and interrupts
void can_received(void);                                            // CAN message interrupt handler
void can_publisher(void);                                           // Ticker interrupt handler to set send flag
void can_publisher_processing(uint8_t, int*, int, int);             // Processor for publisher flag
void can_command_processing(void);                                  // Processor thread for message interrupts
