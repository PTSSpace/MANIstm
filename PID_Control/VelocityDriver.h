#include "PID.h"
#include "QEI.h"
#include "mbed.h"

#define address 0x80
#define encoder_range 1600

#define pulsesPerRev 3200

//****************************************************************************/
// Defines - PID
//****************************************************************************/
#define RATE  0.01
// PID values from prelim tuning
#define Kc    0.7
#define Ti    0.05 
#define Td    0.0


/****************************************************************************/
// Prototypes
//****************************************************************************/
//Set motors to go "forward", brake off, not moving.
void initializeMotors(void);
//Set up PID controllers with appropriate limits and biases.
void initializePidControllers(void); 

void computePIDLeftVelocity(float goal_velocity); 
