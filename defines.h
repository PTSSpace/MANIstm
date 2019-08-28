#ifndef defines_H
#define defines_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */

#define DEBUG					1				                                // Debug output
#define FRONT_LEFT              1                                               // Wheel index ['front_left', 'rear_left', 'rear_right', 'front_right']

// Protocol parameters
#define MaxValue                32768                                           // Maximal signed value for 2 bytes

//****************************************************************************/
// Defines
//****************************************************************************/

//-----------------------------------------/
// Motor/Encoder Parameters
//-----------------------------------------/

// Motor control encoders
#define DRIVE_ENC_PPR           8384
#define STEER_ENC_PPR           8384

#define PI                      3.14159265358979323846
#define PI_2                    1.57079632679489661923

// Set motor parameters
const float MAX_VEL     = PI;                                                   // Maximal wheel velocity [rad/s]
const float MAX_ORT     = PI_2;                                                 // Maximal wheel orientation [rad]
const float MaxVelEnc   = DRIVE_ENC_PPR * MAX_VEL/(2.0*float(PI));              // Maximal wheel velocity [encoder pulses per second]
const float MaxOrEnc    = float(STEER_ENC_PPR)/2.0;                             // Maximal wheel orientation [encoder pulses] (Span 0...MaxOrEnc)

// CAN communication
#define PUB_RATE                5.0                                             // Rate to publish odometry data [s]





#define FREQUENCY				500000                                         // CAN bus bit rate




#if defined(FRONT_LEFT) || defined(REAR_LEFT)
    // Left motor controller
    #define SIDE 				1
#endif

#if defined(REAR_RIGHT) || defined(FRONT_RIGHT)
    // Right motor controller
    #define SIDE 				0
#endif


// Loop rate
#define  RATE 0.01

// PID values from tuning
// Velocity control
#define Vel_Kc          0.1
#define Vel_Ti          0.35                        //expandable
#define Vel_Td          0.0
#define Vel_RP          50                          // Precession for setting point reached flag
// Position control
#define Pos_Kc          180.0//25.0 //3.25 //0.9 // 3.25
#define Pos_Ti          0.0035//0.008 //0.45 //0.0015//expandable //0.65
#define Pos_Td          0.03//0.02 //0.0 //0.02
#define Pos_RP          50                          // Precession for setting point reached flag

#endif /* defines_H */


// Message header ID's:
#if defined(FRONT_LEFT)
    // Front left motor controller
    #define S_CMD       0x0A1                       // Start/stop and initialization command
    #define O_CMD       0x0B1 		                // Steer orientation locomotion command
    #define V_CMD       0x0C1 		                // Drive velocity locomotion command
    #define LC_FB       0x0D1 		                // Locomotion task accomplished feedback (Orientation/Velocity reached)
    #define OM_FB       0x0E1 		                // Wheel encoder odometry update
#endif
#if defined(REAR_LEFT)
    // Rear left motor controller
    #define S_CMD       0x0A2                       // Start/stop and initialization command
    #define O_CMD       0x0B2 		                // Steer orientation locomotion command
    #define V_CMD       0x0C2 		                // Drive velocity locomotion command
    #define LC_FB       0x0D2 		                // Locomotion task accomplished feedback (Orientation/Velocity reached)
    #define OM_FB       0x0E2 		                // Wheel encoder odometry update
#endif
#if defined(REAR_RIGHT)
    // Rear right motor controller
    #define S_CMD       0x0A3                       // Start/stop and initialization command
    #define O_CMD       0x0B3 		                // Steer orientation locomotion command
    #define V_CMD       0x0C3 		                // Drive velocity locomotion command
    #define LC_FB       0x0D3 		                // Locomotion task accomplished feedback (Orientation/Velocity reached)
    #define OM_FB       0x0E3 		                // Wheel encoder odometry update
#endif
#if defined(FRONT_RIGHT)
    // Front right motor controller
    #define S_CMD       0x0A4                       // Start/stop and initialization command
    #define O_CMD       0x0B4 		                // Steer orientation locomotion command
    #define V_CMD       0x0C4 		                // Drive velocity locomotion command
    #define LC_FB       0x0D4 		                // Locomotion task accomplished feedback (Orientation/Velocity reached)
    #define OM_FB       0x0E4 		                // Wheel encoder odometry update
#endif

#define MASK            0x08F                       // CAN filter mask
