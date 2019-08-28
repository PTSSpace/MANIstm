#ifndef defines_H
#define defines_H

/**
 * Includes
 */
#include "mbed.h"

/**
 * Defines
 */

#define DEBUG                   1                   // Debug output
#define EPS                     1                   // Wheel index ['front_left', 'rear_left', 'rear_right', 'front_right']

// Protocol parameters
#define MaxValue                2147483647          // Maximal signed value for 4 bytes

//****************************************************************************/
// Defines
//****************************************************************************/

//-----------------------------------------/
// Electrical Power Supply Monitoring
//-----------------------------------------/
# define ACS_25A        55                          // Sensitivity ACS711 25A [mV/A]
# define ACS_12A        110                         // Sensitivity ACS711 12A [mV/A]
# define ACS_VLT        3300                        // Voltage source for analog current sensors [mV]


//-----------------------------------------/
// CAN Bus
//-----------------------------------------/
// CAN communication
#define PUB_RATE                5.0                 // Rate to publish odometry data [s]

#define FREQUENCY               500000              // CAN bus bit rate

// Message header ID's:
#if defined(EPS)
    // Front right motor controller
    #define P_CMD       0x000                       // Motor power on/off command
    #define E_WRN       0x010                       // Sensor overcurrent warning (fault line latched low)
    #define C_WRN       0x020                       // Critical current warning (80 percent of max current)
    #define P_FD        0x030                       // Motor power status feedback/update
    #define C_FD        0x0E0                       // Feedback from current sensors
#endif

#define MASK            0x00F                       // CAN filter mask

#endif /* defines_H */
