/**
 * Includes
 */
#include "CANInterface.h"

/**
 * Functions
 */
CANInterface::CANInterface(){ }

// Convert from byte to integer value
int CANInterface::byte_to_int(unsigned char byte_array[], int start, int len){
    int val = byte_array[start];
    for (int i = 1 ; i<len; i++){
        val = val | (int)byte_array[start+i]<<(i*8);
        }
    return val;
}

// Convert integer values to byte message
void CANInterface::int_to_byte(unsigned char byte_array[], int val[], int num_values){
    // Bytes per value
    int b = 2;
    short value;                // Necessary for 2 byte protocol
    for (int j = 0 ; j<num_values; j++){
        value = short(val[j]);
        for (int i = 0 ; i<b; i++){
            byte_array[i+j*b] = value >> (i*8) & 0xFF;
        }
    }
}

