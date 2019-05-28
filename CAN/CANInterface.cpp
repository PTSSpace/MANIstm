/**
 * Includes
 */
#include "CANInterface.h"

/**
 * Functions
 */
CANInterface::CANInterface(){ }

// Convert from message unit [0..MaxValue] to encoder unit [pulses]
float CANInterface::msg_to_enc(int value, bool type){
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
float CANInterface::enc_to_msg(float value, bool type){
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
int CANInterface::byte_to_int(unsigned char byte_array[], int start, int len){
    int val = byte_array[start];
    for (int i = 1 ; i<len; i++){
        val = val | (int)byte_array[start+i]<<(i*8);
        }
    return val;
}

// Convert integer values to byte message
void CANInterface::int_to_byte(unsigned char byte_array[], int val[], int num_values){
    for (int j = 0 ; j<num_values; j++){
        for (int i = 0 ; i<4; i++){
            byte_array[i+j*4] = val[j] >> (i*8) & 0xFF;
        }
    }
}

