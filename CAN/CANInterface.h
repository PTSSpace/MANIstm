#ifndef CANInterface_H
#define CANInterface_H
/**
 * Includes
 */
#include "mbed.h"
#include "defines.h"

/**
 * CAN Bus Interface.
 */

class UnlockedCAN: public CAN{
public:
    UnlockedCAN(PinName rd, PinName td) : CAN(rd, td) { }   // CAN rdx pin name, CAN tdx pin name
protected:
    // Unlocked CAN class with disabled mutex locks
    // to be used in the Interrupt Irq
    virtual void lock(void) { };
    virtual void unlock(void) { };
};

class CANInterface{

public:
    CANInterface();
    float msg_to_enc(int, bool);                            // Convert from message unit [0..MaxValue] to encoder unit [pulses]
    float enc_to_msg(float, bool);                          // Convert from encoder encoder unit [pulses] to message unit [0..MaxValue]
    int byte_to_int(unsigned char*, int, int);              // Convert byte message to integer
    void int_to_byte(unsigned char*, int*, int);            // Convert integer to byte message
};

#endif /* CANInterface_H */
