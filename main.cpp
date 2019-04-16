/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "CANnucleo.h"
#include "stm32f103c8t6.h"

// board1 is driving board, board2 is controlling board
// board1 received analog values over CAN, board2 reads analog values from HW
#define BOARD1
//#define BOARD2

#define MAX_VELOCITY 500.0

Serial pc(PB_6, PB_7, NULL, 9600);

#if defined(BOARD1)
    #include "VelocityDriver.h" // include velocity driver on board 1
    const unsigned int RX_ID = 0x100;
    const unsigned int TX_ID = 0x101;
#else
    #define RATE  0.01
    const unsigned int RX_ID = 0x101;
    const unsigned int TX_ID = 0x100;
#endif

CAN*            can;
CANMessage      rxMsg;
CANMessage      txMsg;
Timer           timer;

#if defined(BOARD2)
    AnalogIn        velocityIn(PA_0);
    AnalogIn        directionIn(PA_1);
#endif

uint8_t         direction = 0;
float           velocity = 0.0;        // four bytes
volatile bool   msgAvailable = false;

/**
 * @brief   'CAN receive-complete' interrup service routine.
 * @note    Called on arrival of new CAN message.
 *          Keep it as short as possible.
 * @param
 * @retval
 */
void onMsgReceived() {
    msgAvailable = true;
}

/**
 * @brief   Prints CAN msg to PC's serial terminal.
 * @note}
 * @param   CANMessage to print
 * @retval  none
 */
void printMsg(CANMessage& msg) {
    pc.printf("  ID      = 0x%.3x\r\n", msg.id);
    pc.printf("  Type    = %d\r\n", msg.type);
    pc.printf("  Format  = %d\r\n", msg.format);
    pc.printf("  Length  = %d\r\n", msg.len);
    pc.printf("  Data    =");
    for(int i = 0; i < msg.len; i++)
        pc.printf(" 0x%.2X", msg.data[i]);
    pc.printf("\r\n");
 }

 void CANinit() {
    confSysClock();     //Configure the system clock (72MHz HSE clock, 48MHz USB clock)
    pc.baud(9600);
    can = new CAN(PA_11, PA_12);        // CAN Rx pin name, CAN Tx pin name
    can->frequency(1000000);            // set the bit rate to 1Mbps
    can->attach(&onMsgReceived);        // attach the 'CAN receive-complete' interrupt service routine (ISR)
    timer.start();  // start the timer
    pc.printf("CANnucleo_Hello board #1\r\n");
 }

#if defined(BOARD2)
 void canWriteProcedure() {
    if(timer.read_ms() >= 1000) {           // check for timeout
        timer.stop();                       // stop the timer
        timer.reset();                      // reset the timer
        direction = directionIn.read();   // read the direction from analog input
        velocity = velocityIn.read()*MAX_VELOCITY;          // read the velocity from analog input
        txMsg.clear();                      // clear the Tx message storage
        txMsg.id = TX_ID;                   // set the message ID
        // We are about to transmit two data items to the CAN bus.
        //     counter: uint_8 (unsigned eight bits int) value (one byte).
        //     voltage: floating point value (four bytes).
        // So the total length of payload data is five bytes.
        // We'll use the "<<" (append) operator to add data to the CAN message.
        // The usage is same as of the similar C++ io-stream operators.
        // NOTE: The data length of CAN message is automatically updated when using "<<" operators.
        txMsg << direction << velocity;        // append data (total data length must be <= 8 bytes!)
        if(can->write(txMsg)) {             // transmit the CAN message
            pc.printf("-------------------------------------\r\n");
            pc.printf("CAN message sent\r\n");
            printMsg(txMsg);
            pc.printf("  direction = %d\r\n", direction);
            pc.printf("  velocity = %e\r\n", velocity);
            }
        else
            pc.printf("Transmission error\r\n");
        timer.start();
    }
}
#endif

void canReadProcedure() {
    if(msgAvailable) {
        msgAvailable = false;               // reset the flag for next use in the interrupt service routine
        can->read(rxMsg);                   // read the message into the Rx message storage
        pc.printf("-------------------------------------\r\n");
        pc.printf("CAN message received\r\n");
        printMsg(rxMsg);
        // Filtering performed by software:
        if(rxMsg.id == RX_ID) {             // about filtering performed by hardware see the comments in CANnucleo.cpp
            rxMsg >> direction >> velocity;    // extract data from the received CAN message (in same sequence as they were added)
            velocity = direction == 0 ? -velocity : velocity; // if direction is 0, put negative velocity
            pc.printf("  direction = %d\r\n", direction);
            pc.printf("  velocity = %e\r\n", velocity);
        }
    }
}


// main() runs in its own thread in the OS
int main()
{
    #if defined(BOARD1)
    //Initialization.
    initializeMotors();
    initializePidControllers();
    #endif

    CANinit();

    int count = 0;

    while (true) {  //endTimer.read() < 20.0){

        #if defined(BOARD1)
        canReadProcedure();
        computePIDLeftVelocity(velocity);

        #elif defined(BOARD2)
        canWriteProcedure();
        #endif


        if (count % 100 == 0) {

            #if defined(BOARD1)
            pc.printf("BOARD1:\tRead Velocity: %f,\t Read direction: %f\r\n", velocity, direction);
            //pc.printf("Velocity: %f,\t Pulses: %d,\t Goal: %f,\tPWM: %f, \t Dir: %d\r\n", leftVelocity, leftPulses, velocity, leftPwmDuty, leftDirection.read());
            #elif defined(BOARD2)
            pc.printf("BOARD2:\tRead Velocity: %f,\t Read direction: %f\r\n", velocity, direction);
            #endif
        }

        wait(RATE);
        count++;
    }

    #if defined(BOARD1)
    //Stop motors.
    computePIDLeftVelocity(0.0);
    #endif

}


