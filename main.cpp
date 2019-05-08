#include "mbed.h"
#include "stm32f103c8t6.h"

/*
 * An example showing how to use the mbed CAN API:
 *
 * Two affordable (about $2 on ebay) STM32F103C8T6 boards (20kB SRAM, 64kB Flash),
 * (see [https://developer.mbed.org/users/hudakz/code/STM32F103C8T6_Hello/] for more details)
 * are connected to the same CAN bus via transceivers (MCP2551 or TJA1040, or etc.).
 * CAN transceivers are not part of NUCLEO boards, therefore must be added by you.
 * Remember also that CAN bus (even a short one) must be terminated with 120 Ohm resitors at both ends.
 *
 *
 * The same code is used for both mbed boards, but:
 *      For board #1 compile the example without any change.
 *      For board #2 comment out line 21 before compiling
 *
 * Once the binaries have been downloaded to the boards reset both boards at the same time.
 *
 */

#define TARGET_STM32F103C8T6    1       // uncomment this line to use STM32F103C8T6 boards

#define BOARD1                  1       // comment out this line when compiling for board #2

#define PUB_RATE                5.0     // Rate to publish odometry data [s]

#if defined(TARGET_STM32F103C8T6)
    #define LED_PIN     PC_13
    const int           OFF = 1;
    const int           ON = 0;
#else
    #define LED_PIN     LED1
    const int           OFF = 0;
    const int           ON = 1;
#endif

#if defined(BOARD1)
    const unsigned int  RX_ID = 0x101;
    const unsigned int  TX_ID = 0x100;
#else
    const unsigned int  RX_ID = 0x102;
    const unsigned int  TX_ID = 0x100;
#endif

#include "mbed.h"
#include "CANMsg.h"

Serial              pc(PB_6, PB_7);
CAN                 can(PB_8, PB_9);  // CAN Rx pin name, CAN Tx pin name
Ticker              publisher;

CANMessage              rxMsg;
CANMessage              txMsg;
DigitalOut          led(LED_PIN);
int               speed = 0.0;
int               orientation = 0.0;
volatile bool msg_rx_flag = 0;


// create an event queue
EventQueue com_queue;           // Command queue
EventQueue pub_queue;           // Publisher queue

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
    memcpy(txMsg.data, &speed, 4);
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
    if(can.write(txMsg)) {
        pc.printf("-------------------------------------\r\n");
        pc.printf("-------------------------------------\r\n");
        pc.printf("CAN message sent\r\n");
        printMsg(txMsg);
        pc.printf("  speed = %d rps\r\n", speed);
        pc.printf("  orientation = %d deg\r\n", orientation);

    }
    else
        pc.printf("Transmission error\r\n");
    }

void can_publisher(void){
    led = !led;
    txMsg.id = TX_ID;            // set ID
    // append data (total data length must not exceed 8 bytes!)
    txMsg.len    = 8;
    int val[2] = {speed, orientation};
    int_to_byte(txMsg.data, val, 2);
    can.write(txMsg);
    //pub_queue.call(&can_publisher_processing);
}

// Processing thread outside of interrupt
void can_command_processing(void)
{
    pc.printf("-------------------------------------\r\n");
    pc.printf("CAN message received\r\n");
    printMsg(rxMsg);

    if (rxMsg.id == RX_ID) { //
        // extract data from the received CAN message
        // in the same order as it was added on the transmitter side
        speed = byte_to_int(rxMsg.data, 0, 4);//rxMsg.data[0] | ( (int)rxMsg.data[1] << 8 ) | ( (int)rxMsg.data[2] << 16 ) | ( (int)rxMsg.data[3] << 24 );
        orientation = byte_to_int(rxMsg.data, 4, 4);;
        pc.printf("  Set speed = %d rps\r\n", speed);
        pc.printf("  Set orientation = %d deg\r\n", orientation);
    }
}

// CAN interrupt
// can.read must not contain mutex locking
void can_received(void)
{
    //led = !led;
    can.read(rxMsg);
    com_queue.call(&can_command_processing);

}

// Main function
int main(void)
{
    confSysClock();     //Configure the system clock (72MHz HSE clock, 48MHz USB clock)
    pc.baud(115200);          // set serial speed
    can.mode(CAN::Normal);
    can.frequency(500000); // set CAN bit rate to 1Mbps


    led = ON;

    can.filter(RX_ID, 0xFFF, CANStandard, 0); // set filter #0 to accept only standard messages with ID == RX_ID

    Thread commandThread(osPriorityNormal);
    commandThread.start(callback(&com_queue, &EventQueue::dispatch_forever));
    can.attach(&can_received, CAN::RxIrq);                // attach ISR to handle received messages

    //Thread publisherThread(osPriorityLow);
    //publisherThread.start(callback(&pub_queue, &EventQueue::dispatch_forever));
    publisher.attach(&can_publisher, PUB_RATE);

#if defined(BOARD1)
    pc.printf("CAN_Hello board #1\r\n");
#else
    pc.printf("CAN_Hello board #2\r\n");
#endif
    speed = int(1.9f) * 1000;
    orientation = int(45.0f) * 1000;

    /*
    txMsg.len    = 0;
    txMsg.type   = CANData;
    txMsg.format = CANStandard;
    txMsg.id     = 0;
    memset(txMsg.data, 0, 8);              // clear Tx message storage
    */

    while(1) {
        wait(0.2);
    }
    //wait(osWaitForever);

}
