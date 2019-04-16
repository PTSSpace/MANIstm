/* mbed Microcontroller Library
 * Copyright (c) 2018 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "RoboClaw.h"

#define address 0x80
#define baud 38400
#define encoder_range 1600

Serial pc(SERIAL_TX, SERIAL_RX);
RoboClaw rc(address, baud, PC_10, PC_11);

void setPos(int angle) {

    uint32_t accel = 2000;
    int32_t speed = 2500;
    uint32_t deccel = 2000;
    uint8_t flag = 1;

    int32_t pos = (-(angle/90)*(encoder_range/2)) + (encoder_range/2);
    pc.printf("New pos: %d\r\n", pos);
    rc.SpeedAccelDeccelPositionM1(accel, speed, deccel, pos, flag);
}

// main() runs in its own thread in the OS
int main()
{

    int init1 = 1;
    int enc_val1 = rc.ReadEncM1();
    int prev_enc1 = enc_val1 + 100;
    rc.ForwardM1(15);

    int init2 = 1;
    int enc_val2 = rc.ReadEncM2();
    int prev_enc2 = enc_val2 + 100;
    rc.ForwardM2(15);

    while(init1 || init2) {
        wait_ms(50);
        if (init1) {
            if (enc_val1 == prev_enc1) {
                init1 = 0;
                rc.ForwardM1(0);
            } else {
                prev_enc1 = enc_val1;
                enc_val1 = rc.ReadEncM1();
                pc.printf("Enc1 Value updated\r\n");
            }
        }

        if (init2) {
            if (enc_val2 == prev_enc2) {
                init2 = 0;
                rc.ForwardM2(0);
            } else {
                prev_enc2 = enc_val2;
                enc_val2 = rc.ReadEncM2();
                pc.printf("Enc2 Value updated\r\n");
            }
        }

    }    
    pc.printf("PEnc1: %d\r\n", prev_enc1);
    pc.printf("PEnc2: %d\r\n", prev_enc2);
    pc.printf("Enc1: %d\r\n", enc_val1);
    pc.printf("Enc2: %d\r\n", enc_val2);

    rc.ResetEnc();
    pc.printf("ResetEnc1: %d\r\n", rc.ReadEncM1());
    pc.printf("ResetEnc2: %d\r\n", rc.ReadEncM2());

    wait(5);

    setPos(-90);
    wait(5);
    setPos(90);
    wait(5);
    setPos(0);
    wait(5);

    pc.printf("End\r\n\r\n");
}
