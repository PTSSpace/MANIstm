# MANIstm

The repository for the code that runs on MANIs STM32 microcontrollers.

## PID Control

This Code is based on a software PID controller implementation

### Tuning

The tuning parameters have been found empirically and are hardcoded into the library.
Further fine tuning will be performed on the final Rover setup.

### Datasheet

Information about the motor and encoder can be found at: https://www.pololu.com/product/1440
The encoder maximal precision is 64 counts per revolution on counting every rising and falling edge on both channels (X4_ENCODING).

### Library

This Code is based on multiple Mbed libraries that were adapted to our purpose:

* The Arduino PID library adjustment for arm: https://os.mbed.com/users/aberk/code/PID/

* The quadrature encoder class: https://os.mbed.com/users/aberk/code/QEI/

Since the code is run on the BluePill (STM32F103C8T6) the following files were imported in to the project: https://os.mbed.com/users/hudakz/code/mbed-STM32F103C8T6/. Thereby only the "PinNames.h" file is included and the rest of the files might be removed to minimise flash storage used.

The mbed CAN interface (part of mbed OS) was used: https://os.mbed.com/docs/mbed-os/v5.12/apis/can.html.
**IMPORTANT**: For the CAN interrupts to work the mutex locks in the read and write functions of the CAN interface had to be disabled. The issue being that mutex locks can not be acess from the ISR functions, but to clear the interrupt flag, the read/write functions have to be called from the ISR.

## CAN Bus
The CAN interface is set to **can0** but may be set to **can1** if desired.
The adequate pins are:

| Interface | CAN_STBY | CAN_RX | CAN_TX|
|:----------|:----------:|:--------:|:-------:|
| can0 | Pin 4 | Pin 5| Pin 7 |
| can0 | Pin 14 | Pin 15 | Pin 17 |

### Parameters

The CAN bus bit rate is adjsuted by setting **FREQUENCY**.
The maximum wheel velocity **MAX_VEL** can be set at the top of the Pyhton code on the OBC or in the defines.h file on the Drive Nodes.
If the parameters are changed, they must be adjusted on all nodes connected to the bus.

#### Message ID's:

Messages with lower numeric values for their ID's have higher priority on the CAN network. All message ID's are given in Hexadecimal.
To ensure the priority of specified commands, each command has its own range denoted by the letter in the hex numbers.
Each command further has its own indentifier number to indicate which node it is specified for or originating from. The OBC is the only communication point to the other nodes and does therefore not need an indentifier number.

##### ID List

| Wheel location on rover | Indentifier number | ID's |
|:------------------------|:------------------:|:----:|
| front_left | 1 | 0xXX1 |
| rear_left | 2 | 0xXX2 |
| rear_right | 3 | 0xXX3 |
| front_right | 4 | 0xXX4 |

##### Command List

| Message | ID's | Description | Sender | Receiver | Data length | Data division |
|:--------|:----:|:-----------:|:------:|:--------:|:-----------:|:-------------:|
| powerCmd | 0x0AX | Power switch command for steering/driving motor | OBC| Drive Node | 8 byte | steerMode \[0,1\] (bytes 1 to 4) driveMode \[0,1\] (bytes 5 to 8) |
| initialliseCmd | 0x0BX | Initialisatin command for odometry publisher and zeroing steering encoders | OBC| Drive Node | 8 byte | publisherMode \[0,1\] (bytes 1 to 4) zeroEncoders \[0,1\] (bytes 5 to 8) |
| orientationCmd | 0x0CX | Set orientation command | OBC| Drive Node | 4 byte | set_orientation \[-2147483647..2147483647\] (bytes 1 to 4) |
| velocityCmd | 0x0DX | Set velocity command | OBC| Drive Node | 4 byte | set_velocity \[-2147483647..2147483647\] (bytes 1 to 4) |
| orientationOdm | 0x0EX | Odometry feedback for reached steering orientation | Drive Node | OBC | 4 byte | current_orientation \[-2147483647..2147483647\] (bytes 1 to 4) |
| velocityOdm | 0x0FX | Odometry feedback of absolute encoder counts for rover distance traveled | Drive Node | OBC | 8 byte | pulses \[-2147483647..2147483647\] (bytes 1 to 4) revolutions \[-2147483647..2147483647\] (bytes 5 to 8) |

**The velocity and orientation are scaled values based on the maximal velocity and orientation, respectively.**