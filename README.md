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

#### TODO

Since disabeling the mutex locks is not a nice fix, an alternative to clear the interrupt flags shall be found.