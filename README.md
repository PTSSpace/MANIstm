# MANIstm

The repository for the code that runs on MANIs STM32 microcontrollers.

## CAN

The CAN implementation is based on Mbed CAN. Further work must be done concerning the interrupt handling and treading. It seems that only one thread at a time can be running next to the main programm thread.

**Important:**
As described in https://forums.mbed.com/t/unable-to-use-can-read-when-using-can-interrupt/4325, the CAN interface functions should not be called from within the ISR due to the mutex locks. But to clear the interrupt handler when receiving a CAN message, can.read must be called from the ISR. The workaround at the moment is disabling the mutex lock in the read and write functions. 
In the future a better way shall be found to clear the intterupt register.

## Interrupt Handling

https://os.mbed.com/blog/entry/Simplify-your-code-with-mbed-events/

## PID Control

This Code is based on a software PID controller implementation

### Tuning

The tuning parameters will be found empirically and hardcoded into the library.

### Datasheet

Information about the motor and encoder can be found at: https://www.pololu.com/product/1440
The encoder maximal precision is 64 counts per revolution on counting every rising and falling edge on both channels (X4_ENCODING).

### Library

This Code is based on multiple Mbed libraries that were adapted to our purpose:

* The Arduino PID library adjustment for arm: https://os.mbed.com/users/aberk/code/PID/

* The quadrature encoder class: https://os.mbed.com/users/aberk/code/QEI/

Since the code is run on the BluePill (STM32F103C8T6) the following files had to be imported in to the project: https://os.mbed.com/users/hudakz/code/mbed-STM32F103C8T6/

At the moment CAN Nucleo for Mbed is still used but since general CAN support for Mbed exists by now the Code will be adjusted for this as in: https://os.mbed.com/users/hudakz/code/CAN_Hello/