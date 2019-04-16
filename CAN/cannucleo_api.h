/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modified by Zoltan Hudak    <hudakz@outlook.com>
 *
 */
#ifndef CANNUCLEO_API_H
#define CANNUCLEO_API_H

#include "device.h"
#include "PinNames.h"
#include "PeripheralNames.h"
#include "can_helper.h"

#if defined(TARGET_NUCLEO_F072RB) || \
    defined(TARGET_NUCLEO_F091RC)
    #include "stm32f0xx_hal_msp.h"
#elif defined(TARGET_NUCLEO_F103RB)
    #include "stm32f1xx_hal_msp.h"
#elif defined(TARGET_NUCLEO_F302R8) || \
    defined(TARGET_NUCLEO_F303RE) || \
    defined(TARGET_NUCLEO_F303K8) || \
    defined(TARGET_NUCLEO_F334R8) || \
    defined(TARGET_DISCO_F334C8)
    #include "stm32f3xx_hal_msp.h"
#elif defined(TARGET_NUCLEO_F446RE)
   #include "stm32f4xx_hal_msp.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    IRQ_RX,
    IRQ_TX,
    IRQ_ERROR,
    IRQ_OVERRUN,
    IRQ_WAKEUP,
    IRQ_PASSIVE,
    IRQ_ARB,
    IRQ_BUS,
    IRQ_READY
} CanIrqType;


typedef enum {
    MODE_RESET,
    MODE_NORMAL,
    MODE_SILENT,
    MODE_TEST_GLOBAL,
    MODE_TEST_LOCAL,
    MODE_TEST_SILENT
} CanMode;

typedef void (*can_irq_handler)(uint32_t id, CanIrqType type);

void          can_init     (PinName rd, PinName td, FunctionalState abom);
void          can_free     (void);
int           can_frequency(int hz);
void          can_irq_init (uint32_t id, can_irq_handler handler);
void          can_irq_free (void);
int           can_write    (CAN_Message, int cc);
int           can_read     (CAN_Message *msg, int handle);
int           can_mode     (CanMode mode);
int           can_filter   (uint32_t id, uint32_t mask, CANFormat format, int32_t handle);
void          can_reset    (void);
unsigned char can_rderror  (void);
unsigned char can_tderror  (void);
void          can_monitor  (int silent);
void          can_callback (void);

#ifdef __cplusplus
};
#endif

#endif    // MBED_CAN_API_H




