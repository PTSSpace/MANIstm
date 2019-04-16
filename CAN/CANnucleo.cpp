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
#include "CANnucleo.h"
#include "cmsis.h"

/**
 * @brief   Constructor
 * @note    Constructs an instance of CAN class
 * @param   rxPin: CAN Rx pin name
 * @param   txPin: CAN Tx pin name
 * @param   abom:  Automatic recovery from bus-off state (defaults to enabled)
 * @retval
 */
CAN::CAN(PinName rxPin, PinName txPin, FunctionalState abom /* = ENABLE */) :
    _irq() {
    can_init(rxPin, txPin, abom);
    can_irq_init((uint32_t)this, (&CAN::_irq_handler));
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
CAN::~CAN(void) {
    can_irq_free();
    can_free();
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int CAN::frequency(int f) {
    lock();
    int ret = can_frequency(f);
    unlock();
    return ret;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int CAN::write(CANMessage msg) {
    lock();
    int ret = can_write(msg, 0);
    unlock();
    return ret;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int CAN::read(CANMessage& msg, int handle) {
    lock();
    int ret = can_read(&msg, handle);
    unlock();
    return ret;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void CAN::reset(void) {
    lock();
    can_reset();
    unlock();
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
unsigned char CAN::rderror(void) {
    lock();
    unsigned char ret = can_rderror();
    unlock();
    return ret;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
unsigned char CAN::tderror(void) {
    lock();
    unsigned char ret = can_tderror();
    unlock();
    return ret;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void CAN::monitor(bool silent) {
    lock();
    can_monitor((silent) ? 1 : 0);
    unlock();
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int CAN::mode(Mode mode) {
    lock();
    int ret = can_mode((CanMode) mode);
    unlock();
    return ret;
}

/**
 * @brief   Sets up a CAN filter
 * @note    At the present, CANnucleo supports only mask mode and 32-bit filter scale.
 *          Identifier list mode filtering and 16-bit filter scale are not supported.
 *          There are 14 filters available (0 - 13) for the application to set up.
 *          Each filter is a 32-bit filter defined by a filter ID and a filter mask.
 *          If no filter is set up then no CAN message is accepted (received)!
 *          That's why filter #0 is set up in the constructor to receive all CAN messages by default.
 *          On reception of a message it is compared with filter #0. If there is a match, the message is stored.
 *          If there is no match, the incoming identifier is then compared with the next filter.
 *          If the received identifier does not match any of the identifiers configured in the filters,
 *          the message is discarded by hardware without disturbing the software.
 *
 * @param   id: 'Filter ID' defines the bit values to be compared with the corresponding received bits
 *
 * Mapping of 32-bits (4-bytes) : | STID[10:3] | STID[2:0] EXID[17:13] | EXID[12:5] |  EXID[4:0] IDE RTR 0 |
 *
 * STID - Stardard Identifier bits
 * EXID - Extended Identifier bits
 * [x:y]- bit range
 * IDE  - Identifier Extension bit (0 -> Standard Identifier, 1 -> Extended Identifier)
 * RTR  - Remote Transmission Request bit (0 -> Remote Transmission Request, 1 -> Standard message)
 *
 * @param   mask: 'Filter mask' defines which bits of the 'Filter ID' are compared with the received bits
 *                and which bits are disregarded.

 * Mapping of 32-bits (4-bytes) : | STID[10:3] | STID[2:0] EXID[17:13] | EXID[12:5] |  EXID[4:0] IDE RTR 0 |
 *
 * STID - Stardard Identifier bits
 * EXID - Extended Identifier bits
 * [x:y]- bit range
 * IDE  - Identifier Extension bit
 * RTR  - Remote Transmission Request bit
 *
 * 1 -> bit is considered
 * 0 -> bit is disregarded
 *
 * ----------------------------------------
 * Example of filter set up and filtering:
 * ----------------------------------------
*
 * Let's assume we would like to receive only messages 
 * with standard identifier STID = 0x0207  (STID[15:0] = 00000010 00000111)
 *
 * We map the STID to filter ID by shifting the bits appropriately:
 * Filter id = STID << (16 + (15 - 10)) = STID << 21 = 01000000 11100000 00000000 00000000 = 0x40E00000
 *
 * To compare only the bits representing STID we set the filter mask adequately:
 * Filter mask = 11111111 11100000 00000000 00000100 = 0xFFE00004
 *               |||||||| |||                    |
 *               -------- ---                    |
 *                   |     |                     |
 *            STID[10:3]  STID[2:0]             IDE   
 *
 * Recall that filter #0 has been set up in the constructor to receive all CAN messages by default.
 * So we have to reconfigure it. If we were set up filter #1 here then filter #0 would receive all the messages
 * and no message would reach filter #1!
 *
 * To reconfigure (set up) filter #0 we call:
 *     can.filter(0x0207 << 21, 0xFFE00004, CANAny, 0);
 *
 *             Only these bits of 'Filter id' (set to 1 here in 'Filter mask') are compared with the corresponding
 *             bits of received message (the others are disregarded)
 *                                |
 *                 ---------------------------------
 *                 |||||||| |||                    |
 *   Filter mask = 11111111 11100000 00000000 00000100 (= 0xFFE00004)
 *   Filter id   = 01000000 11100000 00000000 00000000 (= 0x40E00000)
 *                 |||||||| |||                    |
 *                 ---------------------------------
 *                                |
 *            To receive the message the values of these bits must match.
 *            Otherwise the message is passed to the next filter or
 *            discarded if this was the last active filter.
 *                                |
 *                 ---------------------------------
 *                 |||||||| |||                    |
 *   Received id = 01000000 11100000 00000000 00000010 (= 0x40E00002)
 *                             ||||| |||||||| ||||| ||
 *                             -----------------------
 *                                         |
 *                          These bits (set to 0 in 'Filter mask') are disregarded (masked).
 *                          They can have arbitrary values.
 *
 * NOTE: For the meaning of individual bits see the mapping of 32-bits explained above.
 *
 * @param   format: This parameter must be CANAny
 * @param   handle: Selects the filter. This parameter must be a number between 0 and 13.
 * @retval  0 - successful
 *          1 - error
 *          2 - busy
 *          3 - time out  
 */
int CAN::filter(unsigned int id, unsigned int mask, CANFormat format /* = CANAny */, int handle /* = 0 */) {
    lock();
    int ret = can_filter(id, mask, format, handle);
    unlock();
    return ret;
}

/**
 * @brief   Attaches handler funcion to CAN1 RX0 Interrupt
 * @note    Only CAN1 RX0 Interrupt supported
 * @param   fptr: pointer to a void (*)(void) function
 * @param   type: not used (only CAN1 RX0 Interrupt supported) 
 * @retval
 */
void CAN::attach(mbed::Callback<void()> func, IrqType type) {
    lock();
    HAL_NVIC_DisableIRQ(CAN_IRQ);
    if (func)
        _irq[(CanIrqType)type] = func;
    HAL_NVIC_EnableIRQ(CAN_IRQ);
    unlock();
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void CAN::_irq_handler(uint32_t id, CanIrqType type) {
    CAN*    handler = (CAN*)id;
    handler->_irq[type].call();
}

void CAN::lock() {
    _mutex.lock();
}

void CAN::unlock() {
    _mutex.unlock();
}












