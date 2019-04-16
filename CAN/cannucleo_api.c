/*
  ******************************************************************************
  * @file    can_api.c
  * @author  Zoltan Hudak
  * @version 
  * @date    04-August-2015
  * @brief   CAN api for NUCLEO-F103RB platform
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 Zoltan Hudak <hudakz@outlook.com>
  *
  * All rights reserved.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
  */
#include "cannucleo_api.h"
#include "can_helper.h"
#include "pinmap.h"

extern void (*rxCompleteCallback)(void);
extern CAN_HandleTypeDef _canHandle;

static uint32_t          irq_id = 0;
static can_irq_handler   irq_handler = 0;

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void can_init(PinName rd, PinName td, FunctionalState abom) {
    initCAN(rd, td, abom);
    can_filter(0, 0, CANAny, 0);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void can_free(void) {
    HAL_CAN_MspDeInit(&_canHandle);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int can_frequency(int hz) {
    HAL_NVIC_DisableIRQ(CAN_IRQ);
    
#if defined(TARGET_NUCLEO_F072RB) || \
    defined(TARGET_NUCLEO_F091RC)
    
        // APB1 peripheral clock = 48000000Hz
        
    switch(hz) {
    case 1000000:
        // 1000kbps bit rate
        _canHandle.Init.Prescaler = 4;      // number of time quanta = 48000000/4/1000000 = 12
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_8TQ;  // sample point at: (1 + 8) / 12 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_3TQ;
        break;
 
    case 500000:
        // 500kbps bit rate
        _canHandle.Init.Prescaler = 8;      // number of time quanta = 48000000/8/500000 = 12
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_8TQ;  // sample point at: (1 + 8) / 12 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_3TQ;
        break;
 
    case 250000:
        // 250kbps
        _canHandle.Init.Prescaler = 12;     // number of time quanta = 48000000/12/250000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
        break;
 
    case 125000:
        // 125kbps
        _canHandle.Init.Prescaler = 24;     // number of time quanta = 48000000/24/125000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
        break;
 
    default:
        // 125kbps (default)
        _canHandle.Init.Prescaler = 24;     // number of time quanta = 48000000/24/125000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
    }

#elif defined(TARGET_NUCLEO_F103RB) || \
    defined(TARGET_NUCLEO_F303RE) || \
    defined(TARGET_NUCLEO_F334R8) || \
    defined(TARGET_DISCO_F334C8)
    
    // APB1 peripheral clock = 36000000Hz

    switch(hz) {
    case 1000000:
        // 1000kbps bit rate
        _canHandle.Init.Prescaler = 3;      // number of time quanta = 36000000/3/1000000 = 12
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_8TQ;  // sample point at: (1 + 8) / 12 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_3TQ;
        break;

    case 500000:
        // 500kbps bit rate
        _canHandle.Init.Prescaler = 6;      // number of time quanta = 36000000/6/500000 = 12
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_8TQ;  // sample point at: (1 + 8) / 12 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_3TQ;
        break;

    case 250000:
        // 250kbps
        _canHandle.Init.Prescaler = 9;      // number of time quanta = 36000000/9/250000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
        break;

    case 125000:
        // 125kbps
        _canHandle.Init.Prescaler = 18;     // number of time quanta = 36000000/18/125000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
        break;

    default:
        // 125kbps (default)
#if DEBUG
        printf("Unknown frequency specified!\r\n");
        printf("Using default 125kbps\r\n");
#endif
        _canHandle.Init.Prescaler = 18;     // number of time quanta = 36000000/18/125000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
    }
    
#elif defined(TARGET_NUCLEO_F303K8)
    
    // APB1 peripheral clock = 32000000Hz

    switch(hz) {
    case 1000000:
        // 1000kbps bit rate
        _canHandle.Init.Prescaler = 2;      // number of time quanta = 32000000/2/1000000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
        break;

    case 500000:
        // 500kbps bit rate
        _canHandle.Init.Prescaler = 4;      // number of time quanta = 32000000/4/500000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
        break;

    case 250000:
        // 250kbps
        _canHandle.Init.Prescaler = 8;      // number of time quanta = 32000000/8/250000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
        break;

    case 125000:
        // 125kbps
        _canHandle.Init.Prescaler = 16;     // number of time quanta = 32000000/16/125000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
        break;

    default:
        // 125kbps (default)
#if DEBUG
        printf("Unknown frequency specified!\r\n");
        printf("Using default 125kbps\r\n");
#endif
        _canHandle.Init.Prescaler = 16;     // number of time quanta = 32000000/16/125000 = 16
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_4TQ;
    }

#elif defined(TARGET_NUCLEO_F446RE)
    
    // APB1 peripheral clock = 45000000Hz

    switch(hz) {
    case 1000000:
        // 1000kbps bit rate
        _canHandle.Init.Prescaler = 5;      // number of time quanta = 45000000/5/1000000 = 9
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_6TQ;  // sample point at: (1 + 6) / 9 * 100 = 77.78%
        _canHandle.Init.BS2 = CAN_BS2_2TQ;
        break;

    case 500000:
        // 500kbps bit rate
        _canHandle.Init.Prescaler = 10;      // number of time quanta = 45000000/10/500000 = 9
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_6TQ;  // sample point at: (1 + 6) / 9 * 100 = 77.78%
        _canHandle.Init.BS2 = CAN_BS2_2TQ;
        break;

    case 250000:
        // 250kbps
        _canHandle.Init.Prescaler = 15;      // number of time quanta = 45000000/15/250000 = 12
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_8TQ; // sample point at: (1 + 8) / 12 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_3TQ;
        break;

    case 125000:
        // 125kbps
        _canHandle.Init.Prescaler = 30;     // number of time quanta = 45000000/30/125000 = 12
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_8TQ; // sample point at: (1 + 8) / 12 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_3TQ;
        break;

    default:
        // 125kbps (default)
#if DEBUG
        printf("Unknown frequency specified!\r\n");
        printf("Using default 125kbps\r\n");
#endif
        _canHandle.Init.Prescaler = 30;     // number of time quanta = 45000000/30/125000 = 12
        _canHandle.Init.SJW = CAN_SJW_1TQ;
        _canHandle.Init.BS1 = CAN_BS1_8TQ; // sample point at: (1 + 8) / 12 * 100 = 75%
        _canHandle.Init.BS2 = CAN_BS2_3TQ;
    }
    
#endif

    HAL_CAN_Init(&_canHandle);
    HAL_NVIC_EnableIRQ(CAN_IRQ);
   
    return 1;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void can_callback(void) {
    irq_handler(irq_id, IRQ_RX);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void can_irq_init(uint32_t id, can_irq_handler handler) {
    irq_id = id;
    irq_handler = handler;
    rxCompleteCallback = can_callback;

    if(HAL_CAN_Receive_IT(&_canHandle, CAN_FIFO0) != HAL_OK) {
#ifdef DEBUG
        printf("CAN reception initialization error\r\n");
#endif
    }
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void can_irq_free(void) {
    rxCompleteCallback = 0;
}   

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int can_write(CAN_Message msg, int cc) {
    int i = 0;

    if(msg.format == CANStandard) {
        _canHandle.pTxMsg->StdId = msg.id;
        _canHandle.pTxMsg->ExtId = 0x00;
    }
    else {
        _canHandle.pTxMsg->StdId = 0x00;
        _canHandle.pTxMsg->ExtId = msg.id;
    }

    _canHandle.pTxMsg->RTR = msg.type == CANData ? CAN_RTR_DATA : CAN_RTR_REMOTE;
    _canHandle.pTxMsg->IDE = msg.format == CANStandard ? CAN_ID_STD : CAN_ID_EXT;
    _canHandle.pTxMsg->DLC = msg.len;

    for(i = 0; i < msg.len; i++)
        _canHandle.pTxMsg->Data[i] = msg.data[i];

    if(HAL_CAN_Transmit(&_canHandle, 10) != HAL_OK) {
#ifdef DEBUG
        printf("Transmission error\r\n");
#endif
        return 0;
    }
    else
        return 1;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int can_read(CAN_Message* msg, int handle) {
    int i;
    msg->id = _canHandle.pRxMsg->IDE == CAN_ID_STD ? _canHandle.pRxMsg->StdId : _canHandle.pRxMsg->ExtId;
    msg->type = _canHandle.pRxMsg->RTR == CAN_RTR_DATA ? CANData : CANRemote;
    msg->format = _canHandle.pRxMsg->IDE == CAN_ID_STD ? CANStandard : CANExtended;
    msg->len = _canHandle.pRxMsg->DLC;
    for(i = 0; i < msg->len; i++)
        msg->data[i] = _canHandle.pRxMsg->Data[i];
        
    return msg->len;
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int can_mode(CanMode mode) {
    switch(mode) {
    case MODE_RESET:
        return HAL_ERROR;

    case MODE_NORMAL:
        _canHandle.Init.Mode = CAN_MODE_NORMAL;
        break;

    case MODE_SILENT:
        _canHandle.Init.Mode = CAN_MODE_SILENT;
        break;

    case MODE_TEST_GLOBAL:
        _canHandle.Init.Mode = CAN_MODE_LOOPBACK;
        break;

    case MODE_TEST_LOCAL:
        _canHandle.Init.Mode = CAN_MODE_LOOPBACK;
        break;

    case MODE_TEST_SILENT:
        _canHandle.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
        break;
    }

    return HAL_CAN_Init(&_canHandle);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
int can_filter(uint32_t id, uint32_t mask, CANFormat format /*=CANAny*/, int32_t handle /*=0*/ ) {
    CAN_FilterConfTypeDef   sFilterConfig;

    sFilterConfig.FilterNumber = handle;    // Specifies the filter number (must be a number between 0 and 13 at 32-bit filter scale)
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = (((id) >> 16) & 0xFFFF);
    sFilterConfig.FilterIdLow = ((id) & 0xFFFF);
    sFilterConfig.FilterMaskIdHigh = (((mask) >> 16) & 0xFFFF);
    sFilterConfig.FilterMaskIdLow = ((mask) & 0xFFFF);
    sFilterConfig.FilterFIFOAssignment = 0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.BankNumber = 0;           // Selects the start bank filter
    return HAL_CAN_ConfigFilter(&_canHandle, &sFilterConfig);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void can_reset(void) {
    __HAL_CAN_RESET_HANDLE_STATE(&_canHandle);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
unsigned char can_rderror(void) {
    return HAL_CAN_GetError(&_canHandle);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
unsigned char can_tderror(void) {
    return HAL_CAN_GetError(&_canHandle);
}

/**
 * @brief
 * @note
 * @param
 * @retval
 */
void can_monitor(int silent) {

    // not implemented
}








