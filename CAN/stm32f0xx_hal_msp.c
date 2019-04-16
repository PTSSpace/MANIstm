 /**
  ******************************************************************************
  * @file    stm32f0xx_hal_msp.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-December-2014
  * @brief   HAL MSP module.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  *
  * Modified by Zoltan Hudak    <hudakz@outlook.com>
  *
  ******************************************************************************
  */
#if defined(TARGET_NUCLEO_F072RB) || \
    defined(TARGET_NUCLEO_F091RC)

#include "cannucleo_api.h"
#include "pinmap.h"

CAN_HandleTypeDef   _canHandle;
CanRxMsgTypeDef     _canRxMsg;
CanTxMsgTypeDef     _canTxMsg;
PinName             _rxPin;
PinName             _txPin;

void (*rxCompleteCallback)(void);

/**
  * @brief  CAN initialization.
  * @param  obj: can_t object
  * @param  rxPin: RX pin name
  * @param  txPin: TX pin name
  * @param  abom: Automatic recovery from bus-off state
  * @retval None
  */
void initCAN(PinName rxPin, PinName txPin, FunctionalState abom) {
    _rxPin = rxPin;
    _txPin = txPin;

    _canHandle.Instance = ((CAN_TypeDef*)CAN_BASE);
    _canHandle.pTxMsg = &_canTxMsg;
    _canHandle.pRxMsg = &_canRxMsg;

    _canHandle.Init.TTCM = DISABLE;
    _canHandle.Init.ABOM = abom;
    _canHandle.Init.AWUM = DISABLE;
    _canHandle.Init.NART = DISABLE;
    _canHandle.Init.RFLM = DISABLE;
    _canHandle.Init.TXFP = DISABLE;
    _canHandle.Init.Mode = CAN_MODE_NORMAL;

    // 125kbps bit rate (default)
    // APB1 peripheral clock = 48000000Hz
    _canHandle.Init.Prescaler = 24;     // number of time quanta = 48000000/24/125000 = 16
    _canHandle.Init.SJW = CAN_SJW_1TQ;
    _canHandle.Init.BS1 = CAN_BS1_11TQ; // sample point at: (1 + 11) / 16 * 100 = 75%
    _canHandle.Init.BS2 = CAN_BS2_4TQ;

    HAL_CAN_Init(&_canHandle);
}

/**
  * @brief  CAN MSP Initialization
  * @param  hcan: CAN handle pointer
  * @retval None
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
    GPIO_InitTypeDef    GPIO_InitStruct;

    if((_rxPin == PA_11) && (_txPin == PA_12)) {

        /* CAN1 Periph clock enable */
        __CAN_CLK_ENABLE();

        /* Enable GPIO clock */
        __GPIOA_CLK_ENABLE();

        /* CAN1 RX GPIO pin configuration */
        GPIO_InitStruct.Pin = GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate =  GPIO_AF4_CAN;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
        /* CAN1 TX GPIO pin configuration */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate =  GPIO_AF4_CAN;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else
    if((_rxPin == PB_8) && (_txPin == PB_9)) {
        /* CAN1 Periph clock enable */
        __CAN_CLK_ENABLE();

        /* Enable GPIO clock */
        __GPIOB_CLK_ENABLE();

        /* CAN1 RX GPIO pin configuration */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate =  GPIO_AF4_CAN;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        /* CAN1 TX GPIO pin configuration */
        GPIO_InitStruct.Pin = GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Alternate =  GPIO_AF4_CAN;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
    else
        return;
    /* NVIC configuration for CAN1 Reception complete interrupt */
    HAL_NVIC_SetPriority(CAN_IRQ, 1, 0);
    HAL_NVIC_EnableIRQ(CAN_IRQ);
}

/**
  * @brief CAN MSP De-Initialization
  *        This function frees the hardware resources used:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO to their default state
  * @param hcan: CAN handle pointer
  * @retval None
  */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {

    /* Reset peripherals */

    __CAN_FORCE_RESET();
    __CAN_RELEASE_RESET();

    /* Disable peripherals and GPIO Clocks */
    if((_rxPin == PA_11) && (_txPin == PA_12)) {
        /* De-initialize the CAN1 RX GPIO pin */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11);

        /* De-initialize the CAN1 TX GPIO pin */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_12);
    }
    else {

        /* De-initialize the CAN1 RX GPIO pin */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

        /* De-initialize the CAN1 TX GPIO pin */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_9);
    }


    /* Disable the NVIC for CAN reception */
    HAL_NVIC_DisableIRQ(CAN_IRQ);
}

/**
* @brief  Handles CAN RX interrupt request.
* @param  None
* @retval None
*/
void CEC_CAN_IRQHandler(void) {
    HAL_CAN_IRQHandler(&_canHandle);
}

/**
  * @brief  Reception  complete callback in non blocking mode
  * @param  _canHandle: pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _canHandle) {
    // if(HAL_CAN_Receive_IT(_canHandle, CAN_FIFO0) == HAL_OK) {
    //     if(rxCompleteCallback != NULL)
    //         rxCompleteCallback();
    // }
    // else {
    //     error_handler(error);
    // }

    // BUG: CAN race condition if HAL_CAN_Receive_IT() is used.
    // See https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Java/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2FSTM32Java%2FBUG%20CAN%20race%20condition%20if%20HAL%5FCAN%5FReceive%5FIT%20is%20used
    //
    // Fixed by Mark Burton:
    // ideally, we should be able to call HAL_CAN_Receive_IT() here to set up for another
    // receive but the API is flawed because that function will fail if HAL_CAN_Transmit()
    // had already locked the handle when the receive interrupt occurred - so we do what
    // HAL_CAN_Receive_IT() would do

    if (rxCompleteCallback != 0)
        rxCompleteCallback();

    if (_canHandle->State == HAL_CAN_STATE_BUSY_TX)
        _canHandle->State = HAL_CAN_STATE_BUSY_TX_RX;
    else {
        _canHandle->State = HAL_CAN_STATE_BUSY_RX;

        /* Set CAN error code to none */
        _canHandle->ErrorCode = HAL_CAN_ERROR_NONE;

        /* Enable Error warning Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_EWG);

        /* Enable Error passive Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_EPV);

        /* Enable Bus-off Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_BOF);

        /* Enable Last error code Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_LEC);

        /* Enable Error Interrupt */
        __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_ERR);
    }

    // Enable FIFO 0 message pending Interrupt
    __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_FMP0);
}
#endif



