/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
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
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

static int count = 0;
uint8_t sensorData1[8]={0};
uint8_t sensorData2[8]={0};
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_5TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX1_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void CAN_transmit(uint8_t dataTx1[], uint8_t dataTx2[], uint8_t dataTx3[], uint8_t dataTx4[], uint8_t dataTx5[]){
#if FRONT
  static CAN_TxHeaderTypeDef txMsgFRightF = {.StdId = 0x200,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8
	};
  static CAN_TxHeaderTypeDef txMsgFLeftF = {.StdId = 0x202,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8
  };
  static CAN_TxHeaderTypeDef txMsgFRightS = {.StdId = 0x201,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8
  };
  static CAN_TxHeaderTypeDef txMsgFLeftS = {.StdId = 0x203,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8
  };
  static CAN_TxHeaderTypeDef txMsgStrainGauge = {.StdId = 0x204,
	.IDE = CAN_ID_STD,
	.RTR = CAN_RTR_DATA,
	.DLC = 8
  };

  uint32_t mailbox1, mailbox2, mailbox3, mailbox4, mailbox5;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
  HAL_CAN_AddTxMessage(&hcan1, &txMsgFRightF, dataTx1, &mailbox1);
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
  HAL_CAN_AddTxMessage(&hcan1, &txMsgFLeftF, dataTx2, &mailbox2);
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
  HAL_CAN_AddTxMessage(&hcan1, &txMsgStrainGauge, dataTx5, &mailbox5);
  if(count==99){
    count = -1;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
    HAL_CAN_AddTxMessage(&hcan1, &txMsgFRightS, dataTx3, &mailbox3);
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
    HAL_CAN_AddTxMessage(&hcan1, &txMsgFLeftS, dataTx4, &mailbox4);
  }
  count++;
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
#else
  static CAN_TxHeaderTypeDef txMsgRRightF = {.StdId = 0x210,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8
  };
  static CAN_TxHeaderTypeDef txMsgRLeftF = {.StdId = 0x212,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8
  };
  static CAN_TxHeaderTypeDef txMsgRRightS = {.StdId = 0x211,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8
  };
  static CAN_TxHeaderTypeDef txMsgRLeftS = {.StdId = 0x213,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8
  };
  static CAN_txHeaderTypeDef txMsgStrainGauge = {.StdId = 0x214,
	.IDE = CAN_ID_STD,
	.RTR = CAN_RTR_DATA,
	.DLC = 8
  };
  uint32_t mailbox1, mailbox2, mailbox3, mailbox4;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
  HAL_CAN_AddTxMessage(&hcan1, &txMsgRRightF, dataTx1, &mailbox1);
  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
  HAL_CAN_AddTxMessage(&hcan1, &txMsgRLeftF, dataTx2, &mailbox2);
  while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
  HAL_CAN_AddTxMessage(&hcan1, &txMsgStrainGauge, dataTx5, &mailbox5);
  if(count==99){
    count = -1;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
    HAL_CAN_AddTxMessage(&hcan1, &txMsgRRightS, dataTx3, &mailbox3);
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
    HAL_CAN_AddTxMessage(&hcan1, &txMsgRLeftS, dataTx4, &mailbox4);
  }
  count++;
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
#endif
}

// Function that transmits a separate CAN msg that contains the status of AnaCAN
void CAN_updateStatus(void){
	CAN_TxHeaderTypeDef txMsg_status = {.StdId = 0x242,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 4
	};

	uint32_t mailbox;
	uint8_t data[8] = {
		[0] = AN.state,
		[1] = AN.errorBits.can1Fail,
		[2] = AN.errorBits.can2Fail,
		[3] = AN.errorBits.spiFail,
	};
	while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {};
	HAL_CAN_AddTxMessage(&hcan1, &txMsg_status, data, &mailbox);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  static CAN_RxHeaderTypeDef hdr = {
    .IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 8
	};
  CAN_TxHeaderTypeDef txMsg = {
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
  };
  uint32_t mailbox;
	static uint8_t dataRx[8] = {0};
	if (hcan == &hcan2) {
	  HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &hdr, dataRx);
	  txMsg.StdId = hdr.StdId;
	  txMsg.DLC = hdr.DLC;
	  HAL_CAN_AddTxMessage(&hcan1, &txMsg, dataRx, &mailbox);
	}
  if (hcan == &hcan1) {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, dataRx);
    txMsg.StdId = hdr.StdId;
    txMsg.DLC = hdr.DLC;
    HAL_CAN_AddTxMessage(&hcan2, &txMsg, dataRx, &mailbox);
  }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
