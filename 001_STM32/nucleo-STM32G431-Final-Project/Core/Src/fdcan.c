/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "stdlib.h"
/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 16;
  hfdcan1.Init.NominalTimeSeg1 = 63;
  hfdcan1.Init.NominalTimeSeg2 = 16;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */
  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void Prepare_CANFilter(void)
{
	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = CALIBRATION_ID;
	sFilterConfig.FilterID2 = 0x7FF;
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* Configure global filter to reject all non-matching frames */
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		Error_Handler();
	}

	/* Activate Rx FIFO 0 notification */
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}

}

void CAN_Start(void)
{
	/* Start the FDCAN module */
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
	    Error_Handler();
	}
}

/**
  * @brief  Creates the CAN message header and Rx filter.
  * @param  Desired CAN ID
  * @retval Pointer to CAN Message object.
  */
struct CANobject *GetCANMessage(uint32_t can_id)
{
	//FDCAN_FilterTypeDef sFilterConfig;
	struct CANobject *CAN_Message;

	CAN_Message = malloc(sizeof(struct CANobject));                                        //Memory reservation for new CAN msg object.

	for(uint8_t index = 0; index <= 7; index++) CAN_Message->Tx_Payload[index] = 0x00;     //Cleans the msg payload.

	/* Prepare Tx Header */
	CAN_Message->TxHeader.Identifier = can_id;
	CAN_Message->TxHeader.IdType = FDCAN_STANDARD_ID;
	CAN_Message->TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	CAN_Message->TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	CAN_Message->TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	CAN_Message->TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	CAN_Message->TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	CAN_Message->TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	CAN_Message->TxHeader.MessageMarker = 0;

	return CAN_Message;
}

/**
  * @brief  Rx FIFO 0 callback.
  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @param  RxFifo0ITs indicates which Rx FIFO 0 interrupts are signaled.
  *         This parameter can be any combination of @arg FDCAN_Rx_Fifo0_Interrupts.
  * @retval None
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	/* Retrieve Rx messages from RX FIFO0*/
	HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN_MSG_Received.RxHeader, CAN_MSG_Received.Rx_Payload);

	/* Check if CAN msg is standard CAN and if identifier is the calibration id (0x726) */
	if ((CAN_MSG_Received.RxHeader.Identifier == CALIBRATION_ID) && (CAN_MSG_Received.RxHeader.IdType == FDCAN_STANDARD_ID))
	{
	    xTaskNotifyFromISR((TaskHandle_t)CAN_Rx_CtrlrHandle, 0, eNoAction, NULL);
	}

}

/**
  * @brief  Sends a negative response to the CAN network.
  * @param  CANobject pointer to a CANobject structure that contains
  *         the configuration information for the specified CAN msg.
  * @retval None
  */
void send_negative_CAN_Rx(CANobject *CAN_Message)
{
	CAN_Message->Tx_Payload[0] = 0x07;                             //Sets the 0x762 header for response part 1.
	CAN_Message->Tx_Payload[1] = 0x62;
	CAN_Message->Tx_Payload[2] = 0x7F;							   //7F is negative Response.

	HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &CAN_Message->TxHeader, CAN_Message->Tx_Payload); //Sends negative response.
}
/* USER CODE END 1 */
