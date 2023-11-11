/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
uint8_t txMsg[64];
uint8_t rxMsg[65];
/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB7     ------> USART1_RX
    PA9     ------> USART1_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef ReceiveUartMessage(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    uint8_t receivedByte;
    uint16_t rxBufferIndex = 0;
    while(1)
    {
        if(HAL_UART_Receive(huart, &receivedByte, 1, 1000) == HAL_OK)
        {
            if(receivedByte == '\n') // Ï¢ÖÎ£å Î¨∏Ïûê Í∞êÏ?
            {
                buffer[rxBufferIndex] = '\0'; // Î¨∏Ïûê?ó¥?ùò ?Åù?ùÑ ?ëú?ãú
                return HAL_OK;
            }
            else
            {
                buffer[rxBufferIndex] = receivedByte; // Î¨∏Ïûê ???û•
                rxBufferIndex++;
                if(rxBufferIndex >= size) // Î≤ÑÌçº Ï¥àÍ≥º Î∞©Ï?
                {
                    buffer[rxBufferIndex - 1] = '\0'; // Î¨∏Ïûê?ó¥?ùò ?Åù?ùÑ ?ëú?ãú
                    return HAL_ERROR;
                }
            }
        }
    }
}

void startMsg(){
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "--------------------------------------------------------------\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "--------------------------------------------------------------\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "----- Auto Data Logging Device for TACTS made by JaeHyeong----\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "-----------rev XX : Rotaing Revolution Motor (Deg)------------\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "-----------lin XX : Moving Linear Motor (mm)------------------\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "-----------servo XX : Poking XX * 0.8 (mm)--------------------\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "-----------auto : Poking point and data logging---------------\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "-------------------------testbed_axial------------------------\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "--------------------------------------------------------------\n"), 100);
	HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "--------------------------------------------------------------\n"), 100);
}

/* USER CODE END 1 */
