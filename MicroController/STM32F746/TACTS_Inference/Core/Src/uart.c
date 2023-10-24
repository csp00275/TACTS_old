/*
 * uart.c
 *
 *  Created on: Oct 24, 2023
 *      Author: JH_LAB
 */

#include"uart.h"

UART_HandleTypeDef huart1;
uint8_t txMsg[64];
uint8_t rxMsg[65];


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


HAL_StatusTypeDef ReceiveUartMessage(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size)
{
    uint8_t receivedByte;
    uint16_t rxBufferIndex = 0;
    while(1)
    {
        if(HAL_UART_Receive(huart, &receivedByte, 1, 1000) == HAL_OK)
        {
            if(receivedByte == '\n') // 종료 문자 감지
            {
                buffer[rxBufferIndex] = '\0'; // 문자열의 끝을 표시
                return HAL_OK;
            }
            else
            {
                buffer[rxBufferIndex] = receivedByte; // 문자 저장
                rxBufferIndex++;
                if(rxBufferIndex >= size) // 버퍼 초과 방지
                {
                    buffer[rxBufferIndex - 1] = '\0'; // 문자열의 끝을 표시
                    return HAL_ERROR;
                }
            }
        }
    }
}

