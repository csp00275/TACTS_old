/*
 * uart.h
 *
 *  Created on: Oct 24, 2023
 *      Author: JH_LAB
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <main.h>

extern UART_HandleTypeDef huart1;

extern uint8_t txMsg[64];
extern uint8_t rxMsg[65];


void MX_USART1_UART_Init(void);
HAL_StatusTypeDef ReceiveUartMessage(UART_HandleTypeDef *huart, uint8_t *buffer, uint16_t size);


#endif /* INC_UART_H_ */
