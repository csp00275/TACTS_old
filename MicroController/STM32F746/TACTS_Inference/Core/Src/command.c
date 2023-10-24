/*
 * command.c
 *
 *  Created on: Oct 24, 2023
 *      Author: JH_LAB
 */

#include"uart.h"
#include<stdio.h>
void ProcessCommand(uint8_t *command)
{
    if (strcmp((char*)command, "echo") == 0) {
        HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "echo\n"), 100);
    } else if (strcmp((char*)command, "1") == 0) {
    	FirstCommand();
    } else if (strcmp((char*)command, "2") == 0) {
    	SencondCommand();
    } else {
        HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "Please insert correct command\n"), 100);
    }
}


void FirstCommand()
{
    HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "FirstCommand\n"), 100);

}

void SencondCommand()
{
    HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "SencondCommand\n"), 100);
}
