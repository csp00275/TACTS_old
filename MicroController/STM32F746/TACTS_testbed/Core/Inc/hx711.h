/*
 * hx711.h
 *
 *  Created on: May 10, 2023
 *      Author: JH_LAB
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
extern float loadcell_slope;
extern float loadcell_bias;
// HX711 pins
#define HX711_SCK_GPIO_Port GPIOI // D13
#define HX711_SCK_Pin GPIO_PIN_1  // CLK connected to D13 (PI1)
#define HX711_DT_GPIO_Port GPIOB  // D12
#define HX711_DT_Pin GPIO_PIN_14  // DT connected to D12 (PB14)

void HX711_Init(void);
int32_t Read_HX711(void);
void UART_SendWeight_g(float rawData,float loadcell_slope,float loadcell_bias);
void UART_SendWeight_N(float rawData,float loadcell_slope,float loadcell_bias);
void DelayMicroseconds(uint32_t microseconds);

/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif



#endif /* INC_HX711_H_ */
