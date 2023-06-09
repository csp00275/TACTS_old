/*
 * motor.h
 *
 *  Created on: Apr 28, 2023
 *      Author: JH_LAB
 */

#include <stdint.h>

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

void stepRev(int ANG);
void stepLin(int ANG);
void servo_angle(TIM_HandleTypeDef *htim, uint32_t channel, int angle);
/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif


#endif /* INC_MOTOR_H_ */





