/*
 * hidden_layer.h
 *
 *  Created on: 2023. 3. 27.
 *      Author: JH_LAB
 */
/* Define to prevent recursive inclusion -------------------------------------*/


#ifndef INC_HIDDEN_LAYER_H_
#define INC_HIDDEN_LAYER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

extern float in[24][1];

extern float w1[64][24];
extern float b1[64][1];
extern float r1[64][1];

extern float w2[64][64];
extern float b2[64][1];
extern float r2[64][1];

extern float w3[32][64];
extern float b3[32][1];
extern float r3[32][1];

extern float w4[32][32];
extern float b4[32][1];
extern float r4[32][1];

extern float w5[3][32];
extern float b5[3][1];
extern float r5[3][1];


/* USER CODE END Private defines */

void mat_mul_relu_first(float w[64][24],float in[24][1],float r[64][1],float bias[64][1]);
void mat_mul_relu_second(float w[64][64],float in[64][1],float r[64][1],float bias[64][1]);
void mat_mul_relu_third(float w[32][64],float in[64][1],float r[32][1],float bias[32][1]);
void mat_mul_relu_fourth(float w[32][32],float in[32][1],float r[32][1],float bias[32][1]);
void mat_mul_output_fifth(float w[3][32],float in[32][1],float r[3][1],float bias[3][1]);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif




#endif /* INC_HIDDEN_LAYER_H_ */




