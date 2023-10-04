/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include "vl53l0x_api.h"
#include <string.h>
//#include "ai_datatypes_defines.h"
//#include "ai_platform.h"
//#include "sine_model.h"
//#include "sine_model_data.h"





/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define VL53L0X_ADDR	0x29 << 1 // Default I2C address of VL53L0X
#define NUM_SENSOR		8
#define WINDOW_SIZE 5
#define DEBOUNCE_DELAY 20  // ?��바운?�� �??????????�� ?���????????? (�?????????리초)
#define M_PI 3.14159265358979323846



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */




/* Variables for VL35L0 */
uint8_t Message[64];
uint8_t MessageLen;
uint8_t startMessage = 0;

unsigned long timestamp; // Make sure it is declared
float input_data, output_data, sine_data; // Make sure they are declared and used

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* Setting for "printf" */
int __write(int file, char* P, int len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int32_t Read_HX711(void);
void UART_SendWeight(float weight);
void HX711_Init(void);
void DelayMicroseconds(uint32_t microseconds);
void DelayMilliseconds(uint32_t milliseconds);
void DWT_Init(void);
void DWT_Delay(uint32_t us);
void hx711_delay_us(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//	ai_handle sine_model = AI_HANDLE_NULL; // ?��경망 모델 ?��?��
//	ai_buffer ai_input[AI_SINE_MODEL_IN_NUM]; // ?��?�� 버퍼
//	ai_buffer ai_output[AI_SINE_MODEL_OUT_NUM]; // 출력 버퍼
//
//	ai_float activations[AI_SINE_MODEL_DATA_ACTIVATIONS_SIZE]; // ?��?��?�� 버퍼
//
//	// ?��경망 ?��?��?��?�� ?��?��
//	ai_error err = ai_sine_model_create(&sine_model, AI_SINE_MODEL_DATA_CONFIG);
//	if (err.type != AI_ERROR_NONE) {
//	    // ?��?��: ?��?�� ?��?���??? 코드
//	}
//
//	// ?��경망 ?��?��미터 ?��?��
//	ai_network_params ai_params = {
//	    AI_SINE_MODEL_DATA_WEIGHTS(ai_sine_model_data_weights_get()),
//	    AI_SINE_MODEL_DATA_ACTIVATIONS(activations)
//	};


  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  MX_TIM6_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);


  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "JH TACTS test\n\r"), 100);


   // Create instance of neural network

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {

	  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "Message end\n"), 100);

//	  float step = 2.0f * M_PI / 100.0f; // 간격 ?��?��
//	  for (uint32_t i = 0; i < 100; i++) // 1000개의 ?��?��?�� ?��?��?��
//	  {
//	      float input_data = i * step; // 0?��?�� 2*pi까�? �????? ?��?��
//
//	      ((float*)ai_input[0].data)[0] = input_data;
//
//	      timestamp = htim6.Instance->CNT;
//
//	      ai_sine_model_run(sine_model, &ai_input[0], &ai_output[0]);
//
//	      float output_data = ((float*)ai_output[0].data)[0];
//
//	      HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message,"Output: %f | Duration: %lu\r\n", output_data, htim6.Instance->CNT - timestamp), 100);
//	      float sine_data = 10*(input_data);
//		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "Input: %f, Sine: %f, Output: %f\n", input_data, sine_data, output_data), 100);
//	      HAL_Delay(100);
//
//	  }




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/* USER CODE BEGIN 4 */
#if 0
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (rxData != '\n' && rxBufferIndex < RX_BUFFER_SIZE - 1)
    {
      rxBuffer[rxBufferIndex++] = rxData;
    }
    else
    {
      rxBuffer[rxBufferIndex] = '\0';
      rxBufferIndex = 0;
      receivedFlag = 1; //
    }
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
  }
}
#endif

#if 0
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_8) //
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)) // B?�� 값을 ?��?��
    {
      encoderCount++;
    }
    else
    {
      encoderCount--;
    }
  }
  else if (GPIO_Pin == GPIO_PIN_15) // B?��?�� ???�� ?��?��?��?��
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) // A?�� 값을 ?��?��
    {
      encoderCount--;
    }
    else
    {
      encoderCount++;
    }
  }
}
#endif

#if 0
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t currentInterruptTime = HAL_GetTick(); // ?��?�� ?���???????? �?????????��?���????????

  // �?????????�� ?��?��?��?��로�??�� DEBOUNCE_DELAY ?��간이 경과?���???????? ?��?��?���???????? 무시
  if((currentInterruptTime - lastInterruptTime) < DEBOUNCE_DELAY) {
    return;  // ?�� ?��?��?��?��?�� 무시
  }

  // ?��?�� ?���???????? ?��?��?��?��
  lastInterruptTime = currentInterruptTime;

  if (GPIO_Pin == GPIO_PIN_8) // A 채널
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)) // B 채널 값을 ?���????????
    {
      encoderCount++;
    }
    else
    {
      encoderCount--;
    }
  }
  else if (GPIO_Pin == GPIO_PIN_15) // B 채널
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) // A 채널 값을 ?���????????
    {
      encoderCount--;
    }
    else
    {
      encoderCount++;
    }
  }
}
#endif




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
