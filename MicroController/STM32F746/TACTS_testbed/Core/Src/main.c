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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include "vl53l0x_api.h"
#include "motor.h"
#include <string.h>
#include "core_cm7.h"
#include "stm32f7xx.h"




/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// HX711 pins
#define HX711_SCK_GPIO_Port GPIOI // D13
#define HX711_SCK_Pin GPIO_PIN_1  // CLK connected to D13 (PI1)
#define HX711_DT_GPIO_Port GPIOB  // D12
#define HX711_DT_Pin GPIO_PIN_14  // DT connected to D12 (PB14)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define VL53L0X_ADDR	0x29 << 1 // Default I2C address of VL53L0X
#define NUM_SENSOR		4


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rxBuffer[RX_BUFFER_SIZE];
uint16_t rxBufferIndex = 0;
uint8_t rxData;
uint8_t receivedFlag = 0; // ?��?�� ?��?��그�?? 추�??��?��?��.

/* UART1 rx data */
uint8_t rx1_data;

/* Variables for VL35L0 */
uint8_t Message[64];
uint8_t MessageLen;

VL53L0X_RangingMeasurementData_t RangingData;


volatile int32_t encoderCount = 0;
uint32_t count = 0;
uint32_t data = 0;
volatile uint32_t ms_tick_count = 0;


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

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

/*
	// VL53L0X initialization stuff
	//
	uint32_t refSpadCount = 0;
	uint8_t isApertureSpads = 0;
	uint8_t VhvSettings = 0;
	uint8_t PhaseCal = 0;

	VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR];
//	VL53L0X_Dev_t vl53l0x_s;

	VL53L0X_DEV Dev;
	KalmanFilter kalman_filters[NUM_SENSOR];
	uint16_t distance[NUM_SENSOR] = {0,};
	float filtered_distance[NUM_SENSOR] = {0,};

	uint8_t tca_ch[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // control register of TCA9548A
	//uint8_t tca_ch[8] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};
	uint8_t tca_ch_reset = 0x00;
	//uint8_t tca_ch_reset = 0b00000000;
    uint8_t tca_addr[] = {0x70,0x71,0x72};

*/



  // Variables to store load cell data
  int32_t rawData;
  float weight;


  float servo_dist;
  float step_rev_angle;
  float step_lin_dist;



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_UART_Receive_IT(&huart1, &rxData, 1);

  // Initialize the HX711
  HX711_Init();

  /* UART interrupt initialization */
  MessageLen = sprintf((char*)Message, "JH VL53L0X test\n\r");
  HAL_UART_Transmit(&huart1, Message, MessageLen, 100);

/*

		for (int i = 0; i < sizeof(tca_addr); i++) {
		    HAL_I2C_Master_Transmit(&hi2c1, tca_addr[i] << 1, &tca_ch_reset, 1, 1000);
		}

		for (int i = 0; i < NUM_SENSOR; i++) {

			uint8_t q = i / 8;
			uint8_t r = i % 8;

		    for (int j = 0; j < sizeof(tca_addr); j++) {
		        uint8_t *channel = (j == q) ? &tca_ch[r] : &tca_ch_reset;
		        HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, channel, 1, 1000);
		    }

			Dev = &vl53l0x_s[i];
			Dev->I2cHandle = &hi2c1;
			Dev->I2cDevAddr = VL53L0X_ADDR;

			VL53L0X_WaitDeviceBooted( Dev );
			VL53L0X_DataInit( Dev );
			VL53L0X_StaticInit( Dev );
			VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
			VL53L0X_PerformRefCalibration( Dev, &VhvSettings, &PhaseCal);
			VL53L0X_PerformRefSpadManagement( Dev, &refSpadCount, &isApertureSpads);
			VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
			VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
			VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
			VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
			VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, 33000);
			VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
			VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);


			// KalmanFilter initializer BEGIN //
			float Q = 0.1f; // Process noise covariance
			float R = 1.0f;   // Measurement noise covariance
			KalmanFilter_Init(&kalman_filters[i], Q, R);
			// KalmanFilter initializer END //


			MessageLen = sprintf((char*)Message, "%d complete \n\r",i);
			HAL_UART_Transmit(&huart1, Message, MessageLen, 100);
	    }
*/


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {

//  	  uint32_t start = HAL_GetTick();

/*
		   for (int i = 0; i < NUM_SENSOR; i++) {

				uint8_t q = i / 8;
				uint8_t r = i % 8;

			    for (int j = 0; j < sizeof(tca_addr); j++) {
			        uint8_t *channel = (j == q) ? &tca_ch[r] : &tca_ch_reset;
			        HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, channel, 1, 1000);
			    }

		       Dev = &vl53l0x_s[i];

		       VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500us
		       if (RangingData.RangeStatus == 0) {
		    	   distance[i] = RangingData.RangeMilliMeter;
		       }else{
		    	   distance[i] = 0;
		       }

	           MessageLen = sprintf((char*)Message, "%d ",distance[i]);
	           HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);

	           filtered_distance[i] = KalmanFilter_Update(&kalman_filters[i], (float)distance[i]);

	           MessageLen = sprintf((char*)Message, "%.3f ",filtered_distance[i]);
			   HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);

		       in[i][0] = distance[i];

		   }

//			uint32_t end = HAL_GetTick();
//
//
//			mat_mul_relu_first(w1, in, r1, b1);
//			mat_mul_relu_second(w2, r1, r2, b2);
//			mat_mul_relu_third(w3, r2, r3, b3);
//			mat_mul_relu_fourth(w4, r3, r4, b4);
//			mat_mul_output_fifth(w5, r4, r5, b5);
//
//			uint32_t end2 = HAL_GetTick();



//		for(int i=0; i<3; i++){
//			MessageLen = sprintf((char*)Message, "%.8f ",r5[i][0]);
//			HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//		}

		MessageLen = sprintf((char*)Message, "\n");
		HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//
//		MessageLen = sprintf((char*)Message, "%d ms\n",end-start);
//		HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//
//		MessageLen = sprintf((char*)Message, "%d ms\n",end2-end);
//		HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
 *
 *
 */

//	  if (receivedFlag)
//	  {
//	    if (strncmp((char *)rxBuffer, "rev", 4) == 0)
//	    {
//	      float servo_dist, step_rev_angle, step_lin_dist;
//
//	      sscanf((char *)rxBuffer + 5, "%f,%f,%f", &servo_dist, &step_rev_angle, &step_lin_dist);
//
//	      stepRev(step_rev_angle);
//	      stepLin(step_lin_dist);
//	      servo_angle(&htim2, TIM_CHANNEL_1, servo_dist);
//
//	      uint8_t goodMsg[] = "good";
//	      HAL_UART_Transmit(&huart1, goodMsg, strlen((char *)goodMsg), 1000);
//
//	      uint8_t newline[2] = "\r\n";
//	      HAL_UART_Transmit(&huart1, newline, 2, 10);
//	    }
//	    receivedFlag = 0;
//	  }

	  uint32_t startTime, endTime, elapsedTime;

	  // Get the start time before executing your function
	  startTime = HAL_GetTick();


	    // Read the raw data from HX711
	    rawData = Read_HX711();

	    // Convert the raw data to weight (replace the calibration factor with your own)
	    weight = -rawData /1600.0000000f + 10300;


	  // Get the end time after executing your function
	  endTime = HAL_GetTick();

	  // Calculate the elapsed time
	  elapsedTime = endTime - startTime;


	    // Send the weight data over UART
	    UART_SendWeight(weight);

	    		MessageLen = sprintf((char*)Message, "%d ms\n",elapsedTime);
	    		HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);

	    // Add some delay (optional)


		// Convert the raw data to weight (replace the calibration factor with your own)
//		weight = rawData / 10000.0f;

//		Send the weight data over UART

//	    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
//	    DelayMicroseconds(1);
//	    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
//	    DelayMicroseconds(1);

//		HAL_Delay(10); // 1초에 ?�� 번씩 출력?��?��?��.





//	  char msg[20];
//	  float encoderAngle = encoderCount*360.0/4096.0;
//	  sprintf(msg, "Encoder value: %.2f\r\n", encoderAngle);
//	  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);
//
//	  HAL_Delay(10); // 1초에 ?�� 번씩 출력?��?��?��.




//	stepRev(step_rev_angle);
//	stepLin(step_lin_dist);
//	stepLin(-step_lin_dist);
//	servo_angle(&htim2, TIM_CHANNEL_1, servo_dist);
//	HAL_Delay(3000);
//	for(int dist = 0; dist<=16; dist++){
//        servo_angle(&htim2, TIM_CHANNEL_1, dist);
//        HAL_Delay(3000);
//
//		MessageLen = sprintf((char*)Message, "%d \n",dist);
//		HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//	}


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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
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
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/* USER CODE BEGIN 4 */

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
      receivedFlag = 1; // 문자?��?�� ?��?��?��?��?��?�� ?��리는 ?��?��그�?? ?��?��?��?��?��.
    }
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_8) // A?��?�� ???�� ?��?��?��?��
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


void HX711_Init(void)
{
  // Set the SCK pin to low
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
}




void UART_SendWeight(float weight)
{
  char buffer[32];
  int len = sprintf(buffer, "Weight: %.4f g\r\n", weight);

  // Send the buffer content via UART
  HAL_UART_Transmit(&huart1, (uint8_t *)buffer, len, 1000);
}
void DelayMicroseconds(uint32_t microseconds)
{
  uint32_t ticks = microseconds;
  while (ticks--)
  {
    __NOP();
  }
}





int32_t Read_HX711(void)
{
  int32_t data = 0;

  // Wait until the DT pin goes low
  while (HAL_GPIO_ReadPin(HX711_DT_GPIO_Port, HX711_DT_Pin) == GPIO_PIN_SET);

  // Read the 24-bit data
  for (int i = 0; i < 24; i++)
  {
    // Generate a clock pulse on SCK pin
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
    DelayMicroseconds(1);
    data = (data << 1);
    if (HAL_GPIO_ReadPin(HX711_DT_GPIO_Port, HX711_DT_Pin) == GPIO_PIN_SET)
    {
      data++;
    }
    HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
    DelayMicroseconds(1);
  }

  // Generate an additional 25th pulse to set the HX711 back to idle mode
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_SET);
  DelayMicroseconds(1);
  HAL_GPIO_WritePin(HX711_SCK_GPIO_Port, HX711_SCK_Pin, GPIO_PIN_RESET);
  DelayMicroseconds(1);


  // Return the 24-bit data
  return data;
}



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
