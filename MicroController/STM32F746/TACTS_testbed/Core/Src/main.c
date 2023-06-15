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
#include "hx711.h"
#include <string.h>





/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define VL53L0X_ADDR	0x29 << 1 // Default I2C address of VL53L0X
#define NUM_SENSOR		7


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

char string[100]; // 버퍼 크기 설정
HAL_StatusTypeDef status;

uint32_t time_diff =0;
uint32_t start_time =0;
uint32_t end_time =0;

uint8_t startMessage = 0;


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

  uint8_t buffer[100]; // 데이터를 저장할 버퍼 m
  uint8_t received_data;
  uint32_t string_index = 0;
  HAL_StatusTypeDef status;

	// VL53L0X initialization stuff
	//
	uint32_t refSpadCount = 0;
	uint8_t isApertureSpads = 0;
	uint8_t VhvSettings = 0;
	uint8_t PhaseCal = 0;

	VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR];
//	VL53L0X_Dev_t vl53l0x_s;

	VL53L0X_DEV Dev;
	//KalmanFilter kalman_filters[NUM_SENSOR];
	uint16_t distance[NUM_SENSOR] = {0,};
//	float filtered_distance[NUM_SENSOR] = {0,};

	uint8_t tca_ch[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // control register of TCA9548A
	//uint8_t tca_ch[8] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};
	uint8_t tca_ch_reset = 0x00;
	//uint8_t tca_ch_reset = 0b00000000;
    uint8_t tca_addr[] = {0x70};

//    uint8_t tca_addr[] = {0x70,0x71,0x72};


    HAL_UART_Receive_IT(&huart1,&rxData,1);


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
  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "JH TACTS test\n\r"), 100);

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

/*
			// KalmanFilter initializer BEGIN //
			float Q = 0.1f; // Process noise covariance
			float R = 1.0f;   // Measurement noise covariance
			KalmanFilter_Init(&kalman_filters[i], Q, R);
			// KalmanFilter initializer END //			 */
			HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d complete \n\r",i), 100);
	    }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  while (1)
  {
	  if(startMessage==0){
		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "--------------------------------------------------------------\n"), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "----- Auto Data Logging Device for TACTS made by JaeHyeong----\n"), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "-----------rev XX : Rotaing Revolution Motor (Deg)------------\n"), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "-----------lin XX : Moving Linear Motor (mm)------------------\n"), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "-----------servo XX : Poking XX * 0.8 (mm)--------------------\n"), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "-----------auto : Poking the designed point and data logging--\n"), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "-------------------------testbed_axial------------------------\n"), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "--------------------------------------------------------------\n"), 100);
		  startMessage =1;
	  }

	  if(receivedFlag)
	  {
		  ///////////////////// Step rev /////////////////////

		  char* command = "all";
		  if(strncmp((char*)rxBuffer, command,strlen(command)) == 0)
		     {
		         float servo_dist=0;
		         int step_rev_angle=0;
		         int step_lin_dist=0;

		         // Check if three integers are successfully parsed
		         if(sscanf((char*)rxBuffer + strlen(command)+1, "%f,%d,%d",&servo_dist, &step_rev_angle, &step_lin_dist) == 3)
		         {
		             stepRev(step_rev_angle);
		             stepLin(step_lin_dist);
		             servo_angle(&htim2, TIM_CHANNEL_1, servo_dist);
		             servo_dist *=0.8;

		 			HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d deg rev \n\r",step_rev_angle), 100);
					HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d mm lin \n\r",step_lin_dist), 100);
					HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%.2f servo \n\r",servo_dist), 100);
					HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "vaild data\r\n"), 100);

		         }
		         else
		         {
		        	 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "invalid data\r\n"), 100);
		         }
		         HAL_Delay(1000); // Delay for 1 second
		         receivedFlag = 0;
		     }

		  command = "rev";
		  if(strncmp((char*)rxBuffer, command,strlen(command)) == 0)
		     {
		         int step_rev_angle=0;
		         if(sscanf((char*)rxBuffer + strlen(command) + 1, "%d",&step_rev_angle) == 1)
		         {
					 stepRev(step_rev_angle);
		 			HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d deg rev \n\r",step_rev_angle), 100);
		         }
		         else
		         {
		        	 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "invalid data\r\n"), 100);
		         }
		         HAL_Delay(1000); // Delay for 1 second
		         receivedFlag = 0;
		     }

		  command = "lin";
		  if (strncmp((char*)rxBuffer, command, strlen(command)) == 0) {
		      int step_lin_dist = 0;
		      if (sscanf((char*)rxBuffer + strlen(command) + 1, "%d", &step_lin_dist) == 1) {
		          stepLin(step_lin_dist);
		          HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d mm %s \n\r", step_lin_dist, command), 100);
		          HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "valid data\r\n"), 100);
		      } else {
		          HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "invalid data\r\n"), 100);
		      }
		      HAL_Delay(1000);
		      receivedFlag = 0;
		  }

		  command = "servo";
		  if(strncmp((char*)rxBuffer, command,strlen(command)) == 0)
		     {
		         float servo_dist=0;
		         if(sscanf((char*)rxBuffer + strlen(command)+1, "%f",&servo_dist) == 1)
		         {
		            servo_angle(&htim2, TIM_CHANNEL_1, servo_dist);
		            servo_dist *=0.8;
					HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%.2f servo \n\r",servo_dist), 100);
					HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "vaild data\r\n"), 100);
		         }
		         else
		         {
		        	 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "invalid data\r\n"), 100);
		         }
		         HAL_Delay(2000); // Delay for 1 second

		         servo_angle(&htim2, TIM_CHANNEL_1, 0); // return to servo origin

		         receivedFlag = 0;
		     }

		  command = "auto";
		  if(strncmp((char*)rxBuffer, command,strlen(command)) == 0)
		     {
	        	 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "autoMode\r\n"), 100);

//		             stepRev(50); // revolution
//		             stepRev(0); // fix the position
//
//		             stepLin(10); // moving horizontal
//		             stepLin(-10); //

	        	 for( int lin = 0; lin < 16;lin ++){
	        		 stepLin(10); // moving horizontal
					 for(int rev = 0; rev<1; rev++){
						 stepRev(10); // revolution
						 for(int r = 0;r<16;r++){

							 servo_angle(&htim2, TIM_CHANNEL_1, r); // poking

							 ///////////////////////////////////////////////////////
							 ////////////////////Logging Start//////////////////////
							 ///////////////////////////////////////////////////////
							 start_time = HAL_GetTick(); // 시작 시간 측정
							 do{
							  /// Read the VL53l0x data ///
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
								 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d ",distance[i]), 100);
							   }
							   /// End of Reading Vl53l0x data ///


						  /// Read the raw data from HX711 ///
						  rawData = Read_HX711();
						  float loadcell_slope = -1/1600.00f; // Convert the raw data to weight (replace the calibration factor with your own)
						  float loadcell_bias = 10002;
						  UART_SendWeight_g(rawData,loadcell_slope,loadcell_bias); // Send the weight data over UART
						  /// End of Reading HX711 data ///

						  /// Read the raw data from AMT103 ///
						  float encoderAngle = encoderCount*360.0/4096.0;
						  HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, " %.2f ",encoderAngle), 100);
						  /// End of Reading AMT103 data ///

						 end_time = HAL_GetTick(); // 끝 시간 측정
						 time_diff = end_time - start_time; // 시간 차이 계산


						 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d ",lin), 100);
						 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d ",rev), 100);
						 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "%d",r), 100);
						 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "\n"), 100);

						 }while(time_diff<3000);
						 ///////////////////////////////////////////////////////
						 ////////////////////Logging End////////////////////////
						 ///////////////////////////////////////////////////////
						 servo_angle(&htim2, TIM_CHANNEL_1, 0); // turn to origin
						 HAL_Delay(500);
						 }
					 }
	        	 }


		         receivedFlag = 0;
		     }

     	 HAL_UART_Transmit(&huart1, (uint8_t*)Message, sprintf((char*)Message, "Message end\r\n"), 100);
	  }


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
