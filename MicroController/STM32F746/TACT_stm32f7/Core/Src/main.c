///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2023 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "i2c.h"
//#include "usart.h"
//#include "gpio.h"
//#include "hidden_layer.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//
//#include <stdio.h>
//#include "vl53l0x_api.h"
//
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//
//
//#define VL53L0X_ADDR	0x29 << 1 // Default I2C address of VL53L0X
//#define NUM_SENSOR		24
//
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//
///* USER CODE BEGIN PV */
//
//
//
//
//
//
//
//
///* UART1 rx data */
//uint8_t rx1_data;
//
///* Variables for VL35L0 */
//uint8_t Message[64];
//uint8_t MessageLen;
//
//VL53L0X_RangingMeasurementData_t RangingData;
//
//
//
//
//
////uint8_t tca_ch[8] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};
////uint8_t tca_ch_reset = 0b00000000;
//
//
//
//
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
///* USER CODE BEGIN PFP */
//
///* Setting for "printf" */
//int __write(int file, char* P, int len);
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//
//
//
//
//
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//
//	// VL53L0X initialization stuff
//	//
//	uint32_t refSpadCount = 0;
//	uint8_t isApertureSpads = 0;
//	uint8_t VhvSettings = 0;
//	uint8_t PhaseCal = 0;
//
////	VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR]; // ?��?�� 배열
//	VL53L0X_Dev_t vl53l0x_s; // ?��?�� 배열
//
//	VL53L0X_DEV Dev;
//	uint16_t distance[NUM_SENSOR] = {0,};
//
//	uint8_t tca_ch[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // control register of TCA9548A
//	uint8_t tca_ch_reset = 0x00;
//
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_I2C1_Init();
//  MX_USART1_UART_Init();
//  /* USER CODE BEGIN 2 */
//
//
//
//
//
//
//
//  /* UART interrupt initialization */
//  MessageLen = sprintf((char*)Message, "JH VL53L0X test\n\r");
//
//  HAL_UART_Transmit(&huart1, Message, MessageLen, 100);
//
//
//
//		HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &tca_ch_reset, 1, 1000);
//		HAL_I2C_Master_Transmit(&hi2c1, 0x71 << 1, &tca_ch_reset, 1, 1000);
//		HAL_I2C_Master_Transmit(&hi2c1, 0x72 << 1, &tca_ch_reset, 1, 1000);
//
//		for (int i = 0; i < NUM_SENSOR; i++) {
//
//			uint8_t q = i / 8;
//			uint8_t r = i % 8;
//
//			if(q == 0){
//				HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &tca_ch[r], 1, 1000);
//				HAL_I2C_Master_Transmit(&hi2c1, 0x71 << 1, &tca_ch_reset, 1, 1000);
//				HAL_I2C_Master_Transmit(&hi2c1, 0x72 << 1, &tca_ch_reset, 1, 1000);
//
//			}else if(q == 1){
//				HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &tca_ch_reset, 1, 1000);
//				HAL_I2C_Master_Transmit(&hi2c1, 0x71 << 1, &tca_ch[r], 1, 1000);
//				HAL_I2C_Master_Transmit(&hi2c1, 0x72 << 1, &tca_ch_reset, 1, 1000);
//
//			}else if(q == 2){
//				HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &tca_ch_reset, 1, 1000);
//				HAL_I2C_Master_Transmit(&hi2c1, 0x71 << 1, &tca_ch_reset, 1, 1000);
//				HAL_I2C_Master_Transmit(&hi2c1, 0x72 << 1, &tca_ch[r], 1, 1000);
//			}
//
//			Dev = &vl53l0x_s;
//			Dev->I2cHandle = &hi2c1;
//			Dev->I2cDevAddr = VL53L0X_ADDR;
//
//			VL53L0X_WaitDeviceBooted( Dev );
//			VL53L0X_DataInit( Dev );
//			VL53L0X_StaticInit( Dev );
//			VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
//			VL53L0X_PerformRefCalibration( Dev, &VhvSettings, &PhaseCal);
//			VL53L0X_PerformRefSpadManagement( Dev, &refSpadCount, &isApertureSpads);
//			VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
//			VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
//			VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
//			VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
//			VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, 33000);
//			VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
//			VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
//
//			MessageLen = sprintf((char*)Message, "%d complete \n\r",i);
//			HAL_UART_Transmit(&huart1, Message, MessageLen, 100);
//	    }
//
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//
//
//
//  while (1)
//  {
//
//  	  uint32_t start = HAL_GetTick();
//
//
//		   for (int i = 0; i < NUM_SENSOR; i++) {
//
//				uint8_t q = i / 8;
//				uint8_t r = i % 8;
//
//				if(q == 0){
//					HAL_I2C_Master_Transmit(&hi2c1, 0x71 << 1, &tca_ch_reset, 1, 1000);
//					HAL_I2C_Master_Transmit(&hi2c1, 0x72 << 1, &tca_ch_reset, 1, 1000);
//					HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &tca_ch[r], 1, 1000);
//
//				}else if(q == 1){
//					HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &tca_ch_reset, 1, 1000);
//					HAL_I2C_Master_Transmit(&hi2c1, 0x72 << 1, &tca_ch_reset, 1, 1000);
//					HAL_I2C_Master_Transmit(&hi2c1, 0x71 << 1, &tca_ch[r], 1, 1000);
//
//				}else if(q == 2){
//					HAL_I2C_Master_Transmit(&hi2c1, 0x70 << 1, &tca_ch_reset, 1, 1000);
//					HAL_I2C_Master_Transmit(&hi2c1, 0x71 << 1, &tca_ch_reset, 1, 1000);
//					HAL_I2C_Master_Transmit(&hi2c1, 0x72 << 1, &tca_ch[r], 1, 1000);
//				}
//
//		       Dev = &vl53l0x_s;
//
//		       VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500us
////		       VL53L0X_PerformSingleRangingMeasurement(Dev, &RangingData);
//		       if (RangingData.RangeStatus == 0) {
//
//		    	   //distance[i] = RangingData.RangeMilliMeter;
//		    	   in[i][0] = RangingData.RangeMilliMeter;
//
//		           MessageLen = sprintf((char*)Message, "%d ",distance[i]);
//		           HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//		       }else{
//		           MessageLen = sprintf((char*)Message, "NL ");
//		           HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//		       }
//		       if(r == 7){
////		           MessageLen = sprintf((char*)Message, "/ ");
////		           HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//		       }
//		   }
//
//			  uint32_t end = HAL_GetTick();
//
//
//			mat_mul_relu_first(w1, in, r1, b1);
//			mat_mul_relu_second(w2, r1, r2, b2);
//			mat_mul_relu_third(w3, r2, r3, b3);
//			mat_mul_relu_fourth(w4, r3, r4, b4);
//			mat_mul_output_fifth(w5, r4, r5, b5);
//
//			  uint32_t end2 = HAL_GetTick();
//
//
//
//			for(int i=0; i<3; i++){
//				MessageLen = sprintf((char*)Message, "%.8f ",r5[i][0]);
//				HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//			}
//
//
//
//
//
//		   MessageLen = sprintf((char*)Message, "\n");
//		   HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//
//		  MessageLen = sprintf((char*)Message, "%d ms\n",end-start);
//		  HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//
//		  MessageLen = sprintf((char*)Message, "%d ms\n",end2-end);
//		  HAL_UART_Transmit(&huart1, Message, MessageLen, 1000);
//
//
//
//
//
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLM = 25;
//  RCC_OscInitStruct.PLL.PLLN = 432;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = 2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Activate the Over-Drive mode
//  */
//  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///* USER CODE BEGIN 4 */
//
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
