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
#include "vl53l0x_jh.h"
#include "motor.h"
#include "hx711.h"
#include <string.h>
#include "kalman.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


float rawData = 0;
HAL_StatusTypeDef status;

uint32_t time_diff =0;
uint32_t start_time =0;
uint32_t end_time =0;
int8_t startMessage = 0;

uint32_t start_section_time = 0;
uint32_t end_section_time = 0;
uint32_t elapsed_section_time =0;


/* UART1 rx data */
uint8_t rx1_data;

volatile uint32_t lastInterruptTime = 0;
volatile int32_t encoderCount = 0;
uint32_t data = 0;
volatile uint32_t ms_tick_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void DelayMicroseconds(uint32_t microseconds);
void DelayMilliseconds(uint32_t milliseconds);
void ProcessCommand(uint8_t *command);
void RevCommand(char *arg);
void LinCommand();
void ServoCommand();
void SensorCommand();
void AutoCommand();
void FirstCommand();
void SecondCommand();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void ProcessCommand(uint8_t *commandBuffer)
{

    char *command = strtok((char*)commandBuffer, " "); // 명령어 분리

    // 'strtok'는 다음 호출 때 NULL을 사용하여 이전 문자열에서 계속 토큰을 추출
    char *argument = strtok(NULL, " "); // 인자 분리

    if (strcmp((char*)command, "echo") == 0) {
        HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "echo\n"), 100);
    }else if (strcmp((char*)command, "rev") == 0) {
		if (argument != NULL) {
			RevCommand(argument);
		}
    }else if (strcmp((char*)command, "lin") == 0) {
    	LinCommand(argument);
    }else if (strcmp((char*)command, "servo") == 0) {
    	ServoCommand(argument);
    }else if (strcmp((char*)command, "sensor") == 0) {
    	SensorCommand();
    }else {
        HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "Please insert correct command\n"), 100);
    }
}

void RevCommand(char *arg){
    int step_rev_angle;
    if(sscanf(arg, "%d", &step_rev_angle) == 1){
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d deg rev Still moving \n\r",step_rev_angle), 100);
        stepRev(step_rev_angle);
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d deg rev End \n\r",step_rev_angle), 100);
    }else{
    	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "invalid data\r\n"), 100);
    }

    HAL_Delay(1000); // Delay for 1 second
}

void LinCommand(char *arg){
    int step_lin_dist = 0;
    if (sscanf(arg, "%d", &step_lin_dist) == 1) {
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d mm lin Still moving \n\r",step_lin_dist), 100);
        stepLin(step_lin_dist);
        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d mm lin End\n\r", step_lin_dist), 100);
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "invalid data\r\n"), 100);
    }
    HAL_Delay(1000); // Delay for 1 second
}

void ServoCommand(char *arg){
    float servo_angle_val = 0;
    if(sscanf(arg, "%f", &servo_angle_val) == 1) {
    	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f servo Still moving \n\r", servo_angle_val), 100);
    	servo_angle(&htim2, TIM_CHANNEL_1, servo_angle_val);
    	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f servo End \n\r", servo_angle_val), 100);
        HAL_Delay(2000); // Delay for 2 seconds
        servo_angle(&htim2, TIM_CHANNEL_1, 0); // return to servo origin
    }else{
        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "invalid data\r\n"), 100);
    }
}


void SensorCommand(){

	uint32_t start_section_time, end_section_time,elapsed_section_time;
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "sensor test\r\n"), 100);

    start_time = HAL_GetTick();
    for(int count =0; count < 100; count++){
  	  start_section_time = HAL_GetTick();

  	  /// Read the VL53l0x data ///
        for (int i = 0; i < NUM_SENSOR; i++) {

    	    uint8_t q = i / 12;
    	    uint8_t r = i % 12;
    	    uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
    	    uint8_t channel = (r >= 8) ? r - 8 : r;

    	    resetTcaDevicesExcept(active_device, tca_addr);
            setActiveTcaChannel(active_device, channel, tca_addr);
            Dev = &vl53l0x_s[i];
            VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500us
            excuteVl53l0x(&vl53l0x_s[i],i);

        }

		  end_section_time = HAL_GetTick();
		  elapsed_section_time = end_section_time - start_section_time;
		  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%lu ms ", elapsed_section_time), 100);

		  rawData = Read_HX711();
		  UART_SendWeight_g(rawData,loadcell_slope,loadcell_bias); // Send the weight data over UART


        end_time = HAL_GetTick();
        time_diff = end_time - start_time;

        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);
    }
}

void AutoCommand(){
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "kalman steady state\r\n"), 100);
    start_time = HAL_GetTick();
	for(int count =0; count < 100; count++){
	  start_section_time = HAL_GetTick();
	  for (int i = 0; i < NUM_SENSOR; i++) {
			uint8_t q = i / 12;
			uint8_t r = i % 12;
			uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
			uint8_t channel = (r >= 8) ? r - 8 : r;
			resetTcaDevicesExcept(active_device, tca_addr);
			setActiveTcaChannel(active_device, channel, tca_addr);
			excuteVl53l0x(&vl53l0x_s[i],i);
		}
		end_section_time = HAL_GetTick();
		elapsed_section_time = end_section_time - start_section_time;
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%lu ms ", elapsed_section_time), 100);
		rawData = Read_HX711();
		UART_SendWeight_g(rawData,loadcell_slope,loadcell_bias); // Send the weight data over UART
		end_time = HAL_GetTick();
		time_diff = end_time - start_time;
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);
	}
	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "autoMode\r\n"), 100);
	servo_angle(&htim2, TIM_CHANNEL_1, 1); // poking
  	 for(int lin = 0; lin < 21; lin ++){
		 for(int rev = 0; rev < 18; rev++){
			 for(int r = 1; r < 8; r++){
				 servo_angle(&htim2, TIM_CHANNEL_1, r+2); // poking
				 HAL_Delay(500);
				 for(int count = 0; count < 100; count++){
					  for (int i = 0; i < NUM_SENSOR; i++) {
						uint8_t q = i / 12;
						uint8_t r = i % 12;
						uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
						uint8_t channel = (r >= 8) ? r - 8 : r;
						resetTcaDevicesExcept(active_device, tca_addr);
						setActiveTcaChannel(active_device, channel, tca_addr);
						excuteVl53l0x(&vl53l0x_s[i],i);
					  }

				  rawData = Read_HX711();
				  UART_SendWeight_g(rawData,loadcell_slope,loadcell_bias); // Send the weight data over UART
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, " "), 500);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d ",8*lin), 500);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d ",20*rev), 500);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f",r*0.8), 500);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 500);
				 }
			 HAL_Delay(500);
			 servo_angle(&htim2, TIM_CHANNEL_1, 0); // turn to origin
			 HAL_Delay(500);
			 }
			 stepRev(20); // revolution
		 }
		 stepRev(-360);
		 stepLin(-8); // moving horizontal
  	 }
}

void FirstCommand()
{
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "sensor test\r\n"), 100);

    uint32_t timeStart_s, timeEnd_s,timeDiff_s; // single
    uint32_t timeStart_a, timeEnd_a, timeDiff_a; // all

    timeStart_a = HAL_GetTick();
    do {
    	timeStart_s = HAL_GetTick();
        for (int i = 0; i < NUM_SENSOR; i++) {
    	    uint8_t q = i / 12;
    	    uint8_t r = i % 12;
    	    uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
    	    uint8_t channel = (r >= 8) ? r - 8 : r;
    	    resetTcaDevicesExcept(active_device, tca_addr);
            setActiveTcaChannel(active_device, channel, tca_addr);
            excuteVl53l0x(&vl53l0x_s[i],i);
        }
		timeEnd_s = HAL_GetTick();
		timeDiff_s = timeEnd_s - timeStart_s;
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%lu ms ", timeDiff_s), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);
		timeEnd_a = HAL_GetTick();
		timeDiff_a = timeEnd_a - timeStart_a;
    } while (timeDiff_a < 10000);
}

void SecondCommand(){

}


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

  // Initialize the HX711
  HX711_Init();

  /* UART interrupt initialization */
  initializeAllSensors(tca_addr, vl53l0x_s, filters);
  startMsg();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  if(ReceiveUartMessage(&huart1, rxMsg, sizeof(rxMsg)) == HAL_OK)
	  {
	      ProcessCommand(rxMsg);
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
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
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
}

/* USER CODE BEGIN 4 */


#if 1
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
#endif

#if 0
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t currentInterruptTime = HAL_GetTick(); // 현재 시간 가져오기

  // 지난 인터럽트로부터 DEBOUNCE_DELAY 시간이 경과하지 않았다면 무시
  if((currentInterruptTime - lastInterruptTime) < DEBOUNCE_DELAY) {
    return;  // 이 인터럽트는 무시
  }

  // 현재 시간 업데이트
  lastInterruptTime = currentInterruptTime;

  if (GPIO_Pin == GPIO_PIN_8) // A 채널
  {
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)) // B 채널 값을 읽기
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
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)) // A 채널 값을 읽기
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
