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
#include "motor.h"
#include "hx711.h"
#include <string.h>
#include "kalman.h"
#include "vl53l0x_api.h"
#include <math.h>
#include "avgstd.h"


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



/* USER CODE BEGIN Init */


HAL_StatusTypeDef status;

uint32_t time_diff =0;
uint32_t start_time =0;
uint32_t end_time =0;

uint32_t start_section_time = 0;
uint32_t end_section_time = 0;
uint32_t elapsed_section_time =0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void ProcessCommand(uint8_t *command);
void RevCommand(char *arg);
void LinCommand();
void ServoCommand();
void SensorCommand();
void AutoCommand();
void InitializaionCalibrationCommand();
void CalibrationCommand();
void SetSensorCommand();
void AvgStdCommand();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ProcessCommand(uint8_t *commandBuffer)
{
    char *command = strtok((char*)commandBuffer, " "); // Command separation
    // 'strtok'는 다음 호출 때 NULL을 사용하여 이전 문자열에서 계속 토큰을 추출
    char *argument = strtok(NULL, " "); // argument seperation

    if (strcmp((char*)command, "echo") == 0) {HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "echo\n"), 100);}
    else if (strcmp((char*)command, "rev") == 0) {RevCommand(argument);}
    else if (strcmp((char*)command, "lin") == 0) {LinCommand(argument);}
    else if (strcmp((char*)command, "servo") == 0) {ServoCommand(argument);}
    else if (strcmp((char*)command, "sensor") == 0) {SensorCommand();}
    else if (strcmp((char*)command, "cali") == 0) {CalibrationCommand();}
    else if (strcmp((char*)command, "ini") == 0) {InitializaionCalibrationCommand();}
    else if (strcmp((char*)command, "setsensor") == 0) {SetSensorCommand();}
    else if (strcmp((char*)command, "auto") == 0) {AutoCommand();}
    else if (strcmp((char*)command, "3") == 0) {CalibrationCommand();}
    else if (strcmp((char*)command, "4") == 0) {CalibrationCommand();}
    else {HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "Please insert correct command\n"), 100);}
}

void RevCommand(char *arg){
    int step_rev_angle;
    if(sscanf(arg, "%d", &step_rev_angle) == 1){
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d deg revolution Start \n\r",step_rev_angle), 100);
        stepRev(step_rev_angle);
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d deg revolution End \n\r",step_rev_angle), 100);
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

	ResetAllDevices();
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "sensor test\r\n"), 100);
    uint32_t startTime, endTime, diffTime;
    for(int count =0; count < NUM_READINGS; count++){
    	uint8_t sensorCount = 0;
    	startTime = HAL_GetTick();
		for (int i = 0; i < NUM_SENSOR; i++) {
			uint8_t q = i / 12;
			uint8_t r = i % 12;
			uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
			uint8_t channel = (r >= 8) ? r - 8 : r;
			ResetDevicesExcept(active_device);
			setActiveTcaChannel(active_device, channel);
			Dev = &vl53l0x_s[i];
			VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500us
			if (RangingData.RangeStatus == 0) {
			  if (RangingData.RangeMilliMeter < 80) {
				  float filteredValue = Kalman_Estimate(&filters[i], RangingData.RangeMilliMeter);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.1f ", filteredValue), 500);
				  sensorValues[i][readingCount[i]] = filteredValue;
				  readingCount[i]++;
				  sensorCount++;
			  }
			}else{
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d ", RangingData.RangeStatus), 500);
				  // 1 : Sigma Fail | 2 : Signal Fail | 3 : Min Range Fail | 4 : Phase Fail | 5 : Hardware Fail | 255 : No update
			  }
		}
		endTime = HAL_GetTick();
		diffTime = endTime - startTime;
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%lu ms ", diffTime), 100);

		Hx711Data = Read_HX711();
		UART_SendWeight_g(Hx711Data,-1/1600.00f,10002); // Send the weight data over UART
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, " %d", sensorCount), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);

    }
    TransmitStats();
    ResetSensorData();

}

void AutoCommand(){


    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "Auto Command \r\n"), 100);


	ResetAllDevices();
    for(int count =0; count < 100; count++){
		for (int i = 0; i < NUM_SENSOR; i++) {
			uint8_t q = i / 12;
			uint8_t r = i % 12;
			uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
			uint8_t channel = (r >= 8) ? r - 8 : r;
			ResetDevicesExcept(active_device);
			setActiveTcaChannel(active_device, channel);
			Dev = &vl53l0x_s[i];
			VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500us
			if (RangingData.RangeStatus == 0) {
			  if (RangingData.RangeMilliMeter < 80) {
				  float filteredValue = Kalman_Estimate(&filters[i], RangingData.RangeMilliMeter);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.1f ", filteredValue), 500);
			  }
			}else{
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d ", RangingData.RangeStatus), 500);
				  // 1 : Sigma Fail | 2 : Signal Fail | 3 : Min Range Fail | 4 : Phase Fail | 5 : Hardware Fail | 255 : No update
			  }

		}
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);


    }




	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "autoMode\r\n"), 100);
	ResetAllDevices();

	float forceSensorZeroPoint = 0.0f;

	servo_angle(&htim2, TIM_CHANNEL_1, 1); // poking
  	 for(int lin = 11; lin < 22; lin ++){
		 for(int rev = 0; rev < 18; rev++){
			 for(int r = 0; r < 8; r++){
				 servo_angle(&htim2, TIM_CHANNEL_1, r+7); // poking
				 HAL_Delay(500);
				 for(int count = 0; count < 40; count++){
					  for (int i = 0; i < NUM_SENSOR; i++) {
						uint8_t q = i / 12;
						uint8_t r = i % 12;
						uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
						uint8_t channel = (r >= 8) ? r - 8 : r;
						ResetDevicesExcept(active_device);
				        setActiveTcaChannel(active_device,channel);
						Dev = &vl53l0x_s[i];
						VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500us
						if (RangingData.RangeStatus == 0) {
						  if (RangingData.RangeMilliMeter < 80) {
							  float filteredValue = Kalman_Estimate(&filters[i], RangingData.RangeMilliMeter);
							  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", filteredValue), 500);
						  }
						}else{
							  //HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d ", RangingData.RangeStatus), 500);
							  // 1 : Sigma Fail | 2 : Signal Fail | 3 : Min Range Fail | 4 : Phase Fail | 5 : Hardware Fail | 255 : No update
						  }
					  }

				if (r == 0) {
					forceSensorZeroPoint = Read_HX711();
					Hx711Data = 0;
				} else {
					Hx711Data = Read_HX711() - forceSensorZeroPoint;
				}

				  UART_SendWeight_g(Hx711Data,-1/1600.00f,0); // Send the weight data over UART
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, " %d %d %.2f\n", 8*lin+2, 20*rev, r*0.8), 500);
				 }
			 HAL_Delay(500);
			 servo_angle(&htim2, TIM_CHANNEL_1, 0); // turn to origin
			 HAL_Delay(500);
			 }
			 stepRev(20); // revolution
		 }
		 HAL_Delay(500);
		 stepRev(-360);
		 stepLin(-8); // moving horizontal
  	 }
}

void InitializaionCalibrationCommand()
{
	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "InitializationCommand\n\r"), 100);

	ResetAllDevices();
  	for (int i = 0; i < NUM_SENSOR; i++) {
  	    uint8_t q = i / 12;
  	    uint8_t r = i % 12;
  	    uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
  	    uint8_t channel = (r >= 8) ? r - 8 : r;

        ResetDevicesExcept(active_device);
        setActiveTcaChannel(active_device,channel);

  		Dev = &vl53l0x_s[i];
  		Dev->I2cHandle = &hi2c1;
  		Dev->I2cDevAddr = VL53L0X_ADDR;

  		VL53L0X_WaitDeviceBooted( Dev );
  		VL53L0X_DataInit( Dev );
  		VL53L0X_StaticInit( Dev );
  		VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

  		VL53L0X_PerformRefSpadManagement( Dev, &refSpadCount[i], &isApertureSpads[i]);
  		VL53L0X_PerformRefCalibration( Dev, &VhvSettings[i], &PhaseCal[i]);

  		refSpadCountHost[i] = refSpadCount[i];
  		isApertureSpadsHost[i] = isApertureSpads[i];
  		VhvSettingsHost[i] = VhvSettings[i];
  		PhaseCalHost[i] = PhaseCal[i];

  		VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
  		VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  		VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
  		VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
  		VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, 33000);
  		VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  		VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

		Kalman_Init(&filters[i], Q, R, P, 0);  // Q, R, P, 초기값

 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d ",i), 100);
 		if(i%12 ==11){HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n\r"), 100);}
  	}


	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "uint32_t refSpadCountHost[NUM_SENSOR]={"), 100);
	for (int i = 0; i < NUM_SENSOR; i++){
		if(i < NUM_SENSOR - 1){
			HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%lu,",refSpadCount[i]), 100);
		} else {
			HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%lu};\n",refSpadCount[i]), 100);
		}
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "uint8_t isApertureSpadsHost[NUM_SENSOR]={"), 100);
	for (int i = 0; i < NUM_SENSOR; i++){
		if(i < NUM_SENSOR - 1){
			HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d,",isApertureSpads[i]), 100);
		} else {
			HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d};\n",isApertureSpads[i]), 100);
		}
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "uint8_t VhvSettingsHost[NUM_SENSOR]={"), 100);
	for (int i = 0; i < NUM_SENSOR; i++){
		if(i < NUM_SENSOR - 1){
			HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d,",VhvSettings[i]), 100);
		} else {
			HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d};\n",VhvSettings[i]), 100);
		}
	}

	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "uint8_t PhaseCalHost[NUM_SENSOR]={"), 100);
	for (int i = 0; i < NUM_SENSOR; i++){
		if(i < NUM_SENSOR - 1){
			HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d,",PhaseCal[i]), 100);
		} else {
			HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d};\n",PhaseCal[i]), 100);
		}
	}


}

void CalibrationCommand() {
}
void AvgStdCommand() {
}
void SetSensorCommand(){
	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "SetCommand\n\r"), 100);

	ResetAllDevices();
  	for (int i = 0; i < NUM_SENSOR; i++) {
  	    uint8_t q = i / 12;
  	    uint8_t r = i % 12;
  	    uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
  	    uint8_t channel = (r >= 8) ? r - 8 : r;

        ResetDevicesExcept(active_device);
        setActiveTcaChannel(active_device,channel);

  		Dev = &vl53l0x_s[i];
  		Dev->I2cHandle = &hi2c1;
  		Dev->I2cDevAddr = VL53L0X_ADDR;

  		VL53L0X_WaitDeviceBooted( Dev );
  		VL53L0X_DataInit( Dev );
  		VL53L0X_StaticInit( Dev );
  		VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

		VL53L0X_SetReferenceSpads(Dev, refSpadCountHost[i], isApertureSpadsHost[i]);
  		VL53L0X_SetRefCalibration(Dev, VhvSettingsHost[i], PhaseCalHost[i]);


  		VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
  		VL53L0X_SetLimitCheckEnable(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  		VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
  		VL53L0X_SetLimitCheckValue(Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
  		VL53L0X_SetMeasurementTimingBudgetMicroSeconds(Dev, 33000);
  		VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  		VL53L0X_SetVcselPulsePeriod(Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

		Kalman_Init(&filters[i], Q, R, P, 0);  // Q, R, P, 초기값

 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d:(",i), 100);
 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02lu ",refSpadCountHost[i]), 100);
 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d ",isApertureSpadsHost[i]), 100);
 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d ",VhvSettingsHost[i]), 100);
 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d) ",PhaseCalHost[i]), 100);
 		if(i%12 ==11){HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n\r"), 100);}

  	}
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
  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "JH TACTS test\n\r"), 100);
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
