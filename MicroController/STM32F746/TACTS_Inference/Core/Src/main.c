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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "vl53l0x_jh.h"
#include "kalman.h"
#include "i2c.h"
#include "usart.h"
#include "twoLine.h"
#include "twoLine_data.h"
#include "ai.h"
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

CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

static void MX_CRC_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void ProcessCommand(uint8_t *command);
void SensorCommand();
void InitializaionCalibrationCommand();
void CalibrationCommand();
void SetSensorCommand();
void AvgStdCommand();
void InferenceCommand();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void ProcessCommand(uint8_t *command)
{
    if (strcmp((char*)command, "echo") == 0) {HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "echo\n"), 100);}
    else if (strcmp((char*)command, "sensor") == 0) {SensorCommand();}
    else if (strcmp((char*)command, "setsensor") == 0) {SetSensorCommand();}
    else if (strcmp((char*)command, "cali") == 0) {CalibrationCommand();}
    else if (strcmp((char*)command, "avg") == 0) {AvgStdCommand();}
    else if (strcmp((char*)command, "infer") == 0) {InferenceCommand();}
    else {HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "Please insert correct command\n"), 100);}
}

void SensorCommand()
{

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
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);

    }
    TransmitStats();
    ResetSensorData();
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

		Kalman_Init(&filters[i], Q, R, P, 0);  // Q, R, P, 초기�??

 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d:(",i), 100);
 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02lu ",refSpadCountHost[i]), 100);
 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d ",isApertureSpadsHost[i]), 100);
 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d ",VhvSettingsHost[i]), 100);
 		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%02d) ",PhaseCalHost[i]), 100);
 		if(i%12 ==11){HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n\r"), 100);}

  	}
}

void AvgStdCommand(){
	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "Avg Std Force Z\n" ), 1000);
    for(int i = 0; i < sizeof(Xmean)/sizeof(Xmean[0]); i++) {
        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", Xmean[i]), 1000);
        if(i%8==7){HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 1000);}
    }
    for(int i = 0; i < sizeof(Xstd)/sizeof(Xstd[0]); i++) {
        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", Xstd[i]), 1000);
        if(i%8==7){HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 1000);}
    }
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", Fminmax[0]), 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", Fminmax[1]), 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", Zminmax[0]), 1000);
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", Zminmax[1]), 1000);
	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 1000);
}

void CalibrationCommand(){

}


void InferenceCommand()
{
	ResetAllDevices();
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "sensor test\r\n"), 100);

    for (int count =0; count <100;count ++){
    	uint8_t tofCount =0;
        for (int i = 0; i < NUM_SENSOR; i++) {
    	    uint8_t q = i / 12;
    	    uint8_t r = i % 12;
    	    uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
    	    uint8_t channel = (r >= 8) ? r - 8 : r;
    	    ResetDevicesExcept(active_device);
            setActiveTcaChannel(active_device, channel);
            Dev = &vl53l0x_s[i];
            VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500 us

            if (RangingData.RangeStatus == 0) {
                float filteredValue = Kalman_Estimate(&filters[i], RangingData.RangeMilliMeter); // 500 us
                in_data[i]=filteredValue;
                HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", in_data[i]), 1000);
                in_data[i]= (filteredValue-Xmean[i])/Xstd[i];
                tofCount++;
            }else{
               HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "99 "), 1000);
            }
        }


		if(tofCount == NUM_SENSOR){
		aiRun(in_data,out_data);
		out_data[0] = (out_data[0] + 1) * (Fminmax[1] - Fminmax[0]) / 2 + Fminmax[0];
		out_data[1] = (out_data[1] + 1) * (Zminmax[1] - Zminmax[0]) / 2 + Zminmax[0];
		for(int k=0; k<4;k++){
	        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", out_data[k]), 1000);
		}
		float sqSum= out_data[3]*out_data[3] + out_data[4]*out_data[4];
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", sqSum), 1000);
		}
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_CRC_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  aiInit();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}



/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

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
	  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "Error\n\r"), 100);
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
