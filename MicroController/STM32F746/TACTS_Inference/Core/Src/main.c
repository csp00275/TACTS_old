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



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Global handle to reference the instantiated C-model */
static ai_handle twoLine = AI_HANDLE_NULL;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(32)
static ai_u8 activations[AI_TWOLINE_DATA_ACTIVATIONS_SIZE];

/* Array to store the data of the input tensor */
AI_ALIGNED(32)
static ai_float in_data[AI_TWOLINE_IN_1_SIZE];

/* c-array to store the data of the output tensor */
AI_ALIGNED(32)
static ai_float out_data[AI_TWOLINE_OUT_1_SIZE];

/* Array of pointer to manage the model's input/output tensors */
static ai_buffer *ai_input;
static ai_buffer *ai_output;


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
int aiInit(void);
int aiRun(const ai_float *in_data, ai_float *out_data);
void ProcessCommand(uint8_t *command);
void FirstCommand();
void SecondCommand();
void ThirdCommand();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int aiInit(void) {
  ai_error err;

  /* Create and initialize the c-model */
  const ai_handle acts[] = { activations };
  err = ai_twoline_create_and_init(&twoLine, acts, NULL);
  if (err.type != AI_ERROR_NONE) {
      HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "aiInit Error \n"), 100);
  };

  /* Reteive pointers to the model's input/output tensors */
  ai_input = ai_twoline_inputs_get(twoLine, NULL);
  ai_output = ai_twoline_outputs_get(twoLine, NULL);

  return 0;
}

int aiRun(const ai_float *in_data, ai_float *out_data) {
  ai_i32 n_batch;

  /* 1 - Update IO handlers with the data payload */
  ai_input[0].data = AI_HANDLE_PTR(in_data);
  ai_output[0].data = AI_HANDLE_PTR(out_data);

  /* 2 - Perform the inference */
  n_batch = ai_twoline_run(twoLine, &ai_input[0], &ai_output[0]);
  if (n_batch != 1) {
      HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "aiRun Error \n"), 100);
  };

  return 0;
}


void ProcessCommand(uint8_t *command)
{
    if (strcmp((char*)command, "echo") == 0) {
        HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "echo\n"), 100);
    } else if (strcmp((char*)command, "1") == 0) {
    	FirstCommand();
    } else if (strcmp((char*)command, "2") == 0) {
    	SecondCommand();
    }else if (strcmp((char*)command, "3") == 0) {
		ThirdCommand();
    } else {
        HAL_UART_Transmit(&huart1, txMsg, sprintf((char*)txMsg, "Please insert correct command\n"), 100);
    }
}


void FirstCommand()
{
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "sensor test\r\n"), 100);

    uint32_t timeStart_s, timeEnd_s,timeDiff_s; // single
    uint32_t timeStart_a, timeEnd_a, timeDiff_a; // all

    timeStart_a = HAL_GetTick(); //

    do {
    	timeStart_s = HAL_GetTick();

  	  /// Read the VL53l0x data ///
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

void SecondCommand()
{
    HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "sensor test\r\n"), 100);

    uint32_t timeStart_s, timeEnd_s,timeDiff_s; // single
    uint32_t timeStart_a, timeEnd_a, timeDiff_a; // all

    timeStart_a = HAL_GetTick(); //

    do {
    	timeStart_s = HAL_GetTick();

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

            if (RangingData.RangeStatus == 0) {
                float filteredValue = Kalman_Estimate(&filters[i], RangingData.RangeMilliMeter);
                in_data[i]=filteredValue;
            }else{
                in_data[i]=45.5;
            }
            HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", in_data[i]), 1000);


        }

		timeEnd_s = HAL_GetTick();
		timeDiff_s = timeEnd_s - timeStart_s;
		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%lu ms ", timeDiff_s), 1000);

		aiRun(in_data,out_data);
		for(int k=0; k<4;k++){
	        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.16f ", out_data[k]), 1000);
		}

		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);

		timeEnd_a = HAL_GetTick();
		timeDiff_a = timeEnd_a - timeStart_a;

    } while (timeDiff_a < 10000);

}

void ThirdCommand(){

	ai_float test_input_data[24] = {
	    -0.512,  0.283, -0.671,  0.346,
	    -1.124,  0.445,  0.782, -0.455,
	     0.389, -0.317, -0.809,  1.125,
	     0.540, -0.916, -0.342,  0.471,
	    -1.206,  0.289,  0.673, -0.942,
	     0.858, -0.638,  0.234, -1.015
	};

    ai_float expected_output_data[4] = { 1,2,3,4 };
    ai_float test_output_data[4];

    aiRun(test_input_data, test_output_data);


    for(int i = 0; i < 4; i++) {
        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "Expected: %.1f, Got: %.5f\n", expected_output_data[i], test_output_data[i]), 1000);

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



  initializeAllSensors(tca_addr, vl53l0x_s, filters);

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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */


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
