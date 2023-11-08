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
#include <math.h> // 표준편차 계산을 위해 필요


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define VL53L0X_ADDR 0x29 << 1
#define NUM_SENSOR 36
#define preset
#define NUM_READINGS 200


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

VL53L0X_RangingMeasurementData_t RangingData;

float sensorValues[NUM_SENSOR][NUM_READINGS] = {0};
float sensorAverages[NUM_SENSOR] = {0};
float sensorStdDevs[NUM_SENSOR] = {0};
int readingCount[NUM_SENSOR] = {0};


/* USER CODE BEGIN Init */



#if NUM_SENSOR == 24
#elif NUM_SENSOR == 36
uint32_t refSpadCountPre[36] = { 4, 5, 3, 5, 5, 5, 4, 4, 8, 5, 5, 6, 4, 5, 4, 4, 4, 5, 4, 5, 4, 3, 4, 6, 5, 4, 4, 5, 5, 5, 5, 5, 5, 5, 4, 5 };
uint8_t isApertureSpadsPre[36] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t VhvSettingsPre[36] = { 25, 27, 29, 24, 30, 30, 28, 34, 29, 29, 25, 27, 32, 31, 31, 29, 30, 26, 28, 32, 33, 31, 29, 28, 33, 28, 31, 32, 32, 32, 33, 31, 29, 31, 29, 34 };
uint8_t PhaseCalPre[36] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };
#elif NUM_SENSOR == 48
#endif

uint32_t refSpadCount[NUM_SENSOR] = {0};
uint8_t isApertureSpads[NUM_SENSOR] = {0};
uint8_t VhvSettings[NUM_SENSOR] = {0};
uint8_t PhaseCal[NUM_SENSOR] = {0};

VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR];
VL53L0X_DEV Dev;

uint8_t tca_ch[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // control register of TCA9548A
uint8_t tca_ch_reset = 0x00;

#if NUM_SENSOR == 24
  uint8_t tca_addr[4] = {0x70,0x71,0x72,0x73}; // 2 line
#elif NUM_SENSOR == 36
  uint8_t tca_addr[6] = {0x70,0x71,0x72,0x73,0x74,0x75}; // 3 line
#elif NUM_SENSOR == 48
  uint8_t tca_addr[8] = {0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77};  // 4 line
#endif

KalmanFilter filters[NUM_SENSOR];
float Q = 0.001f; // Process noise covariance
float R = 0.03f;   // Measurement noise covariance
float P = 0.001f;

float rawData = 0;
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
void InitializaionCalibrationCommand();
void CalibrationCommand();
void PresetCommand();
void AvgStdCommand();
void CalculateStats(int sensorIndex);
void TransmitStats();
void ResetSensorData();

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
    }else if (strcmp((char*)command, "cali") == 0) {
    	CalibrationCommand();
    }else if (strcmp((char*)command, "ini") == 0) {
    	InitializaionCalibrationCommand();
    }else if (strcmp((char*)command, "preset") == 0) {
    	PresetCommand();
    }else if (strcmp((char*)command, "2") == 0) {
    	CalibrationCommand();
    }else if (strcmp((char*)command, "3") == 0) {
    	CalibrationCommand();
    }else if (strcmp((char*)command, "4") == 0) {
    	CalibrationCommand();
    }else{
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
    for(int count =0; count < NUM_READINGS; count++){
  	  start_section_time = HAL_GetTick();

  	  /// Read the VL53l0x data ///
  	 for (int i = 0; i < NUM_SENSOR; i++) {
		uint8_t q = i / 12;
		uint8_t r = i % 12;
		uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
		uint8_t channel = (r >= 8) ? r - 8 : r;

		//Reset device except active
		for (int j = 0; j < sizeof(tca_addr); ++j) {
			   if (j != active_device) {
				   HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
			   }
		   }

		// set channel of active device
		HAL_I2C_Master_Transmit(&hi2c1, tca_addr[active_device] << 1, &tca_ch[channel], 1, 1000);
		  Dev = &vl53l0x_s[i];
		  VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500us

		  if (RangingData.RangeStatus == 0) {
			  if (RangingData.RangeMilliMeter < 80) {
				  float filteredValue = Kalman_Estimate(&filters[i], RangingData.RangeMilliMeter);
				  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.1f ", filteredValue), 500);
                  sensorValues[i][readingCount[i]] = filteredValue; // 값을 저장
                  readingCount[i]++;
			  }
		  }

	  }

	  end_section_time = HAL_GetTick();
	  elapsed_section_time = end_section_time - start_section_time;
	  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%lu ms ", elapsed_section_time), 100);


	  /// End of Reading and Filtering Vl53l0x data ///

	  /// Read the raw data from HX711 ///
	  rawData = Read_HX711();
	  float loadcell_slope = -1/1600.00f; // Convert the raw data to weight (replace the calibration factor with your own)
	  float loadcell_bias = 10002;
	  UART_SendWeight_g(rawData,loadcell_slope,loadcell_bias); // Send the weight data over UART
	  /// End of Reading HX711 data ///

	  end_time = HAL_GetTick(); // 종료 ?���??? 측정
	  time_diff = end_time - start_time; // ?���??? 차이 계산

	  HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "\n"), 100);
    }
    TransmitStats();
    ResetSensorData();

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

void InitializaionCalibrationCommand()
{

    for (int j = 0; j < sizeof(tca_addr); ++j) {
        HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
    }

  	for (int i = 0; i < NUM_SENSOR; i++) {

  	    uint8_t q = i / 12;
  	    uint8_t r = i % 12;
  	    uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
  	    uint8_t channel = (r >= 8) ? r - 8 : r;

  	    //Reset device except active
  	    for (int j = 0; j < sizeof(tca_addr); ++j) {
  	           if (j != active_device) {
  	               HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
  	           }
  	       }

  	    // set channel of active device
  	    HAL_I2C_Master_Transmit(&hi2c1, tca_addr[active_device] << 1, &tca_ch[channel], 1, 1000);


  		Dev = &vl53l0x_s[i];
  		Dev->I2cHandle = &hi2c1;
  		Dev->I2cDevAddr = VL53L0X_ADDR;

  		VL53L0X_WaitDeviceBooted( Dev );
  		VL53L0X_DataInit( Dev );
  		VL53L0X_StaticInit( Dev );
  		VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  		VL53L0X_PerformRefCalibration( Dev, &VhvSettings[i], &PhaseCal[i]);
  		VL53L0X_PerformRefSpadManagement( Dev, &refSpadCount[i], &isApertureSpads[i]);
  		VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
  		VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  		VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
  		VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
  		VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, 33000);
  		VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  		VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);


  		// KalmanFilter initializer BEGIN //
          Kalman_Init(&filters[i], Q, R, P, 0);  // Q, R, P, 초기값
  		// KalmanFilter initializer END //
  		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d complete \n\r",i), 100);
  	}

}

void CalibrationCommand() {
    // refSpadCount 배열 출력
    char buffer[1024]; // 충분한 크기의 버퍼를 확보합니다. 필요에 따라 크기 조정이 필요할 수 있습니다.
    uint8_t messageLength = 0;
    uint8_t NUM = NUM_SENSOR;

    messageLength = sprintf(buffer, "uint32_t refSpadCount[%d] = { ",NUM);
    for(int i = 0; i < NUM_SENSOR; i++) {
        messageLength += sprintf(buffer + messageLength, "%lu%s", refSpadCount[i], (i < NUM_SENSOR - 1) ? ", " : " ");
    }
    messageLength += sprintf(buffer + messageLength, "};\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, messageLength, 100);

    // isApertureSpads 배열 출력
    messageLength = sprintf(buffer, "uint8_t isApertureSpads[%d] = { ",NUM);
    for(int i = 0; i < NUM_SENSOR; i++) {
        messageLength += sprintf(buffer + messageLength, "%d%s", isApertureSpads[i], (i < NUM_SENSOR - 1) ? ", " : " ");
    }
    messageLength += sprintf(buffer + messageLength, "};\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, messageLength, 100);

    // VhvSettings 배열 출력
    messageLength = sprintf(buffer, "uint8_t VhvSettings[%d] = { ",NUM);
    for(int i = 0; i < NUM_SENSOR; i++) {
        messageLength += sprintf(buffer + messageLength, "%d%s", VhvSettings[i], (i < NUM_SENSOR - 1) ? ", " : " ");
    }
    messageLength += sprintf(buffer + messageLength, "};\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, messageLength, 100);

    // PhaseCal 배열 출력
    messageLength = sprintf(buffer, "uint8_t PhaseCal[%d] = { ",NUM);
    for(int i = 0; i < NUM_SENSOR; i++) {
        messageLength += sprintf(buffer + messageLength, "%d%s", PhaseCal[i], (i < NUM_SENSOR - 1) ? ", " : " ");
    }
    messageLength += sprintf(buffer + messageLength, "};\n");
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, messageLength, 100);
}
void AvgStdCommand() {
	HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "Using Preset\n\r"), 100);
}
void PresetCommand(){
    for (int j = 0; j < sizeof(tca_addr); ++j) {
        HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
    }

  	for (int i = 0; i < NUM_SENSOR; i++) {

  	    uint8_t q = i / 12;
  	    uint8_t r = i % 12;
  	    uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
  	    uint8_t channel = (r >= 8) ? r - 8 : r;

  	    //Reset device except active
  	    for (int j = 0; j < sizeof(tca_addr); ++j) {
  	           if (j != active_device) {
  	               HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
  	           }
  	       }

  	    // set channel of active device
  	    HAL_I2C_Master_Transmit(&hi2c1, tca_addr[active_device] << 1, &tca_ch[channel], 1, 1000);





  		Dev = &vl53l0x_s[i];
  		Dev->I2cHandle = &hi2c1;
  		Dev->I2cDevAddr = VL53L0X_ADDR;

  		VL53L0X_WaitDeviceBooted( Dev );
  		VL53L0X_DataInit( Dev );
  		VL53L0X_StaticInit( Dev );
  		VL53L0X_SetDeviceMode(Dev, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
  		VL53L0X_PerformRefCalibration( Dev, &VhvSettings[i], &PhaseCal[i]);
  		VL53L0X_PerformRefSpadManagement( Dev, &refSpadCount[i], &isApertureSpads[i]);
  		VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
  		VL53L0X_SetLimitCheckEnable( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
  		VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
  		VL53L0X_SetLimitCheckValue( Dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
  		VL53L0X_SetMeasurementTimingBudgetMicroSeconds( Dev, 33000);
  		VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
  		VL53L0X_SetVcselPulsePeriod( Dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

		refSpadCount[i] = refSpadCountPre[i];
		isApertureSpads[i] = isApertureSpadsPre[i];
		VhvSettings[i] =  VhvSettingsPre[i];
		PhaseCal[i] = PhaseCalPre[i];


  		// KalmanFilter initializer BEGIN //
          Kalman_Init(&filters[i], Q, R, P, 0);  // Q, R, P, 초기값
  		// KalmanFilter initializer END //
  		HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d complete \n\r",i), 100);
  	}
}


void CalculateStats(int sensorIndex) {
    float sum = 0;
    float mean = 0;
    float stdDevSum = 0;
    int count = readingCount[sensorIndex];

    // 평균 계산
    for (int i = 0; i < count; i++) {
        sum += sensorValues[sensorIndex][i];
    }
    mean = sum / count;
    sensorAverages[sensorIndex] = mean;

    // 표준편차 계산
    for (int i = 0; i < count; i++) {
        stdDevSum += pow(sensorValues[sensorIndex][i] - mean, 2);
    }
    sensorStdDevs[sensorIndex] = sqrt(stdDevSum / count);
}

// 평균과 표준편차를 UART로 전송하는 함수
void TransmitStats() {
    char msg[128];
    for (int i = 0; i < NUM_SENSOR; i++) {
        CalculateStats(i); // 통계 계산
        sprintf(msg, "Sensor %d - Avg: %.2f, StdDev: %.2f\r\n", i, sensorAverages[i], sensorStdDevs[i]);
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}

void ResetSensorData() {
    // 모든 센서 값과 읽기 횟수를 0으로 설정
    for (int i = 0; i < NUM_SENSOR; i++) {
        memset(sensorValues[i], 0, sizeof(sensorValues[i]));
        readingCount[i] = 0;
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
