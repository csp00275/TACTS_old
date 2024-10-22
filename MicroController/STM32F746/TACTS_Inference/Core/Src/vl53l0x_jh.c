/*
 * vl53l0x_jh.c
 *
 *  Created on: Oct 24, 2023
 *      Author: JH_LAB
 */


#include "vl53l0x_jh.h"
#include "vl53l0x_api.h"
#include "kalman.h"
#include "usart.h"

#define preset

VL53L0X_RangingMeasurementData_t RangingData;

uint32_t refSpadCount[NUM_SENSOR] = {0};
uint8_t isApertureSpads[NUM_SENSOR] = {0};
uint8_t VhvSettings[NUM_SENSOR] = {0};
uint8_t PhaseCal[NUM_SENSOR] = {0};

#ifdef preset
	#if  NUM_SENSOR == 24
	uint32_t refSpadCountHost[NUM_SENSOR]={6,4,11,6,5,4,4,5,4,6,4,3,5,4,4,5,6,4,5,4,4,4,5,6};
	uint8_t isApertureSpadsHost[NUM_SENSOR]={0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t VhvSettingsHost[NUM_SENSOR]={32,26,27,32,25,28,36,26,31,34,30,30,28,28,32,32,33,29,32,32,26,29,30,25};
	uint8_t PhaseCalHost[NUM_SENSOR]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	#elif NUM_SENSOR == 36
	uint32_t refSpadCountHost[NUM_SENSOR]={4,5,3,5,5,5,4,4,8,5,5,6,4,5,4,4,4,5,4,5,4,3,4,6,5,5,3,5,5,5,5,5,5,5,4,5};
	uint8_t isApertureSpadsHost[NUM_SENSOR]={0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0};
	uint8_t VhvSettingsHost[NUM_SENSOR]={26,28,29,25,30,31,29,35,30,30,26,28,33,32,32,30,31,27,28,33,34,32,30,29,34,28,32,33,33,33,34,32,30,32,30,35};
	uint8_t PhaseCalHost[NUM_SENSOR]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	#elif NUM_SENSOR == 48
	uint32_t refSpadCountHost[NUM_SENSOR]={10,7,6,8,8,7,9,8,7,8,7,5,4,4,4,5,5,4,6,4,5,4,5,5,5,5,5,4,4,4,5,4,4,4,4,4,4,4,3,4,11,4,4,5,5,6,5,4};
	uint8_t isApertureSpadsHost[NUM_SENSOR]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0};
	uint8_t VhvSettingsHost[NUM_SENSOR]={28,29,29,27,31,23,27,25,27,29,26,28,27,26,27,30,27,27,27,31,29,26,29,32,30,26,30,25,29,29,29,26,29,31,30,32,31,31,28,26,30,31,30,29,29,30,30,32};
	uint8_t PhaseCalHost[NUM_SENSOR]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
	#endif
#else
uint32_t refSpadCountHost[NUM_SENSOR] = {0};
uint8_t isApertureSpadsHost[NUM_SENSOR] = {0};
uint8_t VhvSettingsHost[NUM_SENSOR] = {0};
uint8_t PhaseCalHost[NUM_SENSOR] = {0};
#endif



VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR];
VL53L0X_DEV Dev;

uint8_t tca_ch[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // control register of TCA9548A
uint8_t tca_ch_reset = 0x00;


#if NUM_SENSOR <= 12
	uint8_t tca_addr[2] = {0x70,0x71};
#elif NUM_SENSOR <= 24
	uint8_t tca_addr[4] = {0x70, 0x71, 0x72, 0x73};
	uint8_t tcaLength = 4;
	float Xmean[24] = { 42.84, 43.38, 49.69, 44.07, 48.22, 60.49, 55.37, 46.97, 58.66, 54.8, 53.67, 49.57, 45.03, 47.63, 48.58, 43.53, 50.25, 48.23, 44.56, 52.52, 51.1, 52.06, 51.06, 42.76 };
	float Xstd[24] = { 1.71, 2.27, 1.63, 2.01, 1.88, 2.9, 2.39, 2.32, 2.05, 1.75, 2.14, 1.73, 1.91, 1.71, 1.54, 1.49, 1.46, 3.19, 2.64, 3.22, 2.65, 2.06, 1.96, 2.08 };
	float Fminmax[2] = { 0.0, 93.61 };
	float Zminmax[2] = { 2.0, 154.0 };
#elif NUM_SENSOR <= 36
	uint8_t tca_addr[6] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75};
	uint8_t tcaLength = 6;
	float Xmean[36] = { 65.4, 66.17, 63.54, 64.64, 58.0, 58.25, 60.0, 64.9, 71.54, 59.39, 66.76, 65.7, 62.38, 52.27, 59.78, 57.0, 55.41, 43.33, 52.79, 59.45, 56.31, 57.6, 64.57, 59.42, 68.92, 68.0, 67.87, 62.91, 59.91, 50.65, 54.16, 52.24, 58.34, 59.76, 66.52, 72.35 };
	float Xstd[36] = { 1.32, 1.46, 1.46, 1.19, 2.39, 1.46, 1.8, 2.35, 1.33, 1.19, 1.36, 1.4, 2.24, 2.49, 2.04, 2.07, 2.24, 2.2, 2.23, 2.14, 2.22, 2.06, 2.06, 2.39, 1.67, 1.44, 1.58, 1.71, 1.51, 1.51, 3.07, 1.6, 1.92, 1.94, 1.55, 1.69 };
	float Fminmax[2]={ 1.23, 397.59 };
	float Zminmax[2]={ 15, 159 };
#elif NUM_SENSOR <= 48
	uint8_t tca_addr[8] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77};
	uint8_t tcaLength = 8;
	float Xmean[48] = {65.4289, 62.3989, 51.143, 55.313, 58.3824, 50.6548, 58.7846, 52.5792, 55.3687, 59.0666, 61.7927, 53.4225, 57.5199, 57.8597, 54.5496, 56.238, 44.4611, 52.6367, 51.1833, 53.3153, 52.9111, 55.2247, 48.5697, 53.1319, 47.1309, 57.0615, 60.0953, 47.7565, 53.8865, 54.3693, 61.8036, 58.5812, 55.37, 50.2959, 53.6366, 50.7526, 60.1635, 57.6653, 56.7975, 56.0926, 55.2098, 61.2049, 61.566, 55.1256, 56.4219, 57.8819, 54.3885, 59.2844};
	float Xstd[48] = {1.84552, 1.48231, 0.88235, 1.14766, 1.35136, 1.37877, 1.31405, 1.34979, 2.09637, 1.19606, 1.9971, 1.40785, 1.67254, 1.81214, 1.58725, 1.43326, 1.35542, 1.59694, 1.79312, 2.55519, 2.56928, 2.17124, 2.09336, 1.68093, 1.59378, 1.65623, 1.25722, 1.37048, 1.52434, 1.61955, 1.8085, 2.19229, 2.55473, 2.08254, 1.93683, 1.90604, 1.50199, 1.33379, 1.32529, 1.49702, 1.52882, 1.18115, 1.47583, 2.32719, 1.2934, 1.69155, 1.4315, 1.53544};
	float Fminmax[2]={ 15.92, 167.45 };
	float Zminmax[2]={ 0, 160 };
#endif


#if 0
void resetTcaDevicesExcept(uint8_t active_device, const uint8_t *tca_addr) {
    for (int j = 0; j < sizeof(tca_addr); ++j) {
        if (j != active_device) {
            HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
        }
    }
}

void setActiveTcaChannel(uint8_t active_device, uint8_t channel, const uint8_t *tca_addr) {
    HAL_I2C_Master_Transmit(&hi2c1, tca_addr[active_device] << 1, &tca_ch[channel], 1, 1000);
}

void initializeVl53l0x(VL53L0X_Dev_t *Dev) {
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
}

void initializeAllSensors(uint8_t *tca_addr, VL53L0X_Dev_t *vl53l0x_s, KalmanFilter *filters) {
    for (int j = 0; j < sizeof(tca_addr)/sizeof(tca_addr[0]); ++j) {
        HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
    }

    for (int i = 0; i < NUM_SENSOR; i++) {
        uint8_t q = i / 12;
        uint8_t r = i % 12;
        uint8_t active_device = q * 2 + (r >= 8 ? 1 : 0);
        uint8_t channel = (r >= 8) ? r - 8 : r;

        resetTcaDevicesExcept(active_device, tca_addr);
        setActiveTcaChannel(active_device, channel, tca_addr);

        initializeVl53l0x(&vl53l0x_s[i]);
        Kalman_Init(&filters[i], Q, R, P, 0);  // Q, R, P, 초기값

        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%d complete \n\r", i), 100);
    }
}

void excuteVl53l0x(VL53L0X_Dev_t *Dev,int i) {
    VL53L0X_PerformContinuousRangingMeasurement(Dev, &RangingData); // 1500us
    if (RangingData.RangeStatus == 0) {
        float filteredValue = Kalman_Estimate(&filters[i], RangingData.RangeMilliMeter);
        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "%.2f ", filteredValue), 100);
    }else{
        HAL_UART_Transmit(&huart1, (uint8_t*)txMsg, sprintf((char*)txMsg, "999 "), 100);
    }
}
#endif


void ResetAllDevices() {
  for (int j = 0; j < tcaLength; ++j) {
	  HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
  }
}

void ResetDevicesExcept(uint8_t active_device) {
    for (int j = 0; j < tcaLength; ++j) {
        if (j != active_device) {
            HAL_I2C_Master_Transmit(&hi2c1, tca_addr[j] << 1, &tca_ch_reset, 1, 1000);
        }
    }
}

void setActiveTcaChannel(uint8_t active_device, uint8_t channel){
	HAL_I2C_Master_Transmit(&hi2c1, tca_addr[active_device] << 1, &tca_ch[channel], 1, 1000);
}

