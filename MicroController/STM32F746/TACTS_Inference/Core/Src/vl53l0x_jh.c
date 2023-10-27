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


VL53L0X_RangingMeasurementData_t RangingData;

// VL53L0X initialization stuff
uint32_t refSpadCount = 0;
uint8_t isApertureSpads = 0;
uint8_t VhvSettings = 0;
uint8_t PhaseCal = 0;

VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR];
VL53L0X_DEV Dev;

KalmanFilter filters[];
float Q = 0.001f; // Process noise covariance
float R = 0.03f;   // Measurement noise covariance
float P = 0.001f;



uint8_t tca_ch[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // control register of TCA9548A
uint8_t tca_ch_reset = 0x00;


#if NUM_SENSOR <= 12
	uint8_t tca_addr[2] = {0x70,0x71};
#elif NUM_SENSOR <= 24
	uint8_t tca_addr[4] = {0x70, 0x71, 0x72, 0x73};
	float Xmean[24] = {47.7893, 46.5874, 55.0583, 43.5072, 51.6538, 64.7541, 60.8311, 53.016, 62.2867, 56.4524, 52.963, 50.5628, 48.354, 49.8986, 50.3405, 46.0296, 51.1459, 55.8726, 47.4925, 54.4715, 50.517, 55.3706, 55.7818, 49.805};
	float Xstd[24] = {1.43535, 1.70831, 1.49912, 1.78615, 1.19965, 1.69715, 1.96623, 1.89145, 1.80004, 1.44443, 1.60045, 1.35581, 1.44356, 1.66595, 1.45649, 1.56476, 1.5824, 2.09196, 2.18042, 3.00113, 2.48663, 1.72633, 1.75801, 1.5154};
	float Fminmax[2]={ 4.0, 134.5 };
	float Zminmax[2]={ 0, 160 };
#elif NUM_SENSOR <= 36
	uint8_t tca_addr[6] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75};
	float Xmean[36] = {68.794, 66.6038, 66.2412, 66.5997, 60.2285, 58.5962, 62.3442, 67.6147, 74.8669, 61.3529, 67.531, 65.2138, 66.4032, 56.2626, 61.7455, 59.7483, 48.089, 46.8554, 57.0667, 53.7475, 60.0471, 62.6412, 68.5552, 62.1381, 67.8747, 68.3507, 66.3984, 65.1063, 63.296, 53.9298, 58.1348, 57.3841, 60.6508, 62.605, 68.7484, 68.3618};
	float Xstd[36] = {1.46743, 1.27941, 1.93325, 1.12181, 1.40079, 1.12315, 1.37114, 1.80106, 1.33741, 1.23588, 1.3504, 1.40722, 1.94493, 2.22899, 2.53636, 2.04718, 2.32633, 2.6425, 1.83862, 2.06066, 1.87703, 1.99821, 1.79039, 1.99543, 1.47107, 1.32125, 1.36736, 1.29468, 1.30637, 1.19649, 1.22083, 1.47852, 1.54042, 1.33985, 1.45481, 1.64888};
	float Fminmax[2]={ 5.58, 453.03 };
	float Zminmax[2]={ 0, 160 };
#elif NUM_SENSOR <= 48
	uint8_t tca_addr[8] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77};
	float Xmean[48] = {65.4289, 62.3989, 51.143, 55.313, 58.3824, 50.6548, 58.7846, 52.5792, 55.3687, 59.0666, 61.7927, 53.4225, 57.5199, 57.8597, 54.5496, 56.238, 44.4611, 52.6367, 51.1833, 53.3153, 52.9111, 55.2247, 48.5697, 53.1319, 47.1309, 57.0615, 60.0953, 47.7565, 53.8865, 54.3693, 61.8036, 58.5812, 55.37, 50.2959, 53.6366, 50.7526, 60.1635, 57.6653, 56.7975, 56.0926, 55.2098, 61.2049, 61.566, 55.1256, 56.4219, 57.8819, 54.3885, 59.2844};
	float Xstd[48] = {1.84552, 1.48231, 0.88235, 1.14766, 1.35136, 1.37877, 1.31405, 1.34979, 2.09637, 1.19606, 1.9971, 1.40785, 1.67254, 1.81214, 1.58725, 1.43326, 1.35542, 1.59694, 1.79312, 2.55519, 2.56928, 2.17124, 2.09336, 1.68093, 1.59378, 1.65623, 1.25722, 1.37048, 1.52434, 1.61955, 1.8085, 2.19229, 2.55473, 2.08254, 1.93683, 1.90604, 1.50199, 1.33379, 1.32529, 1.49702, 1.52882, 1.18115, 1.47583, 2.32719, 1.2934, 1.69155, 1.4315, 1.53544};
	float Fminmax[2]={ 15.92, 167.45 };
	float Zminmax[2]={ 0, 160 };
#endif

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

