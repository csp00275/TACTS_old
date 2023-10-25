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


#if NUM_SENSOR <=12
	uint8_t tca_addr[2] = {0x70,0x71};
#elif NUM_SENSOR <=24
	uint8_t tca_addr[4] = {0x70, 0x71, 0x72, 0x73};
	float Xmean[24] = { 47.79, 46.59, 55.06, 43.51, 51.65, 64.75, 60.83, 53.02, 62.29, 56.45, 52.96, 50.56, 48.35, 49.9, 50.34, 46.03, 51.15, 55.87, 47.49, 54.47, 50.52, 55.37, 55.78, 49.8 };
	float Xstd[24] = { 1.44, 1.71, 1.5, 1.79, 1.2, 1.7, 1.97, 1.89, 1.8, 1.44, 1.6, 1.36, 1.44, 1.67, 1.46, 1.56, 1.58, 2.09, 2.18, 3.0, 2.49, 1.73, 1.76, 1.52 };
	float Fminmax[2]={ 4.0, 134.5 };
	float Zminmax[2]={ 0, 160 };
#elif NUM_SENSOR <=36
	uint8_t tca_addr[6] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75};
#elif NUM_SENSOR <=48
	uint8_t tca_addr[8] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77};
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
    for (int j = 0; j < sizeof(tca_addr); ++j) {
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

