/*
 * vl53l0x_jh.c
 *
 *  Created on: 2023. 11. 4.
 *      Author: 21310
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
	#define tcaLength 2
#elif NUM_SENSOR <= 24
	uint8_t tca_addr[4] = {0x70, 0x71, 0x72, 0x73};
	#define tcaLength 4
#elif NUM_SENSOR <= 36
	uint8_t tca_addr[6] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75};
	#define tcaLength 6
#elif NUM_SENSOR <= 48
	uint8_t tca_addr[8] = {0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77};
	#define tcaLength 8
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
    for (int j = 0; j <tcaLength; ++j) {
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
