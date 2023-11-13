/*
 * vl53l0x_jh.h
 *
 *  Created on: Oct 24, 2023
 *      Author: JH_LAB
 *      TACTS_Inference
 */

#ifndef INC_VL53L0X_JH_H_
#define INC_VL53L0X_JH_H_

#define VL53L0X_ADDR	0x29 << 1 // Default I2C address of VL53L0X
#define NUM_SENSOR 24

#include "vl53l0x_api.h"
#include "kalman.h"
#include "i2c.h"

extern uint32_t refSpadCount[NUM_SENSOR];
extern uint8_t isApertureSpads[NUM_SENSOR];
extern uint8_t VhvSettings[NUM_SENSOR];
extern uint8_t PhaseCal[NUM_SENSOR];

extern uint32_t refSpadCountHost[NUM_SENSOR];
extern uint8_t isApertureSpadsHost[NUM_SENSOR];
extern uint8_t VhvSettingsHost[NUM_SENSOR];
extern uint8_t PhaseCalHost[NUM_SENSOR];

extern VL53L0X_RangingMeasurementData_t RangingData;

extern VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR];
extern VL53L0X_DEV Dev;

extern uint8_t tca_ch[8]; // control register of TCA9548A
extern uint8_t tca_ch_reset;

#if NUM_SENSOR <=12
	extern uint8_t tca_addr[2];
#elif NUM_SENSOR <=24
	extern uint8_t tca_addr[4];
#elif NUM_SENSOR <=36
	extern uint8_t tca_addr[6];
#elif NUM_SENSOR <=48
	extern uint8_t tca_addr[8];
#endif

extern float Xmean[NUM_SENSOR];
extern float Xstd[NUM_SENSOR];
extern float Fminmax[2];
extern float Zminmax[2];

#if 0
void resetTcaDevicesExcept(uint8_t active_device, const uint8_t *tca_addr);
void setActiveTcaChannel(uint8_t active_device, uint8_t channel, const uint8_t *tca_addr);
void initializeVl53l0x(VL53L0X_Dev_t *Dev);
void initializeAllSensors(uint8_t *tca_addr, VL53L0X_Dev_t *vl53l0x_s, KalmanFilter *filters);
void excuteVl53l0x(VL53L0X_Dev_t *Dev, int i);
#endif

void ResetAllDevices();
void ResetDevicesExcept(uint8_t active_device);
void setActiveTcaChannel(uint8_t active_device, uint8_t channel);


#endif /* INC_VL53L0X_JH_H_ */
