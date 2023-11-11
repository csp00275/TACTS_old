/*
 * vl53l0x_jh.h
 *
 *  Created on: 2023. 11. 4.
 *      Author: 21310
 */

#ifndef INC_VL53L0X_JH_H_
#define INC_VL53L0X_JH_H_

#include "vl53l0x_api.h"

#define VL53L0X_ADDR 0x29 << 1
#define NUM_SENSOR 24

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

extern uint8_t tca_ch[8];
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

void ResetAllDevices();
void ResetDevicesExcept(uint8_t active_device);
void setActiveTcaChannel(uint8_t active_device, uint8_t channel);

#endif /* INC_VL53L0X_JH_H_ */
