/*
 * vl53l0x_jh.h
 *
 *  Created on: Oct 24, 2023
 *      Author: JH_LAB
 */

#ifndef INC_VL53L0X_JH_H_
#define INC_VL53L0X_JH_H_

#define NUM_SENSOR 12
#define VL53L0X_ADDR	0x29 << 1 // Default I2C address of VL53L0X

#include "vl53l0x_api.h"
#include "kalman.h"
#include "i2c.h"

extern VL53L0X_RangingMeasurementData_t RangingData;

extern VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR];
extern VL53L0X_DEV Dev;
extern KalmanFilter filters[NUM_SENSOR];


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


void resetTcaDevicesExcept(uint8_t active_device, const uint8_t *tca_addr);
void setActiveTcaChannel(uint8_t active_device, uint8_t channel, const uint8_t *tca_addr);
void initializeVl53l0x(VL53L0X_Dev_t *Dev);
void initializeAllSensors(uint8_t *tca_addr, VL53L0X_Dev_t *vl53l0x_s, KalmanFilter *filters);
void excuteVl53l0x(VL53L0X_Dev_t *Dev, int i);


#endif /* INC_VL53L0X_JH_H_ */
