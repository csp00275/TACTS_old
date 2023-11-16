/*
 * vl53l0x_jh.h
 *
 *  Created on: 2023. 11. 9.
 *      Author: 21310
 */


#include "vl53l0x_jh.h"
#include "i2c.h"

#define preset

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
	uint32_t refSpadCountHost[NUM_SENSOR]={4,5,3,5,5,5,4,4,9,5,5,6,4,5,4,4,5,5,4,5,4,3,4,6,5,4,4,5,5,5,5,5,5,5,4,5};
	uint8_t isApertureSpadsHost[NUM_SENSOR]={0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t VhvSettingsHost[NUM_SENSOR]={25,27,29,24,30,30,28,34,29,29,25,27,32,31,32,30,30,27,28,33,33,31,29,29,33,28,32,33,33,32,33,31,30,32,30,34};
	uint8_t PhaseCalHost[NUM_SENSOR]={1,1,1,1,1,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
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


VL53L0X_RangingMeasurementData_t RangingData;

VL53L0X_Dev_t vl53l0x_s[NUM_SENSOR];
VL53L0X_DEV Dev;

uint8_t tca_ch[8] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}; // control register of TCA9548A
uint8_t tca_ch_reset = 0x00;

#if NUM_SENSOR == 24
  uint8_t tca_addr[4] = {0x70,0x71,0x72,0x73}; // 2 line
  uint8_t tcaLength = 4;
#elif NUM_SENSOR == 36
  uint8_t tca_addr[6] = {0x70,0x71,0x72,0x73,0x74,0x75}; // 3 line
  uint8_t tcaLength = 6;
#elif NUM_SENSOR == 48
  uint8_t tca_addr[8] = {0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77};  // 4 line
  uint8_t tcaLength = 8;
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
