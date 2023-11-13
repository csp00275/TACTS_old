/*
 * kalman.h
 *
 *  Created on: 2023. 10. 04.
 *      Author: JH_LAB
 */

#ifndef KALMAN_H
#define KALMAN_H

#include "vl53l0x_jh.h"


typedef struct {
    float Q;  // process noise covariance
    float R;  // measurement noise covariance
    float P;  // estimation error covariance
    float X;  // value of the best estimate of the desired variable
    float K;  // Kalman gain
} KalmanFilter;


extern KalmanFilter filters[NUM_SENSOR];
extern float Q; // Process noise covariance
extern float R;   // Measurement noise covariance
extern float P;

// Initialize Kalman filter
void Kalman_Init(KalmanFilter *kf, float Q, float R, float P, float initial_value);

// Estimate the new value
float Kalman_Estimate(KalmanFilter *kf, float measurement);

#endif // KALMAN_H
