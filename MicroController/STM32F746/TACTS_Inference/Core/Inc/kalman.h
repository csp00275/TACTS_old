/*
 * kalman.h
 *
 *  Created on: 2023. 10. 04.
 *      Author: JH_LAB
 */

#ifndef KALMAN_H
#define KALMAN_H

typedef struct {
    float Q;  // process noise covariance
    float R;  // measurement noise covariance
    float P;  // estimation error covariance
    float X;  // value of the best estimate of the desired variable
    float K;  // Kalman gain
} KalmanFilter;

// Initialize Kalman filter
void Kalman_Init(KalmanFilter *kf, float Q, float R, float P, float initial_value);

// Estimate the new value
float Kalman_Estimate(KalmanFilter *kf, float measurement);

#endif // KALMAN_H
