/*
 * kalman.c
 *
 *  Created on: 2023. 4. 11.
 *      Author: JH_LAB
 */


#include "kalman.h"

typedef struct {
    float Q;
    float R;
    float P;
    float X_estimate;
    float K;
} KalmanFilter;

void Kalman_Init(KalmanFilter* filter, float processNoise, float measurementNoise, float errorCovariance) {
    filter->Q = processNoise;
    filter->R = measurementNoise;
    filter->P = errorCovariance;
    filter->X_estimate = 0;
}

float Kalman_Estimate(KalmanFilter* filter, float measurement) {
    filter->P = filter->P + filter->Q;
    filter->K = filter->P / (filter->P + filter->R);
    filter->X_estimate = filter->X_estimate + filter->K * (measurement - filter->X_estimate);
    filter->P = (1 - filter->K) * filter->P;
    return filter->X_estimate;
}
