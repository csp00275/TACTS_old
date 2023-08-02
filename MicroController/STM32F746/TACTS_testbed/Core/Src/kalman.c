/*
 * kalman.c
 *
 *  Created on: 2023. 4. 11.
 *      Author: JH_LAB
 */


#include "kalman.h"

void Kalman_Init(KalmanFilter* filter, float processNoise, float measurementNoise, float errorCovariance) {
    filter->Q = processNoise;
    filter->R = measurementNoise;
    filter->P = errorCovariance;
    filter->X_estimate = 0;
}

float KalmanFilter_Update(KalmanFilter *filter, float measurement) {
    filter->P += filter->Q;
    filter->K = filter->P / (filter->P + filter->R);
    filter->X += filter->K * (measurement - filter->X);
    filter->P *= (1.0f - filter->K);
    return filter->X;
}
