/*
 * kalman.c
 *
 *  Created on: 2023. 4. 11.
 *      Author: JH_LAB
 */


#include "kalman.h"

void KalmanFilter_Init(KalmanFilter *filter, float Q, float R) {
    filter->Q = Q;
    filter->R = R;
    filter->P = 0;
    filter->K = 0;
    filter->X = 0;
}

float KalmanFilter_Update(KalmanFilter *filter, float measurement) {
    filter->P += filter->Q;
    filter->K = filter->P / (filter->P + filter->R);
    filter->X += filter->K * (measurement - filter->X);
    filter->P *= (1.0f - filter->K);
    return filter->X;
}
