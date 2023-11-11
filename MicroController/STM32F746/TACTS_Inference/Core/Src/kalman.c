/*
 * kalman.c
 *
 *  Created on: 2023. 10. 04.
 *      Author: JH_LAB
 */


#include "kalman.h"

void Kalman_Init(KalmanFilter *kf, float Q, float R, float P, float initial_value) {
    kf->Q = Q;
    kf->R = R;
    kf->P = P;
    kf->X = initial_value;
}

float Kalman_Estimate(KalmanFilter *kf, float measurement) {
    // Prediction step: Update the error covariance 'P'
    // This is adding process noise to our uncertainty in the estimated state.
    kf->P = kf->P + kf->Q;

    // Update step: Compute the Kalman Gain 'K'
    // Kalman Gain balances the ratio between the prediction and the new measurement
    kf->K = kf->P / (kf->P + kf->R);

    // Update step: Update the state estimate 'X'
    // Here we adjust our state estimate based on the new measurement,
    // scaling the "innovation" by the Kalman Gain
    kf->X = kf->X + kf->K * (measurement - kf->X);

    // Update step: Update the error covariance 'P'
    // We've used our observation, so we adjust our uncertainty accordingly
    kf->P = (1 - kf->K) * kf->P;

    // Return the updated state estimate
    return kf->X;
}


