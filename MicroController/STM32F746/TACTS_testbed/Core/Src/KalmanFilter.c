///*
// * kalman.c
// *
// *  Created on: 2023. 4. 11.
// *      Author: JH_LAB
// */
//
//
//// KalmanFilter.c
//#include "KalmanFilter.h"
//
//void KalmanFilter_Init(KalmanFilter *kf, float processNoise, float measurementNoise, float errorCovariance)
//{
//    kf->Q = processNoise;
//    kf->R = measurementNoise;
//    kf->P = errorCovariance;
//    kf->X_estimate = 0.0f;
//}
//
//float KalmanFilter_Estimate(KalmanFilter *kf, float measurement)
//{
//    // Predict
//    kf->P = kf->P + kf->Q;
//
//    // Update
//    kf->K = kf->P / (kf->P + kf->R);
//    kf->X_estimate = kf->X_estimate + kf->K * (measurement - kf->X_estimate);
//    kf->P = (1 - kf->K) * kf->P;
//
//    return kf->X_estimate;
//}
