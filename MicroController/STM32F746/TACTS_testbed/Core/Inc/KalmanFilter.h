///*
// * kalman.h
// *
// *  Created on: 2023. 4. 11.
// *      Author: JH_LAB
// */
//
//// KalmanFilter.h
//#ifndef KALMANFILTER_H
//#define KALMANFILTER_H
//
//typedef struct
//{
//    float Q;             // Process noise covariance
//    float R;             // Measurement noise covariance
//    float P;             // Estimation error covariance
//    float X_estimate;    // Value of the best estimate of the desired variable
//    float K;             // Kalman gain
//} KalmanFilter;
//
//void KalmanFilter_Init(KalmanFilter *kf, float processNoise, float measurementNoise, float errorCovariance);
//float KalmanFilter_Estimate(KalmanFilter *kf, float measurement);
//
//#endif // KALMANFILTER_H
