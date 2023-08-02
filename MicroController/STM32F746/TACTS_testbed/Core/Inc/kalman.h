/*
 * kalman.h
 *
 *  Created on: 2023. 4. 11.
 *      Author: JH_LAB
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_


typedef struct {
    float Q;
    float R;
    float P;
    float K;
    float X;
} KalmanFilter;

void Kalman_Init(KalmanFilter* filter, float processNoise, float measurementNoise, float errorCovariance);
float Kalman_Estimate(KalmanFilter* filter, float measurement);


#endif /* INC_KALMAN_H_ */
