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

void KalmanFilter_Init(KalmanFilter *filter, float Q, float R);
float KalmanFilter_Update(KalmanFilter *filter, float measurement);


#endif /* INC_KALMAN_H_ */
