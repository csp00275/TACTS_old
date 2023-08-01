#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

#define MAX_DATA_POINTS 7  // Maximum number of data points for moving average filter

class MovingAverageFilter {
  int data[MAX_DATA_POINTS];
  int dataIndex = 0;
  int total = 0;
  int numDataPoints = 0;
  int windowSize;

public:
  MovingAverageFilter(int windowSize) : windowSize(windowSize) {}

  float calculate(float newData) {
    if (numDataPoints < windowSize) {
      numDataPoints++;
    } else {
      total -= data[dataIndex];
    }
    total += newData;
    data[dataIndex] = newData;
    dataIndex = (dataIndex + 1) % windowSize;
    return total / numDataPoints;
  }
};

//MovingAverageFilter filter3(3);
MovingAverageFilter filter5(5);
//MovingAverageFilter filter7(7);

class MovingMedianFilter {
  int data[MAX_DATA_POINTS];
  int dataIndex = 0;
  int windowSize;

public:
  MovingMedianFilter(int windowSize) : windowSize(windowSize) {}

  void bubbleSort(int arr[], int n) {
    for (int i = 0; i < n-1; i++) {
      for (int j = 0; j < n-i-1; j++) {
        if (arr[j] > arr[j+1]) {
          int temp = arr[j];
          arr[j] = arr[j+1];
          arr[j+1] = temp;
        }
      }
    }
  }

  int calculate(int newData) {
    data[dataIndex] = newData;
    dataIndex = (dataIndex + 1) % windowSize;
    
    int sortedData[MAX_DATA_POINTS];
    for (int i = 0; i < windowSize; i++) {
      sortedData[i] = data[i];
    }

    bubbleSort(sortedData, windowSize);

    if (windowSize % 2 == 0) {
      return (sortedData[windowSize/2 - 1] + sortedData[windowSize/2]) / 2;
    } else {
      return sortedData[windowSize/2];
    }
  }
};

//MovingMedianFilter medianFilter3(3);
MovingMedianFilter medianFilter5(5);
//MovingMedianFilter medianFilter7(7);

class KalmanFilter {
private:
    float Q; // process noise covariance
    float R; // measurement noise covariance
    float P; // estimation error covariance
    float X_estimate; // value of the best estimate of the desired variable
    float K; // Kalman gain
    
public:
    KalmanFilter(float processNoise, float measurementNoise, float errorCovariance)
    {
        Q = processNoise;
        R = measurementNoise;
        P = errorCovariance;
        X_estimate = 0;
    }

    float estimate(float measurement)
    {
        // Predict
        P = P + Q;

        // Update
        K = P / (P + R);
        X_estimate = X_estimate + K * (measurement - X_estimate);
        P = (1 - K) * P;

        return X_estimate;
    }
};

// 사용 예시
KalmanFilter kalmanFilter1(0.001, 0.03, 0.1); // Q, R, and initial P values respectively. Adjust based on your specific use case.
KalmanFilter kalmanFilter2(0.002, 0.04, 0.1);
KalmanFilter kalmanFilter3(0.003, 0.05, 0.1);
// class MovingStatistics {
//   float data[MAX_DATA_POINTS];
//   int dataIndex = 0;
//   int numDataPoints = 0;
//   int windowSize;
//   float sum = 0;
//   float sumSquares = 0;

// public:
//   MovingStatistics(int windowSize) : windowSize(windowSize) {}

//   float average(float newData) {
//     if (numDataPoints < windowSize) {
//       numDataPoints++;
//     } else {
//       sum -= data[dataIndex];
//       sumSquares -= data[dataIndex] * data[dataIndex];
//     }

//     sum += newData;
//     sumSquares += newData * newData;
//     data[dataIndex] = newData;
//     dataIndex = (dataIndex + 1) % windowSize;

//     return sum / numDataPoints;
//   }

//   float standardDeviation() {
//     float mean = sum / numDataPoints;
//     float variance = (sumSquares / numDataPoints) - (mean * mean);
//     return sqrt(variance);
//   }
// };

// MovingStatistics statsRaw(20);
// MovingStatistics statsFilter3(20);
// MovingStatistics statsFilter5(20);
// MovingStatistics statsFilter7(20);
// MovingStatistics statsMedian3(20);
// MovingStatistics statsMedian5(20);
// MovingStatistics statsMedian7(20);


void setup() {
  Serial.begin(115200);
  Wire.begin();
  sensor.setTimeout(500);
  
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  sensor.setMeasurementTimingBudget(33000);
}

void loop() {
  float range = sensor.readRangeSingleMillimeters();
  Serial.print(range);
  Serial.print(" ");
  Serial.print(filter5.calculate(range));
  Serial.print(" ");
  Serial.print(medianFilter5.calculate(range));
  Serial.print(" ");
  Serial.print(kalmanFilter1.estimate(range));
  Serial.print(" ");
  Serial.print(kalmanFilter2.estimate(range));
  Serial.print(" ");
  Serial.print(kalmanFilter3.estimate(range));
  Serial.println();
}


