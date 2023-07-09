#include <Wire.h>
#include <VL53L0X.h>
#include <Kalman.h> // Include the Kalman library


VL53L0X sensor;

#define HIGH_SPEED

#define MAX_DATA_POINTS 100  // Maximum number of data points for moving average filter

class MovingAverageFilter {
  int data[MAX_DATA_POINTS];  // Array to store data points
  int dataIndex = 0;         // Index to keep track of where to insert new data points
  int total = 0;             // Sum of data points for calculating average
  int numDataPoints = 0;     // Number of data points collected so far
  int windowSize;

public:
  MovingAverageFilter(int windowSize) : windowSize(windowSize) {}

  // Function to add a new data point and calculate the moving average
  float calculate(float newData) {
    // Check if maximum number of data points have been collected
    if (numDataPoints < windowSize) {
      numDataPoints++;
    } else {
      // Subtract the oldest data point from total
      total -= data[dataIndex];
    }

    // Add new data point to total and data array
    total += newData;
    data[dataIndex] = newData;

    // Increment data index
    dataIndex = (dataIndex + 1) % windowSize;

    // Return the moving average
    return total / numDataPoints;
  }
};

MovingAverageFilter filter3(3);
MovingAverageFilter filter5(5);
MovingAverageFilter filter7(7);
class KalmanFilter {
  float A, Q, H, R;
  float state_estimate, covariance_estimate;

public:
  KalmanFilter(float A, float Q, float H, float R) : A(A), Q(Q), H(H), R(R), state_estimate(0), covariance_estimate(Q) {}

  float update(float measurement) {
    // prediction update
    float prediction = A * state_estimate;
    float predictionCovariance = A * covariance_estimate * A + Q;

    // measurement update
    float K = predictionCovariance * H / (H * predictionCovariance * H + R);
    state_estimate = prediction + K * (measurement - H * prediction);
    covariance_estimate = (1 - K * H) * predictionCovariance;

    return state_estimate;
  }
};

// Initialize the A, Q, H, and R values
float A = 1.0;  // Value for A
float Q = 1;  // Value for Q
float H = 1.02;  // Value for H
float R = 1.04; // Value for R

// Create an instance of the KalmanFilter class
KalmanFilter kalmanFilter(A, Q, H, R);


void setup()
{
  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  sensor.setMeasurementTimingBudget(20000);
}

void loop()
{
  float range = sensor.readRangeSingleMillimeters();

  float estimatedState = kalmanFilter.update(range);

  Serial.print(range);
  Serial.print(" ");
  Serial.print(filter3.calculate(range));
  Serial.print(" ");
  Serial.print(filter5.calculate(range));
  Serial.print(" ");
  Serial.print(filter7.calculate(range));
  Serial.print(" ");
  Serial.print(estimatedState);
  Serial.println();
}
