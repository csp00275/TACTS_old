#include <math.h>
#define RESOLUTION 20
#define NUM_SENSOR 24

float baseForce = 50.0; // 기본 힘 값
float currentTheta = 0.0; // 현재 각도
float currentZ = 10.0; // 현재 Z 위치

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
}

void loop() {
  moveTheta(90);   // θ를 90도로 이동
  moveZ(10);       // Z를 10으로 이동
  moveTheta(0);   // θ를 90도로 유지
  moveZ(50);       // Z를 50으로 이동
  moveTheta(90);   // θ를 90도로 이동
  moveZ(90);       // Z를 10으로 이동
  moveTheta(0);   // θ를 90도로 유지
  moveZ(140);       // Z를 50으로 이동
}

void moveTheta(float endTheta) {
  float thetaStep = (endTheta - currentTheta) / RESOLUTION;

  for(int i = 0; i < RESOLUTION; i++){
    currentTheta += thetaStep;
    float randomForce = baseForce + getRandomFloat(-2.0, 2.0);
    printPosition(randomForce, currentZ, cos(radians(currentTheta)), sin(radians(currentTheta)));
    delay(NUM_SENSOR*2); // 데이터 전송 후 잠시 대기
  }

  currentTheta = endTheta; // 마지막 위치 보정
}

// Z 이동 함수
void moveZ(float endZ) {
  float zStep = (endZ - currentZ) / RESOLUTION;

  for(int i = 0; i < RESOLUTION; i++){
    currentZ += zStep;
    float randomForce = baseForce + getRandomFloat(-2.0, 2.0);
    printPosition(randomForce, currentZ, cos(radians(currentTheta)), sin(radians(currentTheta)));
    delay(NUM_SENSOR*2); // 데이터 전송 후 잠시 대기
  }

  currentZ = endZ; // 마지막 위치 보정
}

void printPosition(float force, float Z, float cosTheta, float sinTheta) {
  Serial.print(force);
  Serial.print(" ");
  Serial.print(Z);
  Serial.print(" ");
  Serial.print(cosTheta);
  Serial.print(" ");
  Serial.println(sinTheta);
  // delay는 여기서 제거하고, moveTheta와 moveZ에 추가함
}

float getRandomFloat(float min, float max) {
  return min + (max - min) * random(10001) / 10000.0;
}
