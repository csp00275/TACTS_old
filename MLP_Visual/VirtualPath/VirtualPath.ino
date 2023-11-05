#include <math.h>
#define RESOLUTION 25
#define NUM_SENSOR 24

float baseForce = 50.0; // 기본 힘 값
float currentTheta = 0.0; // 현재 각도
float currentZ = 10.0; // 현재 Z 위치

void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0));
}

void loop() {
  moveToNextPosition(90, 10); // 다음 위치로 이동 (theta,Z) = (90,10)
  moveToNextPosition(90, 50); // (theta,Z) = (90,50)
  moveToNextPosition(0, 50);  // (theta,Z) = (0,50)
  moveToNextPosition(0, 90);  // (theta,Z) = (0,90)
  moveToNextPosition(90, 90); // 다음 위치로 이동 (theta,Z) = (90,10)
  moveToNextPosition(90, 140); // (theta,Z) = (90,50)
  moveToNextPosition(0, 140);  // (theta,Z) = (0,50)
  moveToNextPosition(-90, 140);  // (theta,Z) = (0,90)
  moveToNextPosition(-90, 100);  // (theta,Z) = (0,90)
  moveToNextPosition(45, 100);  // (theta,Z) = (0,90)
  // ... 계속 다음 위치로 이동하는 코드를 추가
  //while(true); // 루프를 멈추거나 지속적인 동작을 위한 조건을 추가
}

void moveToNextPosition(float endTheta, float endZ) {
  // theta 이동
  if(currentTheta != endTheta){
    for(int i = 0; i < RESOLUTION; i++){
      currentTheta += (endTheta - currentTheta) / RESOLUTION;
      float randomForce = baseForce + getRandomFloat(-2.0, 2.0);
      printPosition(randomForce, currentZ, cos(radians(currentTheta)), sin(radians(currentTheta)));
    }
  }
  // Z 이동
  if(currentZ != endZ){
    for(int i = 0; i < RESOLUTION; i++){
      currentZ += (endZ - currentZ) / RESOLUTION;
      float randomForce = baseForce + getRandomFloat(-2.0, 2.0);
      printPosition(randomForce, currentZ, cos(radians(currentTheta)), sin(radians(currentTheta)));
    }
  }
}

void printPosition(float force, float Z, float cosTheta, float sinTheta) {
  Serial.print(force);
  Serial.print(" ");
  Serial.print(Z);
  Serial.print(" ");
  Serial.print(cosTheta);
  Serial.print(" ");
  Serial.println(sinTheta);
  delay(NUM_SENSOR*2); // 데이터 전송 후 잠시 대기
}

float getRandomFloat(float min, float max) {
  return min + (max - min) * random(10001) / 10000.0;
}
