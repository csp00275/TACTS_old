#include <Wire.h>
#include <VL53L0X.h>
#include <MedianFilter.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70
#define NUM_SENSORS 24
int a=0;

float coeff[3][8] = 
{{1.16,0.76,0.89,0.89,0.94,0.87,0.95,0.90},
{0.91,0.91,0.90,0.91,0.95,1.00,0.84,0.89},
{0.91,0.87,0.91,1.00,0.86,0.91,1.02,0.89}};
float inter[3][8] = 
{{-98.13,4.89,-24.37,-38.38,-10.47,-32.66,-51.50,-12.94},
{-15.78,-16.66,-6.80,-8.03,-15.59,-12.23,-12.58,-26.27},
{-24.09,-12.15,-31.07,-19.40,1.38,-14.71,-45.72 -2.82}};

float senvalLR[24];  

void tcaselect(uint8_t i,uint8_t j) {
  if (i > 7) return;
  Wire.beginTransmission(j);
  Wire.write(1<<i);
  Wire.endTransmission();
}

void tcaoff(uint8_t j) {
  Wire.beginTransmission(j);
  Wire.write(0);
  Wire.endTransmission();
}

typedef VL53L0X* VL53L0XPtr;
VL53L0XPtr sensors[NUM_SENSORS];

#define t 3  // number of median array 몇 개 할거니?
#define y 55 // 초기값 

#define mf MedianFilter(t,y)

MedianFilter MF[] = {mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf,mf};  //객체 배열 생성


void setup() {
  Serial.begin(115200);
  Wire.begin();
//  Serial.println("Starting");
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    int q = i / 8;    
    int r = i % 8;
    tcaoff(TCAADDR + (q+1) % 3);
    tcaoff(TCAADDR + (q+2) % 3);
    tcaselect(r, TCAADDR + q);
    VL53L0XPtr sensor = new VL53L0X();
    sensor->init();    
    sensor->setMeasurementTimingBudget(33000);
    sensor->startContinuous();
    sensors[i] = sensor;
  }
  //Serial.println("Ending....");
}

void loop(){
  
  long TimeStart = millis();

//  if(a==0)
//  {
//    Serial.print("COM Start");
//    a=1;
//  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  { 
    int q = i / 8;
    int r = i % 8;
    tcaoff(TCAADDR + (q+1) % 3);
    tcaoff(TCAADDR + (q+2) % 3);
    tcaselect(r, TCAADDR + q);
    VL53L0XPtr sensor = sensors[i];    
    if (sensor->last_status == 0) // 0: succes other : read 'Wire.endTransmission()'
    {
      float senVal = sensor->readRangeContinuousMillimeters();
      senvalLR[i] = coeff[q][r]*senVal + inter[q][r];
      MF[i].in(senvalLR[i]);
      senvalLR[i] = MF[i].getMean();
      if(senvalLR[i] >999){senvalLR[i]=0;}
      Serial.print(senvalLR[i]);
      Serial.print(" ");      
    }
    else
    {
      Serial.print("(");
      Serial.print(sensor->last_status);
      Serial.print(" ");
      Serial.print(TCAADDR + q,HEX);
      Serial.print("_");
      Serial.print(r);
      Serial.print(") ");
    }
  }

  long Timeend = millis();
  
//  Serial.print("   ");
//  Serial.print("Sensing Time: ");
//  Serial.print(Timeend-TimeStart);
  Serial.println();
}
