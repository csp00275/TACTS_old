#include <Wire.h>
#include <VL53L0X.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR70 0x70
#define TCAADDR71 0x71
#define NUM_SENSORS 8

void tcaselect(uint8_t i,uint8_t j) {
  if (i > 7) return;
  Wire.beginTransmission(j);
  Wire.write(1<<i);
  Wire.endTransmission();
}

typedef VL53L0X* VL53L0XPtr;
VL53L0XPtr sensors[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println("Starting....");
  
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    tcaselect(i,0x70);
    VL53L0XPtr sensor = new VL53L0X();
    sensor->init();
    sensor->setTimeout(1000);
    sensor->setMeasurementTimingBudget(33000);
    sensor->startContinuous();
    sensors[i] = sensor;
  }
  Wire.beginTransmission(0x70);
  Wire.write(0);
  Wire.endTransmission();

  for (uint8_t i = 0; i < 8; i++)
  {
    tcaselect(i,0x71);
    VL53L0XPtr sensor = new VL53L0X();
    sensor->init();
    sensor->setTimeout(1000);
    sensor->setMeasurementTimingBudget(33000);
    sensor->startContinuous();
    sensors[i] = sensor;
  }
  Serial.println("Ending....");
}

void loop() {
long TimeStart = millis();


    for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    Wire.beginTransmission(0x71);
    Wire.write(0);
    Wire.endTransmission();
    tcaselect(i,0x70);
    VL53L0XPtr sensor = sensors[i];
    
    if (sensor->last_status == 0) // 0: succes other : read 'Wire.endTransmission()'
    {
      int senVal = sensor->readRangeContinuousMillimeters();
      if(senVal >8000){senVal=0;}
      Serial.print(senVal);
      Serial.print(" ");      
    }
    else
    {
      Serial.print(sensor->last_status);
      Serial.print(" ");
    }

    if (sensor->timeoutOccurred()) {Serial.print(" [timeout]");}
  }
  
  for (uint8_t i = 0; i < 8; i++)
  {
    Wire.beginTransmission(0x70);
    Wire.write(0);
    Wire.endTransmission();
    tcaselect(i,0x71);
    VL53L0XPtr sensor = sensors[i];
    
    if (sensor->last_status == 0) // 0: succes other : read 'Wire.endTransmission()'
    {
      int senVal = sensor->readRangeContinuousMillimeters();
      if(senVal >8000){senVal=0;}
      Serial.print(senVal);
      Serial.print(" ");      
    }
    else
    {
      Serial.print("error");
      Serial.print(sensor->last_status);
      Serial.print(" ");
    }
    if (sensor->timeoutOccurred()) {Serial.print(" [timeout]");}
  }  

  long Timeend = millis();

  //Serial.print(Timeend-TimeStart);
  Serial.println();
}
