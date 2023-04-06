#include <Wire.h>
#include <VL53L0X.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR70 0x70
#define TCAADDR71 0x71
#define NUM_SENSORS 8

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR70);
  Wire.write(1<<i);
  Wire.endTransmission();
}

typedef VL53L0X* VL53L0XPtr;

VL53L0XPtr sensors[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  Wire.begin();

  
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    tcaselect(i);
    VL53L0XPtr sensor = new VL53L0X();
    sensor->init();
    sensor->setTimeout(1000);
    sensor->setMeasurementTimingBudget(33000);
    sensor->startContinuous();
    sensors[i] = sensor;
  } 
}

void loop() {
long TimeStart = millis();

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
  {
    tcaselect(i);
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
  long Timeend = millis();

  Serial.print("Time: ");
  Serial.println(Timeend-TimeStart);
}
