#include <Wire.h>
#include <VL53L0X.h>
#include <MedianFilter.h>

VL53L0X sensor[] = {VL53L0X(),VL53L0X()};

#define t 5     //몇개를 median 할꺼냐
#define y 40  //초기값 

MedianFilter MF[] = {MedianFilter(t,y), MedianFilter(t,y)};


int s[]={2,3}; // 센서 xshut pin 번호 매칭
int ArraySize = sizeof(s)/sizeof(int);
int val[2];
int cali[]={12,15};
void setup()
{
  Wire.begin();
  Serial.begin (115200);
  for (int i =0; i<ArraySize; i++ )
  {
    pinMode(s[i],OUTPUT);
    digitalWrite(s[i], HIGH);
    sensor[i].init(true);
    sensor[i].setAddress((uint8_t)i+1);
    sensor[i].startContinuous();
  }
}


void loop()
{

    for (int i =0; i<ArraySize; i++ )
    {
      val[i] = sensor[i].readRangeContinuousMillimeters();
      MF[i].in(val[i]);
      val[i]=MF[i].out();
      if(val[i]>8000)
      {
        val[i]=0;
      }
      Serial.print(val[i]-cali[i]);
      Serial.print(" ");    
    }    
    
   Serial.println("1");
}
