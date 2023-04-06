#include <Wire.h>
#include <VL53L0X.h>
#include <MedianFilter.h>

VL53L0X sensor;
VL53L0X sensor2;

#define t 5     //몇개를 median 할꺼냐
#define y 40  //초기값 
#define sensor1_Pin 2
#define sensor2_Pin 3

MedianFilter test1(t, y);
MedianFilter test2(t, y);

int a,b;

void setup()

{

pinMode(sensor1_Pin, OUTPUT);
pinMode(sensor2_Pin, OUTPUT);

digitalWrite(sensor1_Pin, LOW);
digitalWrite(sensor2_Pin, LOW);

delay(500);

Wire.begin();
Serial.begin (115200);

digitalWrite(sensor1_Pin, HIGH);
sensor.init(true);
sensor.setAddress((uint8_t)01);

digitalWrite(sensor2_Pin, HIGH);
sensor2.init(true);
sensor2.setAddress((uint8_t)02);

sensor.startContinuous();
sensor2.startContinuous();

}


void loop()

{
  float val1 = sensor.readRangeContinuousMillimeters();
      test1.in(val1);
      val1=test1.out();
    //  c=sensor3.readRangeContinuousMillimeters();
     if(val1>8000){
      val1=0;
     }
Serial.print(val1);
Serial.print(" ");



  float val2 = sensor2.readRangeContinuousMillimeters();
      test2.in(val2);
      val2=test2.out();
    if(val2>8000){
      val2=0;
     }
Serial.print(val2);
Serial.println(" ");





}
