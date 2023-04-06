/////////////////////////////////////////////////////////////
///////////////////Made by Park Jae Hyeong///////////////////
//////////////contact e-mail : csp00275@skku.edu/////////////
/////////////////////////////////////////////////////////////
#include <Wire.h>
#include <VL53L0X.h>
#include <MedianFilter.h>

#define vl VL53L0X()
VL53L0X sensor[] = {vl,vl,vl,vl,vl,vl,vl,vl,vl,vl};  //객체 배열 생성


#define t 11  // 샘플링하는 값 갯수
#define y 35 // 초기값 

#define mf MedianFilter(t,y)

MedianFilter MF[] = {mf,mf,mf,mf,mf,mf,mf,mf,mf,mf};  //객체 배열 생성


int s[]={22,23,24,25,26,27,28,29,30,31}; // 센서 xshut pin 번호 매칭
int ArraySize = sizeof(s)/sizeof(int);  // sizeof는 byte를 반환함.
float val[10];  
float cali[]={24,31,33,5,26,26,45,17,24,20};  // 캘리브레이션 할 값
uint8_t Adress[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};

void setup()
{

  for (int i =0; i<ArraySize; i++ )
  {
    pinMode(s[i],OUTPUT);
    digitalWrite(s[i], LOW);
  }
  
  Serial.begin (115200);
  Wire.begin();
  
  for (int i =0; i<ArraySize; i++ )
  {
    digitalWrite(s[i], HIGH);
    sensor[i].init(true);
    sensor[i].setAddress(Adress[i]);
    sensor[i].startContinuous();
  }
//  Serial.print("ArraySize is ");
//  Serial.println(ArraySize);
}


void loop()
{

    for (int i =0; i<ArraySize; i++ )
    {
      val[i] = sensor[i].readRangeContinuousMillimeters();
      MF[i].in(val[i]);
      val[i]=MF[i].getMean();  // 중간값
      //val[i]=MF[i].getMin();  // 최솟값
      //val[i]=MF[i].getMax();  // 최댓값
      //val[i]=MF[i].getMean(); // 평균값 ( 자료형 int)
      //val[i]=MF[i].getStDev(); // 표준편차 
      //////위의 모든 값 자료형 int//////
      if(val[i]>8000)
      {
        val[i]=0;
      }
      Serial.print(val[i]-cali[i]);
      Serial.print(" ");    
    }    
    
   Serial.println("1");
   delay(100);
}
