//깊이(r) 각도(theta) 높이(z)  이렇게 입력해야됨 그러면 각도 바뀌고 거리 바뀌고 서보 움직이고
#include<Servo.h>
#include <Wire.h>
#include <VL53L0X.h>

Servo servo;

#define X_STEP_PIN         2
#define X_DIR_PIN          3
#define X_ENABLE_PIN       4
#define Y_STEP_PIN         5
#define Y_DIR_PIN          6
#define Y_ENABLE_PIN       7
#define ANG_RPM            10
#define DIST_RPM           200

#define INC_THETA          10
#define INC_Z              4


int CLKPin_1 =2; // CLK1
int dirPin_1 =3; // DIR1
int enaPin_1 =4;  // ENA1
int CLKPin_2 =5; // CLK2
int dirPin_2 =6; // DIR2
int enaPin_2 =7;  // ENA2

int a=0;
int i;
String string="";
long value_rev_num = 1;
float value_rev_angle=0;
long value_rev_dist = 0;
float servo_dist = 0;
int DeltaTime=0;

#define vl VL53L0X()
VL53L0X sensor[] = {vl,vl,vl,vl,vl,vl,vl,vl,vl,vl};  //객체 배열 생성
int s[]={22,23,24,25,26,27,28,29,30,31}; // 센서 xshut pin 번호 매칭
int ArraySize = sizeof(s)/sizeof(int);  // sizeof는 byte를 반환함.
float val[10];  
float coeff[]={0.86,0.91,0.97,0.89,0.96,0.94,0.93,0.96,0.99,0.89};  // 캘리브레이션 할 값
float inter[]={-0.21,-5.04,-0.23,2.42,1.46,-37.32,-18.9,-37.66,-20.51,0.5};  // 캘리브레이션 할 값
uint8_t Adress[] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09};


#include <SparkFun_MS5803_I2C.h>
MS5803 PreSensor(ADDRESS_HIGH);  //  ADDRESS_HIGH = 0x76

#define encoderPinA 18
#define encoderPinB 19
int encoderPos = 0;
void doEncoderA(){encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1;}
void doEncoderB(){encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1;}

void setup() {
       
  pinMode(X_STEP_PIN,   OUTPUT);
  pinMode(X_DIR_PIN,    OUTPUT);
  pinMode(X_ENABLE_PIN, OUTPUT);
  pinMode(Y_STEP_PIN,   OUTPUT);
  pinMode(Y_DIR_PIN,    OUTPUT);
  pinMode(Y_ENABLE_PIN, OUTPUT);
  servo.attach(9);
  servo.write(0);

    ////////////////////ENCODER//////////////////////
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);
 
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);
  /////////////////////////////////////////////////


  for (int i =0; i<ArraySize; i++ )
  {
    pinMode(s[i],OUTPUT);
    digitalWrite(s[i], LOW);
  }
  delay(500);

  Serial.begin (115200);
  Wire.begin();

  PreSensor.reset();
  PreSensor.begin();

  for (int i =0; i<ArraySize; i++ )
  {
    digitalWrite(s[i], HIGH);
    sensor[i].init(true);
    sensor[i].setAddress(Adress[i]);
    sensor[i].startContinuous();
    sensor[i].setMeasurementTimingBudget(50000);

  }
}

void loop() {
   if(a==1)
     {
        Serial.print("--------------------------------------------------------------\n");
        Serial.print("---------- stepping motor for DTCA made by JaeHyeong----------\n");
      //Serial.print("-------# of rev(n) & speed(rpm) or 0 & holding time(s)--------\n");
        Serial.print("-------------------------Type 'rev'---------------------------\n");
        Serial.print("-------Positive: CW / Negative: CCW / Zero : holding ---------\n");
        Serial.print("-------------------------Max RPM is 270-----------------------\n");
        Serial.print("--------------------automatic_v6------------------------------\n");
        a=1;
    }

if(Serial.available())
  {
    string = Serial.readString();

    //////////////////////  R  E  V  //////////////////////////
          if(string=="rev")
          {
            Serial.println("Put servo distance (cm) & the angle(deg) & distance(mm)");
            while(true)
            {
              if(Serial.available())
              {
                servo_dist = Serial.parseFloat();//얼마나 찌를꺼냐
                value_rev_angle = Serial.parseFloat();
                value_rev_dist = Serial.parseInt();
                break;
              }
            }

            float ANG_deltaStep=(float)1/2/1600*1000*1000*60/ANG_RPM; // micro seconds
            float DIST_deltaStep=(float)1/2/1600*1000*1000*60/DIST_RPM; // micro seconds
            Serial.println("//////////////////////////");
            Serial.print("Angle(degree) : ");
            Serial.println(value_rev_angle); 
            Serial.print("Distance(mm) : ");
            Serial.println(value_rev_dist); 
            Serial.print("Servo distance(cm) : ");
            Serial.println(servo_dist);
            Serial.println("//////////////////////////");

            stepRotate1(CLKPin_1,dirPin_1,enaPin_1,ANG_deltaStep,value_rev_angle);//각도바꿔주고
            stepRotate2(CLKPin_2,dirPin_2,enaPin_2,DIST_deltaStep,value_rev_dist);//거리바꿔주고
            poking(servo_dist);

            Serial.println("rotation end");
            servo_dist = 0;
            value_rev_angle = 0;
            value_rev_dist = 0;
       
           
          }
                 
          if(string=="auto")
          {
            
            for(int z=0; z<10;z++)
            {  
              for(int theta=0; theta<36; theta++)
              {            
                
                for(int r=0; r<10;r++)
                {
                servo_dist = r*0.2; //absolute
                float ANG_deltaStep=(float)1/2/1600*1000*1000*60/ANG_RPM; // micro seconds
                float DIST_deltaStep=(float)1/2/1600*1000*1000*60/DIST_RPM; // micro seconds
                
                stepRotate1(CLKPin_1,dirPin_1,enaPin_1,ANG_deltaStep,value_rev_angle);//각도바꿔주고
                stepRotate1(CLKPin_1,dirPin_1,enaPin_1,ANG_deltaStep,0);//고정시켜주고
                stepRotate2(CLKPin_2,dirPin_2,enaPin_2,DIST_deltaStep,value_rev_dist);//거리바꿔주고
                poking(servo_dist);
                delay(1000);

                long int CurTime = millis();
                do
                {
                  for (int i =0; i<ArraySize; i++ )
                  {
                    val[i] = sensor[i].readRangeContinuousMillimeters();
                    if(val[i]>8000){val[i]=0;}
                    val[i] = coeff[i]*val[i]+inter[i];
                    Serial.print(val[i]);
                    Serial.print(" ");    
                  }
                  double temperature_c = PreSensor.getTemperature(CELSIUS, ADC_512);
                  double pressure_abs = PreSensor.getPressure(ADC_4096);                   
                  int modulo = (encoderPos+8192) % 8192;
                  float angle = (float) modulo/8192*360;                      
                  Serial.print(servo_dist);
                  Serial.print(" ");
                  Serial.print(theta*INC_THETA);
                  Serial.print(" ");
                  Serial.print(angle);
                  Serial.print(" ");
                  Serial.print(z*INC_Z);
                  Serial.print(" ");
                  Serial.print(pressure_abs);
                  Serial.print(" ");
                  Serial.print(temperature_c);
                  Serial.print(" ");
                  Serial.println(DeltaTime);                  
                  DeltaTime = millis()-CurTime;
                  
                }while(DeltaTime<3000);
                servo.write(0);
                delay(1000);
                servo_dist = 0;
                value_rev_angle = 0;
                value_rev_dist = 0;
                }
                value_rev_angle = INC_THETA; // incremetal
              }
                float ANG_deltaStep=(float)1/2/1600*1000*1000*60/ANG_RPM; // micro seconds
                stepRotate1(CLKPin_1,dirPin_1,enaPin_1,ANG_deltaStep,-360);//각도바꿔주고
                delay(3000);
                value_rev_dist = INC_Z;  // incremetal
            }
          }

  }
}


void stepRotate1(int CLK1, int DIR1, int ENA1, float ANG__deltaStep, float ANG){

  digitalWrite(ENA1,LOW); // step motor enable
  
    if(ANG>0)
      {
        digitalWrite(DIR1,HIGH);// CW
        digitalWrite(ENA1,LOW);

        for(int j=0; j<1;j++)
         {
        
            for(int i = 0 ; i < ANG/(360.0/1600.0); i++)
            {
              digitalWrite(CLK1, HIGH);  
              delayMicroseconds(ANG__deltaStep);             
              digitalWrite(CLK1, LOW);  
              delayMicroseconds(ANG__deltaStep);   
            }
         }
      }

    else if(ANG==0)
      {
        digitalWrite(ENA1,LOW); // enable (holding)
      }

    else
      {
        digitalWrite(DIR1,LOW);  
        ANG= -ANG;
        digitalWrite(ENA1,LOW);
 
       for(int j=0; j<1;j++)
         {

            for(int i =0; i < ANG/(360.0/1600.0); i++)
              {
                digitalWrite(CLK1, HIGH);  
                delayMicroseconds(ANG__deltaStep);             
                digitalWrite(CLK1, LOW);  
                delayMicroseconds(ANG__deltaStep);   
              }
          }
         digitalWrite(DIR1,HIGH);
      } 
}

void stepRotate2(int CLK2, int DIR2, int ENA2, float DIST__deltaStep, int DIST){

  digitalWrite(ENA2,LOW); // step motor enable
  
    if(DIST>0)
      {
        digitalWrite(DIR2,HIGH);// CW
        digitalWrite(ENA2,LOW);

        for(int j=0; j<1;j++)
         {
        
            for(int i = 0 ; i < (45/(360.0/1600.0))*DIST; i++)
            {
              digitalWrite(CLK2, HIGH);  
              delayMicroseconds(DIST__deltaStep);             
              digitalWrite(CLK2, LOW);  
              delayMicroseconds(DIST__deltaStep);   
            }
         }
      }

    else if(DIST==0)
      {
        digitalWrite(ENA2,LOW); // enable (holding)
        digitalWrite(CLK2,HIGH);
        //delay(deltaStep*1000);
        digitalWrite(ENA2,HIGH);
      }

    else
      {
        digitalWrite(DIR2,LOW);  
        DIST= -DIST;
        digitalWrite(ENA2,LOW);
 
       for(int j=0; j<1;j++)
         {

            for(int i =0; i < (45/(360.0/1600.0))*DIST; i++)
              {
                digitalWrite(CLK2, HIGH);  
                delayMicroseconds(DIST__deltaStep);             
                digitalWrite(CLK2, LOW);  
                delayMicroseconds(DIST__deltaStep);   
              }
          }
         digitalWrite(DIR2,HIGH);
      } 
}

void poking(float poke)
{
//  Serial.println(poke);
  float poke_dist = poke + 0.5;
//  Serial.println(poke_dist);
  float poke_angle = poke_dist * 60/1.2;
//  Serial.println(poke_angle);
  servo.write(poke_angle);
}
