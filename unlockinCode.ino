#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"
#include <Stepper.h>

#define CAL 512
#define DOOR CAL*11
#define TIMEOUT 5
/**        
  Connection
  Arduino    VoiceRecognitionModule
   2   ------->     TX
   3   ------->     RX
*/
VR myVR(2,3);    // 2:RX 3:TX, you can choose your favourite pins.
int in1Pin1 = 8;
int in1Pin2 = 6;
int in1Pin3 = 9;
int in1Pin4 = 7;

int Speed = 30;
Stepper motor1(CAL, in1Pin1, in1Pin2, in1Pin3, in1Pin4); 

uint8_t records[7]; // save record
uint8_t buf[64];

int buzzer = 5;

int pwr = 12;
int gnd = 11;

#define bazinga    (0)
#define offRecord   (1) 

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf     --> command length
           len     --> number of parameters
*/
void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

/**
  @brief   Print signature, if the character is invisible, 
           print hexible value instead.
  @param   buf  -->  VR module return value when voice is recognized.
             buf[0]  -->  Group mode(FF: None Group, 0x8n: User, 0x0n:System
             buf[1]  -->  number of record which is recognized. 
             buf[2]  -->  Recognizer index(position) value of the recognized record.
             buf[3]  -->  Signature length
             buf[4]~buf[n] --> Signature
*/
void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}

void setup()
{
  /** initialize */
  myVR.begin(9600);
  
  Serial.begin(115200);
  Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");
  
  pinMode(buzzer, OUTPUT);
  pinMode(in1Pin1, OUTPUT);
  pinMode(in1Pin2, OUTPUT);
  pinMode(in1Pin3, OUTPUT);
  pinMode(in1Pin4, OUTPUT);

  pinMode(pwr, OUTPUT);
  pinMode(gnd, OUTPUT);
  digitalWrite(pwr, HIGH);
  digitalWrite(gnd, LOW);
    
  if(myVR.clear() == 0){
    Serial.println("Recognizer cleared.");
  }else{
    Serial.println("Not find VoiceRecognitionModule.");
    Serial.println("Please check connection and restart Arduino.");
    while(1);
  }
  
  if(myVR.load((uint8_t)bazinga) >= 0){
    Serial.println("onRecord loaded");
  }
  
  if(myVR.load((uint8_t)offRecord) >= 0){
    Serial.println("offRecord loaded");
  }

  //initialize motor
  motor1.setSpeed(Speed);
}

void loop()
{
  int ret;
  ret = myVR.recognize(buf, 50);
  if(ret>0){
    switch(buf[1]){
      case 0:
        for(int j=0; j<3; j++)
        {
        for(int i=3000; i>100; i=i-100){
         tone(buzzer, i);
        delay(10); }
        for(int i=100; i<3000; i=i+100){
         tone(buzzer, i);
        delay(10); }
        }
        noTone(buzzer);
        motor1.step(DOOR);
        delay(TIMEOUT*1000);
        motor1.step(-DOOR);
        break;
      case 1:
        tone(buzzer, 1000);
        delay(1000);
        noTone(buzzer);
        motor1.step(DOOR);
        delay(TIMEOUT*1000);
        motor1.step(-DOOR);
        break;
      default:
        Serial.println("Record function undefined");
        break;
    }
    /** voice recognized */
    printVR(buf);
  }
}



