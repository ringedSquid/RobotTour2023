#include <PIDController.h>
#include "hmap.h"


int ticks = 0;
int oldticks = 0;
int count = 0;
double a,b,c,d,e,f,g,h,i,j;
double oldt = 0;
double newt = 0;
void countUp() {
  ticks++;
}

void setup()    {
  Serial.begin(115200);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  pinMode(M4, OUTPUT);
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);
  pinMode(C3, INPUT);
  pinMode(C4, INPUT);
  

  attachInterrupt(digitalPinToInterrupt(C3), countUp, RISING);
  oldt = millis();
  analogWrite(M1, 0);
  analogWrite(M2, 0);
  

  }

void loop() {
  oldticks = ticks;
  oldt = millis();
  //Period when ticks means something, so insert code below
  delay(5);
  Serial.println(((ticks - oldticks)/(millis() - oldt))*6000/12);



  
  
}
