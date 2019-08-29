#include "anglePID.h"
float valueSet=0,valueNow=0;
void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  float a=0,b=0,c=0;
  Angle_PidPara(a,b,c);
  float angleAdd;
  angleAdd=AnglePid(valueSet,valueNow);
}
