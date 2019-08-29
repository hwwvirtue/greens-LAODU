#include "distancePID.h"
float valueSet=0,valueNow=0;
void setup()
{
  Serial.begin(9600);
}

void loop()
{
  float a=0,b=0,c=0;
  Distance_PidPara(a,b,c);
  float speedAdd;
  speedAdd=DistancePid(valueSet,valueNow);
}
