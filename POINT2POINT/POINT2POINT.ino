#include "POINT2POINT.h"
ActPoint presentpoint;
float presentangle = 0;
ActPoint targetpoint;
float targetangle = 0;
ActLine2 presentline;
ActLine2 targetline;
float v;

void VelCrl(unsigned char ElmoNum,int vel)
{
  }

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  float x=0,y=0,p=0;
  presentpoint.x = x;
  presentpoint.y = y;
  presentangle = p;
  
  targetpoint.x=200;
  targetpoint.y=200;
  targetangle=6;
  float *q,distance,angleError;
  q = MvByLine(presentline, targetline,v);
  angleError = *q;
  q++;
  distance = *q;
}
