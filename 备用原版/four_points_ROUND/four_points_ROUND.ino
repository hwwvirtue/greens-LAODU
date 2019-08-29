#include "four_points_ROUND.h"
ActPoint presentpoint;
float presentangle = 0;
ActPoint targetpoint;
float targetangle = 0;
ActLine2 presentline;
ActLine2 targetline;
float speed;

void VelCrl(unsigned char ElmoNum,int vel)
{
  }

void setup()
{
  Serial1.begin(9600);
  Serial2.begin(9600);
}

void loop()
{
  float x=0,y=0,p=0;
  x=Serial.read();
  y=Serial.read();
  p=Serial.read();
  presentpoint.x = x;
  presentpoint.y = y;
  presentangle = p;

  float R=1700;
  int steps=1, clock=0;//dir为0 顺时针；为1 逆时针   steps为转一圈的四个弧的步骤
  if(clock == 0)
  {
    if(steps == 1)
    {
     targetpoint.x=x+R;
     targetpoint.y=y+R;
     targetangle=p+90; 
     steps++;
     }
    else if(steps == 2)
    {
     targetpoint.x=x+R;
     targetpoint.y=y-R;
     targetangle=p+90; 
     steps++;
     }
    else if(steps == 3)
    {
     targetpoint.x=x-R;
     targetpoint.y=y-R;
     targetangle=p-90; 
     steps++;
     }
     else if(steps == 4)
    {
     targetpoint.x=x-R;
     targetpoint.y=y+R;
     targetangle=p-90; 
     steps=1;
     }
  }
  else
  {
    if(steps == 1)
    {
     targetpoint.x=x+R;
     targetpoint.y=y-R;
     targetangle=p-90; 
     steps++;
     }
    else if(steps == 2)
    {
     targetpoint.x=x+R;
     targetpoint.y=y+R;
     targetangle=p-90; 
     steps++;
     }
    else if(steps == 3)
    {
     targetpoint.x=x-R;
     targetpoint.y=y+R;
     targetangle=p+90; 
     steps++;
     }
     else if(steps == 4)
    {
     targetpoint.x=x-R;
     targetpoint.y=y-R;
     targetangle=p+90; 
     steps=1;
     }
  }
  float *q,distance,angleError;
  q = MvByLine(presentline, targetline,speed);
  angleError = *q;
  q++;
  distance = *q;
}
