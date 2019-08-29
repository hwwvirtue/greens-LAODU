#include "xihuaROUND.h"
#include "math.h"
ActPoint presentpoint;
float presentangle = 0;
ActPoint targetpoint;
float targetangle = 0;
ActLine2 presentline;
ActLine2 targetline;
float v;
ActPoint pointstart;
ActPoint pointend;

void VelCrl(unsigned char ElmoNum,int vel)
{
  }

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  //杨迁传来的三个值
  float GetX=0,GetY=0,p=0;
  /*GetX=Serial.read();
  GetY=Serial.read();*/
  p=Serial.read();
  presentpoint.x = GetX;
  presentpoint.y = GetY;
  presentangle = p;
  
  //圆心坐标和半径
  float x=0,y=0,R=0;
  float TwopointDis=0;
  TwopointDis=sqrt(pow(GetX-x,2)+pow(GetY-y,2))-R;//走圆之前的位置点到圆点距离
  if(TwopointDis != R)//判断初始点在不在圆上
  {
    targetpoint.x=x;
    targetpoint.y=y;
    float ang1=0;
    pointstart=presentpoint;
    pointend=targetpoint;
    ang1=CcltLineAngle(pointstart, pointend);
    
    targetangle=ang1-90;//目标点设为圆点，目的是确定移动的方向
    while(TwopointDis != R)//当移动到圆上后停止
    {
      MvByLine(presentline, targetline, v);
     }
     for(int t=0;t<=360;t++)//开始走圆
    {
     targetpoint.x=R*cos(t*CHANGE_TO_RADIAN);
     targetpoint.y=R*sin(t*CHANGE_TO_RADIAN);
     targetangle=6;
     MvByLine(presentline, targetline, v);
    }
   }
   else
   {
     for(int t=0;t<=360;t++)
    {
     targetpoint.x=R*cos(t*CHANGE_TO_RADIAN);
     targetpoint.y=R*sin(t*CHANGE_TO_RADIAN);
     targetangle=6;
     MvByLine(presentline, targetline, v);
    }
   }
}
