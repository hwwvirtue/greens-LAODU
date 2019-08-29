#include "four_points_ROUND.h"
//点到点部分变量
ActPoint presentpoint;
float presentangle = 0;
ActPoint targetpoint;
float targetangle = 0;
ActLine2 presentline;
ActLine2 targetline;
float speed;
int speed_Serial[2] = {0};

//与杨迁传输部分变量
unsigned char Get_buf[5];
unsigned char counter=0;
unsigned char sign1=0;
int GetPos[3] = {0};

//与乐天传输部分变量
unsigned char Pri_buf[6];


void VelCrl(unsigned char ElmoNum,int vel)
{
  if(ElmoNum == MOTOR_ONE)
  speed_Serial[0]=vel;
  else
  speed_Serial[1]=vel;
}

void setup()
{
  Serial2.begin(9600);
  Serial3.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  //与杨迁通信部分
  int sum = 0x55;//帧尾数值
  if (sign1 == 1)
  {
    sign1 = 0;
    if (sum == Get_buf[4] )     //检查帧头，帧尾
    {
      GetPos[0] = Get_buf[1];
      GetPos[1] = Get_buf[2];
      GetPos[2] = Get_buf[3];
      Serial2.print("x:");
      Serial2.print( GetPos[0]);
      Serial2.print(",y:");
      Serial2.print( GetPos[1]);
      Serial2.print(",a:");
      Serial2.println( GetPos[2]);
    }
  }
  while (Serial2.available())
  {
    int readedchar = Serial2.read();
    Serial.println(readedchar);
    Get_buf[counter] = (unsigned char)readedchar;
    if (counter == 0 && Get_buf[0] != 0xAA) return; // 检查帧头
    counter++;
    if (counter == 4)             //接收到数据
    {
      counter = 0;               //重新赋值，准备下一帧数据的接收
      sign1 = 1;
    }
  }





  
  //路径部分
  float R=1700;
  int steps=1,dir=0;  //dir为0 顺时针；为1 逆时针   steps为转一圈的四个弧的步骤
  if(dir == 0)
  {
    if(steps == 1)
    {
     targetpoint.x=GetPos[0]+R;
     targetpoint.y=GetPos[1]+R;
     targetangle=GetPos[2]+90;
     steps++;
     }
    else if(steps == 2)
    {
     targetpoint.x=GetPos[0]+R;
     targetpoint.y=GetPos[1]-R;
     targetangle=GetPos[2]+90; 
     steps++;
     }
    else if(steps == 3)
    {
     targetpoint.x=GetPos[0]-R;
     targetpoint.y=GetPos[1]-R;
     targetangle=GetPos[2]-90; 
     steps++;
     }
     else if(steps == 4)
    {
     targetpoint.x=GetPos[0]-R;
     targetpoint.y=GetPos[1]+R;
     targetangle=GetPos[2]-90; 
     steps=1;
     }
  }
  else
  {
    if(steps == 1)
    {
     targetpoint.x=GetPos[0]+R;
     targetpoint.y=GetPos[1]-R;
     targetangle=GetPos[2]-90; 
     steps++;
     }
    else if(steps == 2)
    {
     targetpoint.x=GetPos[0]+R;
     targetpoint.y=GetPos[1]+R;
     targetangle=GetPos[2]-90; 
     steps++;
     }
    else if(steps == 3)
    {
     targetpoint.x=GetPos[0]-R;
     targetpoint.y=GetPos[1]+R;
     targetangle=GetPos[2]+90; 
     steps++;
     }
     else if(steps == 4)
    {
     targetpoint.x=GetPos[0]-R;
     targetpoint.y=GetPos[1]-R;
     targetangle=GetPos[2]+90;
     steps=1;
     }
  }
  float *q,distance,angleError;
  q = MvByLine(presentline, targetline,speed);
  angleError = *q;
  q++;
  distance = *q;




 

  //与乐天通信部分
  if(Serial3.available())
  {
    unsigned char left1,right1,left2,right2;
    speed_Serial[0]=256;
    speed_Serial[1]=257;
    left1=speed_Serial[0] >>8;
    right1=speed_Serial[0] &0xFF;
    left2=speed_Serial[1] >>8;
    right2=speed_Serial[1] &0xFF;
    Serial3.write(0xAA);
    delay(10);
    Serial3.write(left1);
    delay(10);
    Serial3.write(right1);
    delay(10);
    Serial3.write(left2);
    delay(10);
    Serial3.write(right2);
    delay(10);
    Serial3.write(0x55);
    delay(10);
    
  }
}
