#include"POINT2POINT.h"
#include"PID.h"
#include"tiaoPID_2.h"

int Point_speed_Serial[2] = {0};
//与杨迁传输部分变量
int Get_buf[8];
unsigned char counter = 0;
unsigned char sign = 0;
getpos getPos2;
getpos_LR getPos_LR1;

//路径部分
//直线
straightLinepara straightLinePara2;
uint8_t Dir;
float setspeed;
//圆

//PID部分电机函数
unsigned char motornum;
int speedbase;
int vel;
int LEFT_v, RIGHT_v;

//与乐天传输部分变量
unsigned char Pri_buf[6];



void VelCrl(unsigned char ElmoNum, int vel)
{
  if (ElmoNum == MOTOR_ONE)
    Point_speed_Serial[0] = vel;
  else
    Point_speed_Serial[1] = vel;
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
  if (sign == 1)
  {
    sign = 0;
    if ( sum == Get_buf[7] )     //检查帧头，帧尾
    {
      getPos_LR1.X_L = Get_buf[1];
      getPos_LR1.X_R = Get_buf[2];
      getPos2.X = getPos_LR1.X_L;
      getPos2.X <<= 8;
      getPos2.X |= getPos_LR1.X_R;

      getPos_LR1.Y_L = Get_buf[3];
      getPos_LR1.Y_R = Get_buf[4];
      getPos2.Y = getPos_LR1.Y_L;
      getPos2.Y <<= 8;
      getPos2.Y |= getPos_LR1.Y_R;

      getPos_LR1.ANG_L = Get_buf[5];
      getPos_LR1.ANG_R = Get_buf[6];
      getPos2.ANG = getPos_LR1.ANG_L;
      getPos2.ANG <<= 8;
      getPos2.ANG |= getPos_LR1.ANG_R;

      Serial.print("x:");
      Serial.println(getPos2.X);
      Serial.print("y:");
      Serial.println(getPos2.Y);
      Serial.print("ang:");
      Serial.println(getPos2.ANG);
    }
  }
  while (Serial2.available())
  {
    int readedchar = Serial2.read();
    //Serial.println(readedchar);
    Get_buf[counter] = readedchar;
    if (counter == 0 && Get_buf[0] != 0xAA) break; // 检查帧头
    else
    {
      counter++;
      if (counter == 8)             //接收到数据
      {
        counter = 0;               //重新赋值，准备下一帧数据的接收
        sign = 1;
      }
    }
  }



  
  //路径部分
  closeRound(getPos2, 0, 2300, 450, 1, 1000);
  float *OutPos_v;
  OutPos_v = motorCMD(motornum, speedbase, vel);
  LEFT_v = *OutPos_v;
  OutPos_v += 1;
  RIGHT_v = *OutPos_v;




  //与乐天通信部分
  if (Serial3.available())
  {
    unsigned char left1, right1, left2, right2;
    left1 = LEFT_v >> 8;
    right1 = LEFT_v & 0xFF;
    left2 = RIGHT_v >> 8;
    right2 = RIGHT_v & 0xFF;
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
