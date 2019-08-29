#ifndef _tiaoPID_2_H
#define _tiaoPID_2_H
#define Pi       3.141593f
typedef unsigned     char uint8_t;
typedef float fp32;



//与杨迁传输部分结构体
static struct getpos{
  int X=0;
  int Y=0;
  int ANG=0;
}getPos;
static struct getpos_LR{  //高八位低八位转化中间量
  int X_L=0;
  int X_R=0;
  int Y_L=0;
  int Y_R=0;
  int ANG_L=0;
  int ANG_R=0;
}getPos_LR;


//直线一般式的三个系数
static struct straightLinepara{
  float A1=0;
  float _B1=0;
  float C1=0;
}straightLinePara;



//float GetPosX(void);
//float GetPosY(void);
//float GetAngle(void);
//void Distance_PidPara(float fKp,float fKi,float fKd);
//void Angle_PidPara(float fKp,float fKi,float fKd);
//void minimum_Turn(float angle);
//void forward_Turn(float angle,float gospeed);
float *motorCMD(unsigned char motorNUM,int speedBase,int Vel);
uint8_t straightLine(getpos getPos1,straightLinepara straightLinePara1,uint8_t dir,float setSpeed);
void closeRound(getpos getPos1,float x,float y,float R,float clock,float v);
#endif
