#ifndef _cai_H
#define _cai_H
#define Pi       3.141593f
typedef unsigned     char uint8_t;
typedef float fp32;



//与杨迁传输部分结构体
static struct getpos_LR { //高八低八转16
  int X_L;
  int X_R;
  int Y_L;
  int Y_R;
  int ANG_L;
  int ANG_R;
} getPos_LR;

static struct getpos { //用short型因为DUE是32位处理器,它的int型为4个字节,32位.而一般的arduino版（八位）int型为2个字节.
  short X = 0;
  short Y = 0;
  short ANG = 0;
} getPos;

//PID输出速度值
static float PiD_speed_Serial[2];


//直线一般式的三个系数
static struct straightLinepara {
  float A1 = 0;
  float _B1 = 0;
  float C1 = 0;
} straightLinePara;

//距离PID赋值结构体
static struct Distance_Pidass {
  float dKP = 0;
  float dKI = 0;
  float dKD = 0;
} Distance_PidAss;

//角度PID赋值结构体
static struct Angle_Pidass {
  float aKP = 0;
  float aKI = 0;
  float aKD = 0;
} Angle_PidAss;


//float *motorCMD(unsigned char motorNUM, int speedBase, int Vel);

uint8_t straightLine(Distance_Pidass Distance_PidAss1 , Angle_Pidass Angle_PidAss1  , getpos getPos1 , straightLinepara straightLinePara1 , uint8_t dir, float setSpeed);

void closeRound(Distance_Pidass Distance_PidAss2 , Angle_Pidass Angle_PidAss4 , getpos getPos2, float x, float y, float R, float clock, float v);

void back_Turn(Angle_Pidass Angle_PidAss4, fp32 getAnglenow3, float angle, float gospeed);
#endif
