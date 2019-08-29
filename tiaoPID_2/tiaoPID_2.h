#ifndef _tiaoPID_2_H
#define _tiaoPID_2_H
#define Pi 3.141593f
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
static float PID_speed_Serial[2];

float *motorCMD_Back(unsigned char motorNUM);


void minimum_Turn(float angle);
void forward_Turn(float angle,float gospeed);
void back_Turn(float angle,float gospeed);
float DistancePid(float distance);
float Distance_Arc_Pid(float distance);
float AnglePid(float valueSet,float valueNow);
uint8_t straightLine(float A1,float b1,float C1,uint8_t dir,float setSpeed);
void closeRound(float x,float y,float R,float clock,float backspeed,int roundsize);
int GetPosX();
int GetPosY();
int GetAngle();


#endif
