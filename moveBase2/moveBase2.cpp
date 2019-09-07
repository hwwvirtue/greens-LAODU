#include"POINT2POINT.h"
#include"PID.h"
#include"moveBase2.h"
#include"math.h"
#include "variant.h"

#define  PI 3.1415926
ActPoint initpoint;     //点到点算法 初始点结构体初始化
float initangle = 0;
ActPoint targetpoint;		//目标点结构体初始化
float targetangle = 0;
ActLine2 presentline;		//初始直线结构体初始化
ActLine2 targetline;		//目标直线结构体初始化


PidTypeDef PID_distance;	//PID结构体初始化
PidTypeDef PID_angle;
PidTypeDef PID_DisArc;		//角度对距离增量PID

fp32 PID_param_dis[3];		//距离PID变量参数结构体初始化
fp32 PID_param_angle[3];//角度PID变量参数结构体初始化
fp32 PID_param_DisArc[3];//距离转角度PID变量参数结构体初始化


/**
    @name     VelTwoCtl   POINT2POINT电机控制函数
    @brief    正数朝前
    @param    vellBase:朝前速度；
              vellErr:符号为正，朝左拐
    @retval
*/
static void elDecomposeCtl(int vellBase, int vellErr)
{
  //1号电机速度赋值函数
  VelCrl(MOTOR_ONE, vellBase - vellErr);

  //2号电机速度赋值函数
  VelCrl(MOTOR_TWO, vellBase + vellErr);
}


/**
    @name     VelTwoCtl   PID电机控制函数
    @brief    正数朝前
    @param    speedBase:基础前进速度；
              Vel:差值为正则为右转;
    @retval
*/
float *motorCMD(unsigned char motorNUM, int speedBase, int Vel)
{
  if (motorNUM == 1)
    PID_speed_Serial[0] = speedBase - Vel;
  if (motorNUM == 2)
    PID_speed_Serial[1] = speedBase + Vel;
//                                            Serial.print("Vel=");
//                                            Serial.println(Vel);
  return PID_speed_Serial;
}
float motor1()
{
  return PID_speed_Serial[0];
  
  }
float motor2()
{
    return PID_speed_Serial[1];
  
  }

/**
    @name     VelTwoCtl   PID电机控制函数                          倒退
    @brief    正数朝前
    @param    speedBase:基础前进速度；
              Vel:差值为正则为右转;
    @retval
*/
float *motorCMD_Back(unsigned char motorNUM, int speedBase, int Vel)
{
  if (motorNUM == 2)
    PID_speed_Serial[0] = -(speedBase + Vel);
  if (motorNUM == 1)
    PID_speed_Serial[1] = -(speedBase - Vel);
  return PID_speed_Serial;
}



float DistancePid(float distance)//先不用
{
    fp32 valueOut;
    PID_param_dis[0]=55;
    PID_param_dis[1]=0;
    PID_param_dis[2]=0;

    PID_Init(&PID_distance,PID_POSITION,PID_param_dis,9000.0f,1000.0f);//PID初始化
    valueOut=PID_Calc(&PID_distance,distance,0);


    return valueOut;
}

/**
* @brief 距离对角度的增量PID
* @note  PID类型：位置型PID
* @param valueSet：距离设定值
* @param valueNow：当前距离值
* @retval 电机前进基础速度
* @attention
*/
float Distance_Arc_Pid(float distance)
{
    float valueOut;
    PID_Init(&PID_DisArc,PID_POSITION,PID_param_DisArc,90.0f,10.0f);//位置型PID初始化
    valueOut=PID_Calc(&PID_DisArc,distance,0);

    return valueOut;
}

/**
* @brief 角度PID
* @note PID类型：位置型PID
* @param valueSet：角度设定值
* @param valueNow：当前角度值
* @retval 电机速度差值
* @attention
*/
float AnglePid(float valueSet,float valueNow)
{
    float err=0;
    float valueOut=0;
    err=valueSet-valueNow;
    //角度突变处理
    if(err > 180)
    {
        err=err-360;
    }
    else if(err < -180)
    {
        err=360+err;
    }
    PID_Init(&PID_angle,PID_POSITION,PID_param_angle,9000.0f,20.0f);//PID初始化
    valueOut=PID_Calc(&PID_angle,err,0);  //PID计算转弯差值

    return valueOut;
}

/**
   @brief  PID 最小转弯
   @note		无前进速度 仅为绕自身旋转中心旋转
   @param  angle：给定角度,为正左转，为负右转
   @param  gospeed：基础速度
   @retval None
*/
//小动作 原地旋转
void minimum_Turn(float angle)
{
  float getAngle = 0.0f;
  float speed = 0.0f;
  PID_param_angle[0] = 55; // 转速10000除以转弯最大角度180 = 55
  PID_param_angle[1] = 0;
  PID_param_angle[2] = 0;
  getAngle = GetAngle();
  speed = AnglePid(angle, getAngle);	//根据角度PID算出转向的差速
  *motorCMD(1, 0, speed);
  *motorCMD(2, 0, speed);
}

//小动作 向前旋转
void forward_Turn(float angle, float gospeed)
{
  float getAngle = 0.0f;
  float speed = 0.0f;


  PID_param_angle[0] = 30;
  PID_param_angle[1] = 0;
  PID_param_angle[2] = 0;

  getAngle = GetAngle();
  speed = AnglePid(angle, getAngle);	//根据角度PID算出转向的差速
  if (gospeed < 0)
  {
    *motorCMD(1, gospeed, speed);
    *motorCMD(2, gospeed, speed);
  }
  else
  {
    *motorCMD(1, gospeed, speed);
    *motorCMD(2, gospeed, speed);
  }
}


/**
 * @brief  PID 后退转弯
 * @note
 * @param  angle：给定角度,为正左转，为负右转
 * @param  gospeed：基础速度
 * @retval None
 */
//小动作 倒行转弯
void back_Turn(float angle,float gospeed)
{
    float getAngle=0.0f;
    float speed=0.0f;
    PID_param_angle[0]=50;
    PID_param_angle[1]=0;
    PID_param_angle[2]=0;

    getAngle=GetAngle();
    speed = AnglePid(angle,getAngle);  //根据角度PID算出转向的差速
    *motorCMD_Back(1, gospeed, speed);
    *motorCMD_Back(2, gospeed, speed);
}








/**
    @brief  新底盘直线闭环
    @note	Ax1+By1+C1=0 直线方程一般式
    @note	可由A1，B1，C1设置直线方程后，沿目标直线方向行走
    @note	前进速度可由距离PID调整  问题是两个PID不太好调  待调试后解决
    @note angleAdd为正值

    @param A1
    @param B1
    @param C1
    @param dir:为0 往上或右走，为1 往下或往左走
    @param setSpeed：速度
    @retval return_value 0为 未到达目标直线  1为 已到达目标直线距离范围内

    @note 大动作 在目标直线轨道上行驶 可通过坐标判断在直线的哪个位置
	  @note 用到了距离-角度PID 来接近目标轨道 角度PID来保证沿着直线轨道行驶
    @note 用到了两种小动作 离目标直线距离远时 用前进转弯来接近
    @note 接近到达目标直线时原地旋转调整到目标方向 在用前进转弯来保持行驶直线
*/
uint8_t straightLine(float A1, float b1, float C1, uint8_t dir, float setSpeed)
{
  fp32 setAngle = 0;
  fp32 angleAdd = 0;
  int	return_value = 0;
  fp32 getAngleNow = GetAngle();
  fp32 getX = GetPosX();
  fp32 getY = GetPosY();
  float distance = ((A1 * getX) + (b1 * getY) + C1) / sqrt((A1 * A1) + (b1 * b1)); //当前点到目标直线距离

  //		PID_param_dis[0]=0.5f;
  //		PID_param_dis[0]=0;
  //		PID_param_dis[0]=0;

  PID_param_DisArc[0] = 0.15;
  PID_param_DisArc[1] = 0;
  PID_param_DisArc[2] = 0;


  angleAdd = Distance_Arc_Pid(distance);
  //离直线35以内时表示到达直线
//  if ((distance < 150) && (distance > -150))	//到达直线位置时用最小半径旋转调整为目标角度
//  {
//    angleAdd = 0;
//    return_value = 1;
//    forward_Turn(setAngle, setSpeed);
//  }
//  else		//未到达目标直线距离范围内时，用前进转弯来接近目标直线
//  {
    if ((b1 > -0.005f) && (b1 < 0.005f))
    {
      if (!dir)
      {
        setAngle = 0;							//目标角度为水平直线 方向由dir确定 0为右 1为左
        forward_Turn(setAngle + angleAdd, setSpeed);
      }
      else
      {
        if (A1 > 0)
        {
          setAngle = -180;			//目标角度为水平直线 方向由dir确定 0为上 1为下
          forward_Turn(setAngle - angleAdd, setSpeed); //角度PID转向目标角度
        }
        else
        {

          setAngle = 180;
          forward_Turn(setAngle + angleAdd, setSpeed);
        }
      }
    }
    else
    {
      if (!dir)
      {
        setAngle = (atan(-A1 / b1) * 180 / PI) - 90;
        forward_Turn(setAngle - angleAdd, setSpeed);
      }
      else
      {
        setAngle = (atan(-A1 / b1) * 180 / PI) + 90;
        forward_Turn(setAngle + angleAdd, setSpeed);
      }

    }
    return_value = 0;
  //}
  return return_value;
}

/**
    @brief  底盘圆环路径
    @note    直线斜率为负  存疑
    @note   思路为 直线距离增大角度差值 到达圆环轨迹后 直线-角度PID不再计算 仅计算角度PID
    @note   PID期望角度始终为当前行进位置指向圆心的方向
    @note   直线——角度PID，角度——角度参数调整完成后，可添加直线——距离PID 调整前进速度 使得底盘更快进入目标轨道
    @param x：圆心x坐标
    @param y：圆心y坐标
    @param R：半径
    @param clock:为1 顺时针，为2 逆时针
    @param v：速度
    @param roundsize:圆环尺寸 0内圈 1中圈 2大圈
    @retval None
*/
void closeRound(float x, float y, float R, float clock, float forwardspeed, int roundsize)
{
  float target_Distance = 0;
  float k = 0;
  float setangle = 0, Agl = 0, frontspeed = 0;
  target_Distance = sqrt(pow(GetPosX() - x, 2) + pow(GetPosY() - y, 2)) - R;
  k = (GetPosX() - x) / (y - GetPosY()); //前进直线斜率
  if (roundsize == 2)
  {
    PID_param_angle[0] = 30.0f;
    PID_param_angle[1] = 0.0f;
    PID_param_angle[2] = 0.0f;

    PID_param_DisArc[0] = 0.08;
    PID_param_DisArc[1] = 0;
    PID_param_DisArc[2] = 0;
  }
  if (roundsize == 1)
  {
    PID_param_angle[0] = 45.0f;
    PID_param_angle[1] = 0.0f;
    PID_param_angle[2] = 0.0f;

    PID_param_DisArc[0] = 0.05;
    PID_param_DisArc[1] = 0;
    PID_param_DisArc[2] = 0;
  }
  if (roundsize == 0)
  {
    PID_param_angle[0] = 60.0f;
    PID_param_angle[1] = 0.0f;
    PID_param_angle[2] = 0.0f;

    PID_param_DisArc[0] = 0.05;
    PID_param_DisArc[1] = 0;
    PID_param_DisArc[2] = 0;
  }
  //  顺1逆2
  if (clock == 1)
  {
    if (GetPosY() > y)
      Agl = -90 + atan(k) * 180 / PI;
    else if (GetPosY() < y)
      Agl = 90 + atan(k) * 180 / PI;
    else if (GetPosY() == y && GetPosX() >= x)
      Agl = 180;
    else if (GetPosY() == y && GetPosX() < x)
      Agl = 0;
    setangle = Agl - Distance_Arc_Pid(target_Distance); //距离赋值给角度的增量 Agl为角度转换常量
          //    Serial.print(Agl);
          //     Serial.print(setangle);
    frontspeed = AnglePid(setangle, GetAngle()); //角度PID计算两轮速度差值
   
  }
  else if (clock == 2)
  {
    if (GetPosY() > y)
      Agl = 90 + atan(k) * 180 / PI;
    else if (GetPosY() < y)
      Agl = -90 + atan(k) * 180 / PI;
    else if (GetPosY() == y && GetPosX() >= x)
      Agl = 0;
    else if (GetPosY() == y && GetPosX() < x)
      Agl = 180;
    setangle = Agl + Distance_Arc_Pid(target_Distance); //距离赋值给角度的增量
    frontspeed = AnglePid(setangle, GetAngle()); //角度PID计算两轮速度差值
  }
  *motorCMD(1, forwardspeed, frontspeed);
  *motorCMD(2, forwardspeed, frontspeed);
}
