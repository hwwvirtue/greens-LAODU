#include"POINT2POINT.h"
#include"PID.h"
#include"cai.h"
#include"math.h"




/**
  * @name     VelTwoCtl   POINT2POINT电机控制函数
  * @brief    正数朝前
  * @param    vellBase:朝前速度；
              vellErr:符号为正，朝左拐
  * @retval   
  */
static void VelDecomposeCtl(int vellBase,int vellErr)
{
  //1号电机速度赋值函数
  VelCrl(MOTOR_ONE, vellBase - vellErr);
  
  //2号电机速度赋值函数
  VelCrl(MOTOR_TWO, vellBase + vellErr);        
}



/**
  * @name     VelTwoCtl   PID电机控制函数 
  * @brief    正数朝前
  * @param    speedBase:基础前进速度；
  *           Vel:差值为正则为右转;
  * @retval
  */
float *motorCMD(unsigned char motorNUM,int speedBase,int Vel)
{
  if(motorNUM=='1')
  PiD_speed_Serial[0]=speedBase-Vel;
  if(motorNUM=='2')
  PiD_speed_Serial[1]=speedBase+Vel;
  return PiD_speed_Serial;
}





/**
  * @brief  PID 最小转弯  其中的PID值为固定值，不需要再外部赋值
  * @note   无前进速度 仅为绕自身旋转中心旋转
  * @param  angle：给定角度,为正左转，为负右转
  * @param  gospeed：基础速度
  * @retval None
  */
void minimum_Turn(fp32 getAnglenow1,float angle)
{
  float getAngle=0.0f;
  float A_speed=0.0f;//角度PID调出的差速
  float AKp=55,AKi=0,AKd=0;// 转速10000除以转弯最大角度180 = 55
  Angle_PidPara(AKp,AKi,AKd);
  
  getAngle=getAnglenow1;
  A_speed = AnglePid(angle,getAngle); //根据角度PID算出转向的差速
  *motorCMD(1,0,A_speed);
  *motorCMD(2,0,A_speed);
}


/**
  * @brief  PID 前进转弯  其中的PID值不是固定值，需要再外部赋值
  * @note  
  * @param  angle：给定角度,为正左转，为负右转
  * @param  gospeed：基础速度
  * @retval None
  */
void forward_Turn(Angle_Pidass Angle_PidAss2,fp32 getAnglenow2,float angle,float gospeed)
{
  float getAngle=0.0f;
  float A_speed=0.0f;
  float AKp=Angle_PidAss2.aKP;
  float AKi=Angle_PidAss2.aKI;
  float AKd=Angle_PidAss2.aKD;
  Angle_PidPara(AKp,AKi,AKd);

  getAngle=getAnglenow2;
  A_speed = AnglePid(angle,getAngle); //根据角度PID算出转向的差速
  *motorCMD(1,gospeed,A_speed);
  *motorCMD(2,gospeed,A_speed);
}




 /**
  * @brief  PID 后退转弯  其中的PID值不是固定值，需要再外部赋值
  * @note
  * @param  angle：给定角度,为正左转，为负右转
  * @param  gospeed：基础速度
  * @retval None
  */
void back_Turn(Angle_Pidass Angle_PidAss4,fp32 getAnglenow3,float angle,float gospeed)
{
  float getAngle=0.0f;
  float A_speed=0.0f;
//  float AKp=gospeed/180,AKi=0,AKd=0;
  
  float AKp=Angle_PidAss4.aKP;
  float AKi=Angle_PidAss4.aKI;
  float AKd=Angle_PidAss4.aKD;
  Angle_PidPara(AKp,AKi,AKd);
  getAngle=getAnglenow3;
  A_speed = AnglePid(angle,getAngle); //根据角度PID算出转向的差速
  *motorCMD(1,-gospeed,-A_speed);
  *motorCMD(2,-gospeed,-A_speed);
}




/**
  * @brief  新底盘直线闭环
  * @note   Ax1+By1+C1=0 直线方程一般式
  * @note   可由A1，_B1，C1设置直线方程后，沿目标直线方向行走
  * @note   前进速度可由距离PID调整  问题是两个PID不太好调  待调试后解决
  * @note   直线斜率为负值，存疑
  * @param  A1
  * @param  _B1
  * @param  C1
  * @param  dir:为0 往上或右走，为1 往下或往左走
  * @param  setSpeed：速度
  * @retval return_value 0为 未到达目标直线  1为 已到达目标直线距离范围内
  */
uint8_t straightLine(Distance_Pidass Distance_PidAss1 , Angle_Pidass Angle_PidAss1  , getpos getPos1 , straightLinepara straightLinePara1 , uint8_t dir,float setSpeed)
{
  fp32 setAngle=0;
  fp32 angleAdd=0;
  int return_value=0;
  fp32 getAngleNow=getPos1.ANG;
  fp32 getX=getPos1.X;
  fp32 getY=getPos1.Y;
  float disSet=((straightLinePara1.A1*getX)+(straightLinePara1._B1*getY)+straightLinePara1.C1)
                  /sqrt((straightLinePara1.A1*straightLinePara1.A1)+(straightLinePara1._B1*straightLinePara1._B1));//当前点到目标直线距离

  float DKp=Distance_PidAss1.dKP;
  float DKi=Distance_PidAss1.dKI;
  float DKd=Distance_PidAss1.dKD;
  Distance_PidPara(DKp,DKi,DKd);//给距离PID赋PID值

  float disNow=0;
  angleAdd=DistancePid(disSet, disNow);
  //离直线35以内时表示到达直线
  if((disSet < 35) && (disSet > -35)) //到达直线位置时用最小半径旋转调整为目标角度
  {
    angleAdd=0;
      
  //计算出直线角度，角度突变处理
    if((straightLinePara1._B1 > -0.005f) && (straightLinePara1._B1 < 0.005f))
    {
      if(!dir)
      {
        setAngle=0;//目标角度为水平直线 方向由dir确定 0为右 1为左
        minimum_Turn(getAngleNow,setAngle+angleAdd);
      }
      else
      {
        if(straightLinePara1.A1 > 0)
        {
          setAngle=-180;      //目标角度为竖直直线 方向由dir确定 0为上 1为下
          minimum_Turn(getAngleNow,setAngle-angleAdd);//角度PID转向目标角度
        }
        else
        {  
          setAngle=180;
          minimum_Turn(getAngleNow,setAngle+angleAdd);
        }
      }
    }
    else
    {
      if(!dir)
      {
        setAngle=(atan(-straightLinePara1.A1/straightLinePara1._B1)*180/Pi)-90;
        minimum_Turn(getAngleNow,setAngle-angleAdd);
      }
      else
      {
        setAngle=(atan(-straightLinePara1.A1/straightLinePara1._B1)*180/Pi)+90;
        minimum_Turn(getAngleNow,setAngle+angleAdd);
      }
    
    }
        return_value=1;
        return return_value;
  }
  else    //未到达目标直线距离范围内时，用前进转弯来接近目标直线
  {
    if((straightLinePara1._B1 > -0.005f) && (straightLinePara1._B1 < 0.005f))
    {
      if(!dir)
      {
        setAngle=0;//目标角度为竖直直线 方向由dir确定 0为右 1为左
        Angle_Pidass Angle_PidAss3=Angle_PidAss1;
        forward_Turn(Angle_PidAss3,getAngleNow,setAngle+angleAdd,setSpeed);
      }
      else
      {
        if(straightLinePara1.A1 > 0)
        {
          setAngle=-180;      //目标角度为竖直直线 方向由dir确定 0为上 1为下
          Angle_Pidass Angle_PidAss3=Angle_PidAss1;
          forward_Turn(Angle_PidAss3,getAngleNow,setAngle-angleAdd,setSpeed);//角度PiD转向目标角度
        }
        else
        {
           
          setAngle=180;
          Angle_Pidass Angle_PidAss3=Angle_PidAss1;
          forward_Turn(Angle_PidAss3,getAngleNow,setAngle+angleAdd,setSpeed);
        }
      }
    }
    else
    {
      if(!dir)
      {
        setAngle=(atan(-straightLinePara1.A1/straightLinePara1._B1)*180/Pi)-90;
        Angle_Pidass Angle_PidAss3=Angle_PidAss1;
        forward_Turn(Angle_PidAss3,getAngleNow,setAngle-angleAdd,setSpeed);
      }
      else
      {
        setAngle=(atan(-straightLinePara1.A1/straightLinePara1._B1)*180/Pi)+90;
        Angle_Pidass Angle_PidAss3=Angle_PidAss1;
        forward_Turn(Angle_PidAss3,getAngleNow,setAngle+angleAdd,setSpeed);
      }
    
    }
        return_value=0;
        return return_value; 
  }
}




/**
  * @brief  底盘圆环路径
  * @note    直线斜率为负  存疑
  * @note   思路为 直线距离增大角度差值 到达圆环轨迹后 直线-角度PID不再计算 仅计算角度PID
  * @note   PID期望角度始终为当前行进位置指向圆心的方向
  * @note   直线——角度PID，角度——角度参数调整完成后，可添加直线——距离PID 调整前进速度 使得底盘更快进入目标轨道
  * @param x：圆心x坐标
  * @param y：圆心y坐标
  * @param R：半径
  * @param clock:为0 顺时针，为1 逆时针
  * @param v：速度
  * @retval None
  */
void closeRound(Distance_Pidass Distance_PidAss2 , Angle_Pidass Angle_PidAss4 , getpos getPos2,float x,float y,float R,float clock,float v)
{
  float k=0;
  float setangle=0,Agl=0,A_speed=0;
  float disSet=sqrt(pow(getPos2.X-x,2)+pow(getPos2.Y-y,2))-R;
  k=(getPos2.X-x)/(y-getPos2.Y);    //前进直线斜率

  float AKp=Angle_PidAss4.aKP;
  float AKi=Angle_PidAss4.aKI;
  float AKd=Angle_PidAss4.aKD;
  Angle_PidPara(AKp,AKi,AKd);

  float DKp=Distance_PidAss2.dKP;
  float DKi=Distance_PidAss2.dKI;
  float DKd=Distance_PidAss2.dKD;
  Distance_PidPara(DKp,DKi,DKd);
//  顺0逆1
  if(clock==0)
  {
    if(getPos2.Y>y)
      Agl=-90+atan(k)*180/Pi;
    else if(getPos2.Y<y)
      Agl=90+atan(k)*180/Pi;
    else if(getPos2.Y==y&&getPos2.X>=x)
      Agl=180;
    else if(getPos2.Y==y&&getPos2.X<x)
      Agl=0;
    float disNow=0;
    setangle=Agl+DistancePid(disSet, disNow);//距离赋值给角度的增量
    A_speed=AnglePid(setangle,getPos2.ANG);//角度PID计算两轮速度差值
  }
  else if(clock==1)
  {
    if(getPos2.Y>y)
      Agl=90+atan(k)*180/Pi;
    else if(getPos2.Y<y)
      Agl=-90+atan(k)*180/Pi;
    else if(getPos2.Y==y&&getPos2.X>=x)
      Agl=0;
    else if(getPos2.Y==y&&getPos2.X<x)
      Agl=180;
    float disNow=0;
    setangle=Agl+DistancePid(disSet, disNow);//距离赋值给角度的增量
    A_speed=AnglePid(setangle,getPos2.ANG);//角度PID计算两轮速度差值
  }
  *motorCMD(1,1000,A_speed);//差速左轮
  *motorCMD(2,1000,A_speed);//差速右轮
}
