#include"PID.h"
#include"Arduino.h"


/**
* @brief 距离PID，返回值即为角度差值
* @param valueSet：距离设定值
* @param valueNow：当前距离值
* @retval none
* @attention
*/
void Distance_PidPara(float fKp,float fKi,float fKd)
{
  Dispid_Para.dKp=fKp;
  Dispid_Para.dKi=fKi;
  Dispid_Para.dKd=fKd;
}
float DistancePid(float valueSet,float valueNow)//valueNow一直是0
{
  while(millis()%10 == 0)
  {
    float err=0;
    float valueOut=0;
    static float errLast=0;
    static float iTerm=0;
    err=valueSet-valueNow;
  
    iTerm+=(Dispid_Para.dKi*err);
    if(err > 1 || err < -1)
    {
    if(iTerm > 10) iTerm=10;
    if(iTerm < -10) iTerm=-10;
    }
  
    valueOut=(Dispid_Para.dKp*err)+iTerm+(Dispid_Para.dKd*(err-errLast));
    if(valueOut > outMax2) valueOut=outMax2;
    if(valueOut < outMin2) valueOut=outMin2;
    errLast=err;
    return valueOut;
  }
}




/**
* @brief 角度PID，返回值即为速度的增量，此增量使用两个前轮
* @param valueSet：角度设定值
* @param valueNow：当前角度值
* @retval none
* @attention 
*/
void Angle_PidPara(float fKp,float fKi,float fKd)
{
   Angpid_Para.aKp=fKp;
   Angpid_Para.aKi=fKi;
   Angpid_Para.aKd=fKd;
   
}
float AnglePid(float valueSet,float valueNow)
{
  while(millis()%10 == 0)
  {
    float err=0;
    float valueOut=0;
    static float errLast=0;
    static float iTerm=0;
    err=valueSet-valueNow;//目标角度和当前角度差值
    if(err > 180)//让err处于[-180,180]的范围
    {
      err=err-360;
    }
    else if(err < -180)
    {
      err=360+err;
    }
  
    iTerm+=(Angpid_Para.aKi*err);
    if(iTerm > outMax) iTerm=outMax;//让iTerm处于[outMin,outMax]的范围
    if(iTerm < outMin) iTerm=outMin;
  
    valueOut=(Angpid_Para.aKp*err)+iTerm+(Angpid_Para.aKd*(err-errLast));
    if(valueOut > outMax) valueOut=outMax;//让valueOut处于[outMin，outMax]的范围
    if(valueOut < outMin) valueOut=outMin;
    errLast=err;
    return valueOut;
  }
}
