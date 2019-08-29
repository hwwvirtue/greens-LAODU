#include"PID.h"
/**
* @brief 距离PID，返回值即为速度的增量，此增量使用两个前轮
* @param valueSet：距离设定值
* @param valueNow：当前距离值
* @retval none
* @attention
*/
void Distance_PidPara(float fKp,float fKi,float fKd)
{
  Dispid_Para.aKp=fKp;
  Dispid_Para.aKi=fKi;
  Dispid_Para.aKd=fKd;
}
float DistancePid(float valueSet,float valueNow)
{
  float err=0;
  float valueOut=0;
  static float errLast=0;
  static float iTerm=0;
  err=valueSet-valueNow;
  
  iTerm+=(Dispid_Para.aKi*err);
  if(err > 1 || err < -1)
  {
  if(iTerm > 10) iTerm=10;
  if(iTerm < -10) iTerm=-10;
  }
  
  valueOut=(Dispid_Para.aKp*err)+iTerm+(Dispid_Para.aKd*(err-errLast));
  if(valueOut > outMax2) valueOut=outMax2;
  if(valueOut < outMin2) valueOut=outMin2;
  errLast=err;
  return valueOut;
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
   Angpid_Para.dKp=fKp;
   Angpid_Para.dKi=fKi;
   Angpid_Para.dKd=fKd;
   
}
float AnglePid(float valueSet,float valueNow)
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
  
  iTerm+=(Angpid_Para.dKi*err);
  if(iTerm > outMax) iTerm=outMax;//让iTerm处于[outMin,outMax]的范围
  if(iTerm < outMin) iTerm=outMin;
  
  valueOut=(Angpid_Para.dKp*err)+iTerm+(Angpid_Para.dKd*(err-errLast));
  if(valueOut > outMax) valueOut=outMax;//让valueOut处于[outMin，outMax]的范围
  if(valueOut < outMin) valueOut=outMin;
  errLast=err;
  return valueOut;
}
