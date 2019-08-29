/**
* @brief 距离PID，返回值即为速度的增量，此增量使用两个前轮
* @param valueSet：距离设定值
* @param valueNow：当前距离值
* @retval none
* @attention
*/
#include"distancePID.h"
void Distance_PidPara(float fKp,float fKi,float fKd)
{
  Dispid_Para.dKp=fKp;
  Dispid_Para.dKi=fKi;
  Dispid_Para.dKd=fKd;
}
float DistancePid(float valueSet,float valueNow)
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
