/**
* @brief 角度PID，返回值即为速度的增量，此增量使用两个前轮
* @param valueSet：角度设定值
* @param valueNow：当前角度值
* @retval none
* @attention 
*/
#include"anglePID.h"
void Angle_PidPara(float fKp,float fKi,float fKd)
{
   Angpid_Para.aKp=fKp;
   Angpid_Para.aKi=fKi;
   Angpid_Para.aKd=fKd;
   
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
  
  iTerm+=(Angpid_Para.aKi*err);
  if(iTerm > outMax) iTerm=outMax;//让iTerm处于[outMin,outMax]的范围
  if(iTerm < outMin) iTerm=outMin;
  
  valueOut=(Angpid_Para.aKp*err)+iTerm+(Angpid_Para.aKd*(err-errLast));
  if(valueOut > outMax) valueOut=outMax;//让valueOut处于[outMin，outMax]的范围
  if(valueOut < outMin) valueOut=outMin;
  errLast=err;
  return valueOut;
}
