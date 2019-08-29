#ifndef _PID_H
#define _PID_H


//距离PID
static float outMax2=500;
static float outMin2=-500;
static struct PIDPara_D{
  float dKp;
  float dKi;
  float dKd;
}Dispid_Para;
void Distance_PidPara(float fKp,float fKi,float fKd);
float DistancePid(float valueSet,float valueNow);




//角度PID
static float outMax=1500;
static float outMin=-1500;
static struct PIDPara_A{
  float aKp;
  float aKi;
  float aKd;
}Angpid_Para;
void Angle_PidPara(float fKp,float fKi,float fKd);
float AnglePid(float valueSet,float valueNow);
#endif
