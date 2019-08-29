#ifndef _distancePID_H
#define _distancePID_H
static float outMax2=500;
static float outMin2=-500;
static struct PIDPara_{
  float dKp;
  float dKi;
  float dKd;
}Dispid_Para;
void Distance_PidPara(float fKp,float fKi,float fKd);
float DistancePid(float valueSet,float valueNow);
#endif
