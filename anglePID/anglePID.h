#ifndef _anglePID_H
#define _anglePID_H
static float outMax=1500;
static float outMin=-1500;
static struct PIDPara_{
  float aKp;
  float aKi;
  float aKd;
}Angpid_Para;
void Angle_PidPara(float fKp,float fKi,float fKd);
float AnglePid(float valueSet,float valueNow);
#endif
