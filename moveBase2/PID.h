#ifndef PID_H
#define PID_H
typedef float fp32;
typedef double fp64;
#ifndef NULL
#define NULL 0
#endif

#ifdef  __PID_GLOBALS   //其定义加在自己的c文件中
#define __PID_EXT
#else
#define __PID_EXT extern
#endif

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    char mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, char mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern fp32 PID_motor_Calc(PidTypeDef *pid, fp32 ref, fp32 set, char sign);

extern void PID_clear(PidTypeDef *pid);
extern void PID_para_init(PidTypeDef *pid,const fp32 PID[3]);
  

#endif
