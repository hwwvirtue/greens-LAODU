#include"POINT2POINT.h"
#include"PID.h"
#include"cai.h"


//与杨迁传输部分参数
int Get_buf[8];
unsigned char counter = 0;
unsigned char sign = 0;
getpos getPos_U;
getpos_LR getPos_LR1;


//路径部分
//直线部分参数
Distance_Pidass Distance_PidAss2;
Angle_Pidass Angle_PidAss2;
straightLinepara straightLinePara1;

//圆部分参数
Distance_Pidass Distance_PidAss1;
Angle_Pidass Angle_PidAss1;

//后退部分参数
Angle_Pidass Angle_PidAss3;
fp32 getAnglenow;

//PID部分电机函数参数
int LEFT_v, RIGHT_v;

//POINT2POINT部分电机函数参数
int Point_speed_Serial[2] = {0};

//与乐天传输部分参数
unsigned char Pri_buf[6];



void VelCrl(unsigned char ElmoNum, int vel)
{
  if (ElmoNum == MOTOR_ONE)
    Point_speed_Serial[0] = vel;
  else
    Point_speed_Serial[1] = vel;
}


void setup()
{
  //杨迁口
  Serial2.begin(9600);
  //乐天口
  Serial3.begin(9600);
  //分球标志发送串口
  Serial1.begin(9600);
  Serial.begin(9600);
  //顺 逆  右手(52)挥,顺  两手挥,逆
  pinMode(52, INPUT);
  digitalWrite(52, LOW);
  pinMode(53, INPUT);
  digitalWrite(53, LOW);
  //自动 手动
  pinMode(51, INPUT);
  digitalWrite(51, HIGH);
  //车后俩按键
  pinMode(49, INPUT);
  digitalWrite(49, HIGH);
  pinMode(47, INPUT);
  digitalWrite(47, HIGH);
}

void loop()
{
  //与杨迁通信部分
  int sum = 0x55;//帧尾数值
  if (sign == 1)
  {
    sign = 0;
    if ( sum == Get_buf[7] )     //检查帧头，帧尾
    {
      getPos_LR1.X_L = Get_buf[1];
      getPos_LR1.X_R = Get_buf[2];
      getPos_U.X = getPos_LR1.X_L;
      getPos_U.X <<= 8;
      getPos_U.X |= getPos_LR1.X_R;

      getPos_LR1.Y_L = Get_buf[3];
      getPos_LR1.Y_R = Get_buf[4];
      getPos_U.Y = getPos_LR1.Y_L;
      getPos_U.Y <<= 8;
      getPos_U.Y |= getPos_LR1.Y_R;

      getPos_LR1.ANG_L = Get_buf[5];
      getPos_LR1.ANG_R = Get_buf[6];
      getPos_U.ANG = getPos_LR1.ANG_L;
      getPos_U.ANG <<= 8;
      getPos_U.ANG |= getPos_LR1.ANG_R;

      Serial.print("x:");
      Serial.println(getPos_U.X);
      Serial.print("y:");
      Serial.println(getPos_U.Y);
      Serial.print("ang:");
      Serial.println(getPos_U.ANG);
    }
  }
  while (Serial2.available())
  {
    int readedchar = Serial2.read();
    //Serial.println(readedchar);
    Get_buf[counter] = readedchar;
    if (counter == 0 && Get_buf[0] != 0xAA) break; // 检查帧头
    else
    {
      counter++;
      if (counter == 8)             //接收到数据1
      {
        counter = 0;               //重新赋值，准备下一帧数据的接收
        sign = 1;
      }
    }
  }




  //路径部分
  //三圈收球
//  int time_Now;
//  time_Now = millis();
//  delay(50);
//  while(time_Now%100 == 0)
//  {
    if (digitalRead(52) == LOW && digitalRead(53) == HIGH)
  {
    //顺  外圆
    Distance_PidAss1.dKP = 0.1f;
    Distance_PidAss1.dKI = 0;
    Distance_PidAss1.dKD = 0;
    Angle_PidAss1.aKP = 40;
    Angle_PidAss1.aKI = 0;
    Angle_PidAss1.aKD = 0;
    while (getPos_U.X < -200 || getPos_U.X > 200)
    {
      closeRound(Distance_PidAss1, Angle_PidAss1, getPos_U, 0, 2400, 2100, 0, 1500);
    }
    //中圆
    Distance_PidAss1.dKP = 0.08f;
    Distance_PidAss1.dKI = 0;
    Distance_PidAss1.dKD = 0;
    Angle_PidAss1.aKP = 80;
    Angle_PidAss1.aKI = 0;
    Angle_PidAss1.aKD = 0;
    while (getPos_U.X < -200 || getPos_U.X > 200)
    {
      closeRound(Distance_PidAss1, Angle_PidAss1, getPos_U, 0, 2400, 1500, 0, 1500);
    }
    //内圆
    Distance_PidAss1.dKP = 0.08f;
    Distance_PidAss1.dKI = 0;
    Distance_PidAss1.dKD = 0;
    Angle_PidAss1.aKP = 70;
    Angle_PidAss1.aKI = 0;
    Angle_PidAss1.aKD = 0;
    while (getPos_U.X < -200 || getPos_U.X > 200)
    {
      closeRound(Distance_PidAss1, Angle_PidAss1, getPos_U, 0, 2400, 900, 0, 1500);
    }
  }
  else if (digitalRead(52) == HIGH && digitalRead(53) == LOW)
  {
    //逆  外圆
    Distance_PidAss1.dKP = 0.1f;
    Distance_PidAss1.dKI = 0;
    Distance_PidAss1.dKD = 0;
    Angle_PidAss1.aKP = 40;
    Angle_PidAss1.aKI = 0;
    Angle_PidAss1.aKD = 0;
    while (getPos_U.X < -200 || getPos_U.X > 200)
    {
      closeRound(Distance_PidAss1, Angle_PidAss1, getPos_U, 0, 2400, 2100, 1, 1500);
    }
    //中圆
    Distance_PidAss1.dKP = 0.08f;
    Distance_PidAss1.dKI = 0;
    Distance_PidAss1.dKD = 0;
    Angle_PidAss1.aKP = 80;
    Angle_PidAss1.aKI = 0;
    Angle_PidAss1.aKD = 0;
    while (getPos_U.X < -200 || getPos_U.X > 200)
    {
      closeRound(Distance_PidAss1, Angle_PidAss1, getPos_U, 0, 2400, 1500, 1, 1500);
    }
    //内圆
    Distance_PidAss1.dKP = 0.08f;
    Distance_PidAss1.dKI = 0;
    Distance_PidAss1.dKD = 0;
    Angle_PidAss1.aKP = 70;
    Angle_PidAss1.aKI = 0;
    Angle_PidAss1.aKD = 0;
    while (getPos_U.X < -200 || getPos_U.X > 200)
    {
      closeRound(Distance_PidAss1, Angle_PidAss1, getPos_U, 0, 2400, 900, 1, 1500);
    }
  }

  //撞中间
  Distance_PidAss2.dKP = 0.1;
  Distance_PidAss2.dKI = 0;
  Distance_PidAss2.dKD = 4;
  Angle_PidAss2.aKP = 27;
  Angle_PidAss2.aKI = 0;
  Angle_PidAss2.aKD = 0;
  straightLinePara1.A1 = 1;
  straightLinePara1._B1 = 0;
  straightLinePara1.C1 = 0;
  while (1925 < getPos_U.Y && getPos_U.Y < 1930)
  {
    straightLine(Distance_PidAss2 , Angle_PidAss2  , getPos_U , straightLinePara1 , 0, 1500);
  }

  //往回倒
  while (digitalRead(49) == HIGH &&  digitalRead(47) == HIGH)
  {
    Angle_PidAss2.aKP = 8;//AKp=gospeed/180
    Angle_PidAss2.aKI = 0;
    Angle_PidAss2.aKD = 0;
    getAnglenow = getPos_U.ANG;
    back_Turn(Angle_PidAss3, getAnglenow, 0, 1500);
  }

  //发送分球指令
  Serial3.write(digitalRead(49));
  Serial3.write(digitalRead(47));




















  //与乐天通信部分
  if (Serial3.available())
  {
//    LEFT_v = PiD_speed_Serial[0];
//    RIGHT_v = PiD_speed_Serial[1];
    LEFT_v = 1500;
    RIGHT_v = 1500;
    unsigned char left1, right1, left2, right2;
    left1 = LEFT_v >> 8;
    right1 = LEFT_v & 0xFF;
    left2 = RIGHT_v >> 8;
    right2 = RIGHT_v & 0xFF;
    if (digitalRead(51) == HIGH)
    {
      Serial3.write(0xAA);
      delay(10);
      Serial3.write(left1);
      delay(10);
      Serial3.write(right1);
      delay(10);
      Serial3.write(left2);
      delay(10);
      Serial3.write(right2);
      delay(10);
      Serial3.write("N");
      delay(10);
      Serial3.write(0x55);
      delay(10);
    }
    else
    {
      Serial3.write(0xAA);
      delay(10);
      Serial3.write(left1);
      delay(10);
      Serial3.write(right1);
      delay(10);
      Serial3.write(left2);
      delay(10);
      Serial3.write(right2);
      delay(10);
      Serial3.write("Y");
      delay(10);
      Serial3.write(0x55);
      delay(10);
    }
  }
}
