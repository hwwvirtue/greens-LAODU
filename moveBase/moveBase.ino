#include "POINT2POINT.h"
#include "PID.h"
#include "moveBase.h"

//与杨迁传输部分参数
int Get_buf[8];
unsigned char counter = 0;
unsigned char sign = 0;
getpos getPos_U;
getpos_LR getPos_LR1;


//初始判断条件部分参数
int Red_Bule = 0;//红(0)  蓝(1)
int BarrelNum = 0;//路线号
int Shun_Ni = 0;//顺时针(0) 逆时针(1)
int close_size = 0;//大(2) 中(1) 小(0)

//路径部分
int step_flag = 0;//步骤

//PID部分电机函数参数
int LEFT_v, RIGHT_v;

//POINT2POINT部分电机函数参数
int Point_speed_Serial[2] = {0};

//与乐天传输部分参数
unsigned char Pri_buf[6];

void setup()
{
  //!杨迁口
  Serial2.begin(9600);
  //!乐天口
  Serial3.begin(9600);
  //!分球传输高电平
  //  pinMode(31, INPUT);
  //  digitalWrite(31, HIGH);
  //!射球标志发送串口
  Serial1.begin(9600);
  Serial.begin(9600);
  //红(28)    蓝(29)
  pinMode(28, INPUT);
  digitalWrite(28, HIGH);
  pinMode(29, INPUT);
  digitalWrite(29, HIGH);
  //1桶(30) 2桶(31) 3桶(32) 4桶(33)
  pinMode(30, INPUT);
  digitalWrite(30, HIGH);
  pinMode(31, INPUT);
  digitalWrite(31, HIGH);
  pinMode(32, INPUT);
  digitalWrite(32, HIGH);
  pinMode(33, INPUT);
  digitalWrite(33, HIGH);
  //大(22) 中(23) 小圈(24)
  pinMode(22, INPUT);
  digitalWrite(22, HIGH);
  pinMode(23, INPUT);
  digitalWrite(23, HIGH);
  pinMode(24, INPUT);
  digitalWrite(24, HIGH);
  //!顺 逆  右手(52)挥,顺  左手(53)挥,逆
  pinMode(52, INPUT);
  digitalWrite(52, LOW);//初始状态为LOW，检测有物体也为LOW
  pinMode(53, INPUT);
  digitalWrite(53, LOW);
  //车后俩按键
  pinMode(48, INPUT);
  digitalWrite(48, HIGH);
  pinMode(49, INPUT);
  digitalWrite(49, HIGH);
  //自动 手动
  pinMode(51, INPUT);
  digitalWrite(51, HIGH);
}

void loop()
{
  ComwithYANG();
  Judgment_Condition();

  //!路径部分
  if (Red_Bule == 0)
  {
    if (1)
    {
      if (close_size == 2)
      {
        if (step_flag == 0)
        {
          //顺  外圆
          closeRound(0, 2400, 1900, 0, 1500, 2);
          if (1775 < getPos_U.Y && getPos_U.Y < 1850 && -75 < getPos_U.X && getPos_U.X < 75)
            step_flag = 1;
        }
      }
      if (close_size == 1)
      {
        //中圆
        closeRound(0, 2400, 1000, 0, 1500, 1);
      }
    }
    else if (digitalRead(52) == HIGH && digitalRead(53) == LOW)
    {
      if (close_size == 2)
      {
        //逆  外圆
        closeRound(0, 2400, 2100, 1, 1500, 2);
      }
      if (close_size == 1)
      {
        //中圆
        closeRound(0, 2400, 1000, 1, 1200, 1);
      }
    }
    if (-75 < getPos_U.X && getPos_U.X < 75)
    {
      straightLine(1, 0, 0, 0, 1500);
    }
  }
  else
  {
    
  }
  ComwithTIAN();
}

/**
   @brief 获取定位模块数据
   @details 处理Get_buf的定位模块数据
            储存在结构体getPos_U的X Y P中
*/
void ComwithYANG()
{
  int tail = 0x55; //帧尾数值
  if (sign == 1)
  {
    sign = 0;
    if (tail == Get_buf[7]) //检查帧头，帧尾
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
  serialEvent2();
}
/**
   @brief 串口2事件
   @details 将串口2接受到的数据储存到Get_buf中
*/
void serialEvent2()
{
  while (Serial2.available())
  {
    int readedchar = Serial2.read();
    //Serial.println(readedchar);
    Get_buf[counter] = readedchar;
    if (counter == 0 && Get_buf[0] != 0xAA)
      break; // 检查帧头
    else
    {
      counter++;
      if (counter == 8) //接收到数据1
      {
        counter = 0; //重新赋值，准备下一帧数据的接收
        sign = 1;
      }
    }
  }
}


/**
   @brief 获取坐标X
   @return int
*/
int GetPosX()
{
  return getPos_U.X;
}
/**
   @brief 获取坐标Y
   @return int
*/
int GetPosY()
{
  return getPos_U.Y;
}
/**
   @brief 获取坐标p
   @return int
*/
int GetAngle()
{
  return getPos_U.ANG;
}


/**
   @brief 判断初始条件

*/
void Judgment_Condition()
{
  //判断 红 蓝 方
  if (digitalRead(28) == LOW && digitalRead(29) == HIGH)
  {
    Red_Bule = 0;
  }
  else if (digitalRead(28) == HIGH && digitalRead(29) == LOW)
  {
    Red_Bule = 1;
  }

  //判断抽到的  桶号  以得到对应的  路线号
  if (digitalRead(30) == LOW && digitalRead(31) == LOW && digitalRead(32) == HIGH && digitalRead(33) == HIGH)//1.2桶
  {
    BarrelNum = 1;
  }
  else if (digitalRead(30) == HIGH && digitalRead(31) == LOW && digitalRead(32) == LOW && digitalRead(33) == HIGH)//2.3桶
  {
    BarrelNum = 2;
  }
  else if (digitalRead(30) == HIGH && digitalRead(31) == HIGH && digitalRead(32) == LOW && digitalRead(33) == LOW)//3.4桶
  {
    BarrelNum = 3;
  }
  else if (digitalRead(30) == LOW && digitalRead(31) == HIGH && digitalRead(32) == HIGH && digitalRead(33) == LOW)//1.4桶
  {
    BarrelNum = 4;
  }
  else if (digitalRead(30) == LOW && digitalRead(31) == HIGH && digitalRead(32) == LOW && digitalRead(33) == HIGH)//1.3桶
  {
    BarrelNum = 5;
  }
  else if (digitalRead(30) == LOW && digitalRead(31) == HIGH && digitalRead(32) == LOW && digitalRead(33) == HIGH)//2.4桶
  {
    BarrelNum = 6;
  }

  //判断 顺 逆
  if (digitalRead(52) == LOW && digitalRead(53) == HIGH)
  {
    Shun_Ni = 0;
  }
  else if (digitalRead(52) == HIGH && digitalRead(53) == LOW)
  {
    Shun_Ni = 1;
  }

  //判断初始圈大小
  if (digitalRead(22) == LOW && digitalRead(23) == HIGH)
  {
    close_size = 2;
  }
  else if (digitalRead(22) == HIGH && digitalRead(23) == LOW)
  {
    close_size = 1;
  }
}





/**
   @brief

   @param ElmoNum
   @param vel
*/
void VelCrl(unsigned char ElmoNum, int vel)
{
  if (ElmoNum == MOTOR_ONE)
    Point_speed_Serial[0] = vel;
  else
    Point_speed_Serial[1] = vel;
}


/**
   @brief 和乐天通信

*/
void ComwithTIAN()
{
  //与乐天通信部分
  LEFT_v = PID_speed_Serial[0];
  RIGHT_v = PID_speed_Serial[1];
  Serial.print("LEFT:");
  Serial.println(LEFT_v);
  Serial.print("RIGHT:");
  Serial.println(RIGHT_v);
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
    Serial3.write('N');
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
    Serial3.write('Y');
    delay(10);
    Serial3.write(0x55);
    delay(10);
  }
}
