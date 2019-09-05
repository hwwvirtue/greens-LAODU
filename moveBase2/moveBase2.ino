
/**
 * @file moveBase2.ino
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-09-05
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include "POINT2POINT.h"
#include "PID.h"
#include "moveBase2.h"

/*显示器*/
long int _millis1 = 0;
#include "Adafruit_SSD1306.h"
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

//调路径用
//#define Logic_DEBUG
#define Logic_DEBUG2

//与杨迁传输部分参数
int Get_buf[8];
unsigned char counter = 0;
unsigned char sign = 0;
getpos getPos_U;
getpos_LR getPos_LR1;

//初始判断条件部分参数
int Red_Blue = 0;   //红(0)  蓝(1)
int BarrelNum = 0;  //路线号
int Shun_Ni = 0;    //顺时针(0) 逆时针(1)
int close_size = 0; //大(2) 中(1) 小(0)

//路径部分
int step_flag = 0;      //步骤
unsigned char motorNum; //倒车电机号

//PID部分电机函数参数
int LEFT_v, RIGHT_v;

//POINT2POINT部分电机函数参数
int Point_speed_Serial[2] = {0};

//与乐天传输部分参数
unsigned char Pri_buf[6];

long int _millis = 0;

void setup()
{
  Serial.begin(9600);

  //!杨迁口
  Serial2.begin(9600);
  //!乐天口
  Serial3.begin(9600);
  //!分球传输
  pinMode(40, INPUT);
  pinMode(41, OUTPUT);
  //!射球传输
  pinMode(42, INPUT);
  //!顺 逆  右手(52)挥,顺  两手挥,逆
  pinMode(52, INPUT);
  digitalWrite(52, LOW);
  pinMode(53, INPUT);
  digitalWrite(53, LOW);
  //自动 手动
  pinMode(51, INPUT);
  digitalWrite(51, HIGH);
  //车后俩按键
  pinMode(48, INPUT);
  digitalWrite(48, HIGH);
  pinMode(49, INPUT);
  digitalWrite(49, HIGH);

  //!显示器启动
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500);
  display.clearDisplay();
}

/**
   @brief 主循环
   @note 这是主循环
*/
void loop()
{
  ComwithYANG();
  //Judgment_Condition();
  close_size = 2;
  Shun_Ni = 0;
  //step_flag = ;

  //!路径部分
  if (close_size == 2)
  { //走 圈 开 始(大 圈 开 始)
    if (step_flag == 0)
    {
      if (Shun_Ni == 0)
      {
        closeRound(0, 2400, 1500, 1, 1500, 2);
#ifdef Logic_DEBUG
        Serial.print(millis());
        Serial.println("  closeRound1");
#endif
      }
      else
      {
        closeRound(0, 2400, 1500, 2, 1500, 2);
      }
//      if (300 < getPos_U.X && getPos_U.X < 500 && 2300 < getPos_U.Y && getPos_U.Y < 2500)
//        step_flag = 1;
#ifdef Logic_DEBUG
      if (millis() - _millis > 3000)
        step_flag = 1;
#endif
    }
    if (step_flag == 1)
    {
      if (Shun_Ni == 0)
      {
        closeRound(0, 2400, 1000, 1, 1500, 1);
#ifdef Logic_DEBUG
        Serial.print(millis());
        Serial.println("  closeRound2");
#endif
      }
      else
      {
        closeRound(0, 2400, 1000, 2, 1500, 1);
      }
      if (299 < getPos_U.X && getPos_U.X < 500 && 300 < getPos_U.Y && getPos_U.Y < 500)
        step_flag = 2;
#ifdef Logic_DEBUG
      if (millis() - _millis > 6000)
        step_flag = 2;
#endif
    }
  } //走 圈 完 毕
  else
  { //走 圈 开 始(中 圈 开 始)
    if (step_flag == 0)
    {
      if (Shun_Ni == 0)
      {
        closeRound(0, 2400, 1000, 1, 1500, 2);
#ifdef Logic_DEBUG
        Serial.print(millis());
        Serial.println("  closeRound1");
#endif
      }
      else
      {
        closeRound(0, 2400, 1000, 2, 1500, 2);
      }
      if (300 < getPos_U.X && getPos_U.X < 500 && 2300 < getPos_U.Y && getPos_U.Y < 2500)
        step_flag = 1;
#ifdef Logic_DEBUG
      if (millis() - _millis > 3000)
        step_flag = 1;
#endif
    }
    if (step_flag == 1)
    {
      if (Shun_Ni == 0)
      {
        closeRound(0, 2400, 500, 1, 1500, 1);
#ifdef Logic_DEBUG
        Serial.print(millis());
        Serial.println("  closeRound2");
#endif
      }
      else
      {
        closeRound(0, 2400, 500, 2, 1500, 1);
      }
      if (299 < getPos_U.X && getPos_U.X < 500 && 300 < getPos_U.Y && getPos_U.Y < 500)
        step_flag = 2;
#ifdef Logic_DEBUG
      if (millis() - _millis > 6000)
        step_flag = 2;
#endif
    }
  } //走 圈 完 毕(中 圈 开 始)
  if (step_flag == 2)
  {
    straightLine(1, 0, 0, 0, 1500);
    //    if(299 < getPos_U.X && getPos_U.X < 500 && 1825 < getPos_U.Y && getPos_U.Y < 1925)
    //       step_flag = 3;
  }
  if (step_flag == 3)
  { //撞 中 间 开 始
#ifdef Logic_DEBUG
    Serial.print(millis());
    Serial.println("  zhuangbian");
#endif
    back_Turn(0, 1500);
    if (digitalRead(48) == LOW && digitalRead(49) == LOW)
    {
      PID_speed_Serial[0] = 0;
      PID_speed_Serial[1] = 0;
      step_flag = 4;
    }
#ifdef Logic_DEBUG
    if (millis() - _millis > 10000)
      step_flag = 4;
#endif
  } //撞 中 间 完 毕

  if (step_flag == 4)
  { //开 始 给 分 球，射 球 发 送 标 志
    digitalWrite(40, HIGH);
    digitalWrite(42, HIGH);
    step_flag = 5;
#ifdef Logic_DEBUG
    Serial.print(millis());
    Serial.println("  sheqiu");
    step_flag = 0;
    _millis = millis();
#endif
  } //结 束 给 分 球，射 球 发 送 标 志
  if (step_flag == 5)
  { //若 没 球，重 新 收 球
    if (digitalRead(41) == HIGH)
    {
      step_flag = 0;
    }
  }
  ComwithTIAN();

  Display();

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
    if (tail == Get_buf[8]) //检查帧头，帧尾
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

      int q = 0;
      q = getPos_U.X;
      getPos_U.X = getPos_U.Y;
      getPos_U.Y = q;
      //      Serial.print("x:");
      //      Serial.print(getPos_U.X);
      //      Serial.print(" y:");
      //      Serial.print(getPos_U.Y);
      //      Serial.print(" p:");
      //      Serial.println(getPos_U.ANG);
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
    //        Serial.println(readedchar);
    Get_buf[counter] = readedchar;
    if (counter == 0 && Get_buf[0] != 0xAA)
      break; // 检查帧头
    else
    {
      counter++;
      if (counter == 9) //接收到数据1
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
    Red_Blue = 0;
  }
  else if (digitalRead(28) == HIGH && digitalRead(29) == LOW)
  {
    Red_Blue = 1;
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
  LEFT_v = motor1();
  RIGHT_v = motor2();
  //LEFT_v = 1000;
  //RIGHT_v = 2000;
  Serial.print("LEFT:");
  Serial.print(LEFT_v);
  Serial.print("  Right:");
  Serial.print(RIGHT_v);

  Serial.print("  x:");
  Serial.print(getPos_U.X);
  Serial.print("  y:");
  Serial.print(getPos_U.Y);
  Serial.print("  p:");
  Serial.println(getPos_U.ANG);
  unsigned char left1, right1, left2, right2;
  left1 = LEFT_v >> 8;
  right1 = LEFT_v & 0xFF;
  left2 = RIGHT_v >> 8;
  right2 = RIGHT_v & 0xFF;

  Serial3.write(0xAA);
  delay(10);
  Serial3.write(left1);
  Serial.print("left1:");
  Serial.print(left1);
  delay(10);
  Serial3.write(right1);
  Serial.print(" right1:");
  Serial.print(right1);
  delay(10);
  Serial3.write(left2);
  Serial.print(" left2:");
  Serial.print(left2);
  delay(10);
  Serial3.write(right2);
  Serial.print(" right2:");
  Serial.println(right2);
  delay(10);
  Serial3.write('N');
  delay(10);
  Serial3.write(0x55);
  delay(10);
}

void Display()
{
  if (millis() - _millis1 >= 100)
  {
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();

    display.setCursor(0, 0);
    display.print("Time:");
    display.setCursor(40,0);
    display.print(millis());

    display.setCursor(0, 10);
    display.print("L:");
    display.setCursor(10,10);
    display.print(LEFT_v);

    display.setCursor(64, 10);
    display.print("R:");
    display.setCursor(74, 10);
    display.print(RIGHT_v);

    display.setCursor(0,18);
    display.print("X:");
    display.setCursor(10,18);
    display.print(getPos_U.X);

    display.setCursor(50, 18);
    display.print("Y:");
    display.setCursor(60, 18);
    display.print(getPos_U.Y);

    display.setCursor(90, 18);
    display.print("P:");
    display.setCursor(100, 18);
    display.print(getPos_U.ANG);

    display.display();
    _millis1 = millis();
  }
}