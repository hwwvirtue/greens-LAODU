#include "POINT2POINT.h"
#include "PID.h"
#include "moveBase2.h"

/*显示器*/
long int _millis1 = 0;
#include "Adafruit_SSD1306.h"
//#define OLED_RESET 4
//Adafruit_SSD1306 display(OLED_RESET);

//调路径用
//#define Logic_DEBUG


#define shun_PIN  A0
#define ni_PIN  A1

#define To_fenqiu_PIN  52
#define From_fenqiu_PIN  53

long int _millis = 0;
long int _millis2 = 0;
//与杨迁传输部分参数
int Get_buf[8];
unsigned char counter = 0;
unsigned char sign = 0;
getpos getPos_U;
getpos_LR getPos_LR1;

//初始判断条件部分参数
int Red_Blue = 0;   //红(0)  蓝(1)
int BarrelNum = 0;  //路线号
int Shun_Ni = 0;    //顺时针(1) 逆时针(2)
int close_size = 0; //大(2) 中(1) 小(0)

//路径部分
int step_flag = -1; //步骤
int state_flag = 0;
unsigned char motorNum; //倒车电机号
int seconds;

int GET_X_before= 0;
int GET_X_d = 0;
int GET_Y_before= 0;
int GET_Y_d = 0;
int GET_POS_d= 0;
//PID部分电机函数参数
int LEFT_v, RIGHT_v;

//POINT2POINT部分电机函数参数
int Point_speed_Serial[2] = {0};

//与乐天传输部分参数
unsigned char Pri_buf[6];

void setup()
{
  Serial.begin(9600);

  //!杨迁口
  Serial2.begin(9600);
  //!乐天口
  Serial3.begin(9600);



  //红(HIGH) 蓝(LOW) 队
  pinMode(A2, INPUT);
  digitalWrite(A2, HIGH);
  //!分球传输
  pinMode(52, OUTPUT);
  digitalWrite(52, HIGH);
  pinMode(53, INPUT);
  digitalWrite(53, HIGH);
  //!顺 逆
  pinMode(A0, INPUT);
  digitalWrite(A0, HIGH);
  pinMode(A1, INPUT);
  digitalWrite(A1, HIGH);
  //!显示器启动
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(500);
  display.clearDisplay();

  Serial2.write('r');
}

/**
   @brief 主循环
   @note 这是主循环
*/

void loop()
{
  ComwithYANG();
  if (digitalRead(A2) == HIGH)
  {
    Red_Blue = 0;
  }
  else if (digitalRead(A2) == LOW)
  {
    Red_Blue = 1;
  }

  if (step_flag == -1)
  {
    if (digitalRead(A0) == LOW || digitalRead(A1) == LOW)
    {
      if (digitalRead(shun_PIN) == LOW)
      {
        //顺时针
        Shun_Ni = 1;
        step_flag = 0;
      }
      else if (digitalRead(ni_PIN) == LOW)
      {
        //逆时针
        Shun_Ni = 2;
        step_flag = 0;
      }
    }
  }
  if (step_flag == 0)
  {
    closeRound(0, 2400, 1500, Shun_Ni, 3000, 2);
    if (700 < getPos_U.X && getPos_U.X < 1200 && 800 < getPos_U.Y && getPos_U.Y < 1200 && Shun_Ni == 1)
      {
        step_flag = 1;
        GET_X_before = 0;
        GET_Y_before = 0;
      }
    if (-1200 < getPos_U.X && getPos_U.X < -700 && 800 < getPos_U.Y && getPos_U.Y < 1200 && Shun_Ni == 2)
      {
        step_flag = 1;
        GET_X_before = 0;
        GET_Y_before = 0;
      }
  }


  // if (step_flag == 1)
  // {
  //   closeRound(0, 2200, 300, Shun_Ni, 3000, 0);
  //   if (800 < getPos_U.X && getPos_U.X < 1200 && 3600 < getPos_U.Y && getPos_U.Y < 3900 && Shun_Ni == 1)
  //     step_flag = 2;
  //   if (-1200 < getPos_U.X && getPos_U.X < -800 && 3600 < getPos_U.Y && getPos_U.Y < 3900 && Shun_Ni == 2)
  //     step_flag = 2;
  // }
  // if(step_flag == 2)
  // {
  //   closeRound(0, 1900, 800, Shun_Ni, 3000, 1);
  //   if (800 < getPos_U.X && getPos_U.X < 1200 && 800 < getPos_U.Y && getPos_U.Y < 1200 && Shun_Ni == 1)
  //     step_flag = 3;
  //   if (-1200 < getPos_U.X && getPos_U.X < -800 && 800 < getPos_U.Y && getPos_U.Y < 1200 && Shun_Ni == 2)
  //     step_flag = 3;
  // }


  //GET_Y_before= getPos_U.Y;
  if (step_flag == 1)
  {
    straightLine(0, 1, 0, 0, 1500);
     GET_X_d = GET_X_before - getPos_U.X;
     GET_Y_d = GET_Y_before - getPos_U.Y;
     GET_POS_d = GET_X_d*GET_X_d+GET_Y_d*GET_Y_d;
    //if (-100 < getPos_U.X && getPos_U.X < 100 && GET_Y_before- GET_Y_Old < 20 && -10 < getPos_U.ANG && getPos_U.ANG < 10 && 1500 < getPos_U.Y && getPos_U.Y < 1700) //1500 < getPos_U.Y && getPos_U.Y < 1700
      if(GET_POS_d<10)
      {
        step_flag = 2;
        seconds = millis();
      }
      if(millis()-_millis2>100)
      {
        GET_X_before= getPos_U.X;
        GET_Y_before= getPos_U.Y;
      }
  } //GET_Y_before- GET_Y_Old < 50

  //!靠 边 射 球
  if (step_flag == 2)
  {
    *motorCMD(1, 0, 0);
    *motorCMD(2, 0, 0);    //停车
    digitalWrite(To_fenqiu_PIN, LOW); //发送分球标志
    if (millis() - seconds > 50000) //等待没球
    {                 //!如 果 要 重 新 跑
      digitalWrite(To_fenqiu_PIN, HIGH);
      step_flag = 3;
    }
  }
  if (step_flag == 3)
  {
    back_Turn(0, 1500);
    if (-100 < getPos_U.X && getPos_U.X < 100 && 600 < getPos_U.Y && getPos_U.Y < 1000)
    {
      step_flag = 0;
     // state_flag++;
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
  //  Serial.print("LEFT:");
  //  Serial.print(LEFT_v);
  //  Serial.print("  Right:");
  //  Serial.print(RIGHT_v);
  //
  //  Serial.print("  x:");
  //  Serial.print(getPos_U.X);
  //  Serial.print("  y:");
  //  Serial.print(getPos_U.Y);
  //  Serial.print("  p:");
  //  Serial.println(getPos_U.ANG);
  unsigned char left1, right1, left2, right2;
  left1 = LEFT_v >> 8;
  right1 = LEFT_v & 0xFF;
  left2 = RIGHT_v >> 8;
  right2 = RIGHT_v & 0xFF;

  Serial3.write(0xAA);
  delay(10);
  Serial3.write(left1);
  //  Serial.print("left1:");
  //  Serial.print(left1);
  delay(10);
  Serial3.write(right1);
  //  Serial.print(" right1:");
  //  Serial.print(right1);
  delay(10);
  Serial3.write(left2);
  //  Serial.print(" left2:");
  //  Serial.print(left2);
  delay(10);
  Serial3.write(right2);
  //  Serial.print(" right2:");
  //  Serial.println(right2);
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
    display.setCursor(40, 0);
    display.print(millis());

    display.setCursor(0, 10);
    display.print("L:");
    display.setCursor(10, 10);
    display.print(LEFT_v);

    display.setCursor(64, 10);
    display.print("R:");
    display.setCursor(74, 10);
    display.print(RIGHT_v);

    display.setCursor(0, 18);
    display.print("X:");
    display.setCursor(10, 18);
    display.print(getPos_U.X);

    display.setCursor(50, 18);
    display.print("Y:");
    display.setCursor(60, 18);
    display.print(getPos_U.Y);

    display.setCursor(90, 18);
    display.print("P:");
    display.setCursor(100, 18);
    display.print(getPos_U.ANG);

    //    display.setCursor(0, 26);
    //    display.print("Red_Blue:");
    display.setCursor(100, 26);
    if (Red_Blue == 0)
    {
      display.print("Red");
    }
    else if (Red_Blue == 1)
    {
      display.print("Blue");
    }

   display.setCursor(0, 34);
    display.print("step_flag=");
    display.setCursor(90, 34);
    display.print(step_flag);

   display.setCursor(0, 46);
    display.print("GET_POS_d=");
    display.setCursor(90, 46);
    display.print(GET_POS_d);

    display.display();
    _millis1 = millis();
  }
}
