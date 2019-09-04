#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#define LOGO16_GLCD_HEIGHT 16 
#define LOGO16_GLCD_WIDTH  16 
void setup()        //初始化
{
  Serial.begin(9600);
  delay(500);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  
}
void loop()
{
  display.clearDisplay();   // clears the screen and buffer
  display.setTextSize(2); //选择字号
  display.setTextColor(WHITE);  //字体颜色
  display.setCursor(0,0);   //起点坐标
  display.println(123);
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.display();
  delay(2000);
}



  
  
