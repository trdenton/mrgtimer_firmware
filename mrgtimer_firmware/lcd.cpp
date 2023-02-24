#include "Arduino.h"
#include "DFRobot_RGBLCD1602.h"

static DFRobot_RGBLCD1602 lcd(16, 2); // 16 characters x 2 lines

// assumes cursor is in the correct position
// converts t to MM:SS.sss
// assumes we are receiving ticks at 32768 kHz
long unsigned int lcd_print_time(long unsigned int t)
{
  // 32768 counts is one second
  int minutes = t / (32768*60);
  int seconds = (t/ 32768)%60;
  int milliseconds = (t/33)%1000;
  if (minutes < 10)
    lcd.print("0");
  lcd.print(minutes); lcd.print(":"); 

  if (seconds < 10)
    lcd.print("0");
  lcd.print(seconds); lcd.print(".");

  if (milliseconds < 100)
    lcd.print("0");
  if (milliseconds < 10)
    lcd.print("0");
  lcd.print(milliseconds);
}

void lcd_init() 
{
  lcd.init();
  lcd.print("MRGTimer v0.1");
}

void lcd_clear()
{
  lcd.clear();
}

void lcd_message(const char* str)
{
  // basic mechanism to not spam LCD with
  // same message
  static const char* last = NULL;
  if ( last != str ) {
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print(str);
  }

  last = str;
}
