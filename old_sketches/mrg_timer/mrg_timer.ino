/*!
 * @file HelloWorld.ino
 * @brief Show helloworld.
 * @copyright	Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @maintainer [yangfeng](feng.yang@dfrobot.com)
 * @version  V1.0
 * @date  2021-09-24
 * @url https://github.com/DFRobot/DFRobot_RGBLCD1602
 */
#include "DFRobot_RGBLCD1602.h"
#include "rx8803.h"

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

const int NO_WINNER_YET = -1;

// TODO
// test with hw signals
// test with usb serial
// test with actual serial
// nice to have: control software over serial


enum state {
  STATE_SETUP,
  STATE_COUNTDOWN_START,
  STATE_WAIT,
  STATE_WINNER
};

enum state state;

const int f_in = 2;
const int led_excite = 4; // these control the "motor off" LEDs
const int led_start = 3; 
long unsigned int g_count = 0;

//whatever the count was when we initiate the race
long unsigned int g_t0 = 0;


// indices into arrays for each lanes
const int lane0 = 0;
const int lane1 = 1;

// end times for each lane
long unsigned int g_t1[2] = {0,0};
int win_led[] = {A6, A7};
int beam_in[] = {A2, A3};
int button_in = 9;
int g_winner = NO_WINNER_YET;


DFRobot_RGBLCD1602 lcd(/*lcdCols*/16,/*lcdRows*/2);  //16 characters and 2 lines of show

void setup() {
    state = STATE_SETUP;
    lcd.init();
    pinMode(f_in,INPUT);
    pinMode(led_excite, OUTPUT);
    pinMode(win_led[0], OUTPUT);
    pinMode(win_led[1], OUTPUT);
    pinMode(beam_in[0], INPUT);
    pinMode(beam_in[1], INPUT);
    pinMode(button_in, INPUT_PULLUP);
    /**
     *  @brief initialize the LCD and master IIC
     */ 
    
    // Print a message to the LCD.
    lcd.print("MRGTimer v0.1");
    delay(1000);

    attachInterrupt(digitalPinToInterrupt(f_in), count, RISING);
    
}

// this is the edge triggered interrupt on the precision rtc
void count() {
  g_count++;
}

void beam_excite() {
  digitalWrite(led_excite, LOW);
}

void beam_off() {
  digitalWrite(led_excite, HIGH);
}

void start_race() {
  attachInterrupt(digitalPinToInterrupt(f_in), count, RISING);
  start_light();
  beam_off();
}

void stop_race_counter() {
  detachInterrupt(digitalPinToInterrupt(f_in));
  g_count = 0;
}

void start_light() {
  digitalWrite(led_start, LOW);
}

void winner_light(int i) {
  digitalWrite(win_led[i], LOW);
}

void winner_light_reset() {
  digitalWrite(win_led[0], HIGH);
  digitalWrite(win_led[1], HIGH);
}

void do_countdown() {

  for(int i = 0; i < 3; ++i) {
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print(3-i);
    winner_light(0); winner_light(1);
    delay(500);
    winner_light_reset();
    delay(500);
  }
}

void blink_winner() {
    for(int i = 0; i < 5; ++i) {
    winner_light(g_winner);
    delay(500);
    winner_light_reset();
    delay(500);
  }
}

int detect_beam() {
  if (digitalRead(beam_in[lane0]) == LOW) {
    return lane0;
  }
  if (digitalRead(beam_in[lane1]) == LOW) {
    return lane1;
  }
  return NO_WINNER_YET;
}

int advance_state() {
  if (digitalRead(button_in)==LOW) {
    while(digitalRead(button_in)==LOW); //wait for it to go back high
    return 1;
  }
  return 0;
}


void check_win(int i) 
{
  int other;
  other = (i == lane0 ? lane1 : lane0 );
  if (0 == g_t1[i] && detect_beam() == i)
  {
    g_t1[i] = g_count;
    if (0 == g_t1[other]) // if the other lane (lane1) hasnt finished, we are the winner
    {
      winner_light(i);
      g_winner = i;
    }
  }
}

void lcd_message(const char* str)
{
  static const char* last = NULL;
  if ( last != str ) {
    lcd.clear();
    lcd.setCursor(0,1);
    lcd.print(str);
  }

  last = str;
}

long unsigned int print_time(int i)
{
  lcd.setCursor(0,i);
  long unsigned int delta = g_t1[i] - g_t0;
  if (0 == g_t1[i]) 
  {
    delta = 0;  
  }
  // 32768 counts is one second
  int minutes = delta / (32768*60);
  int seconds = (delta/ 32768)%60;
  int milliseconds = (delta/33)%1000;
  lcd.print("L"); lcd.print(i); lcd.print(": "); 
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

void show_times()
{
  lcd.clear();
  print_time(lane0);
  print_time(lane1);
}

void loop()
{
  switch(state) {
    case STATE_SETUP:
      lcd_message("SETUP ROBOTS");
      beam_excite();
      g_t1[lane0] = 0;
      g_t1[lane1] = 0;
      stop_race_counter();
      g_winner = NO_WINNER_YET;
      if (advance_state()) {
        state = STATE_COUNTDOWN_START;
      }
    break;
    case STATE_COUNTDOWN_START:
      do_countdown();
      lcd_message("GO!!!");
      start_race();
      state = STATE_WAIT;
    break;
    case STATE_WAIT:
      check_win(lane0);
      check_win(lane1);

      // testing...
      g_t1[0] = g_count;
      g_t1[1] = g_count;

      // wait for both to finish
      // testing...
      if (/*(g_t1[lane0] != 0 && g_t1[lane1] != 0) ||*/ advance_state())
      {
        show_times();
        blink_winner();
        state = STATE_WINNER;
      }
    break;
    case STATE_WINNER:
      if (advance_state())
      {
        state = STATE_SETUP;
      }
    break;
  }
}
