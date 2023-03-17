#include <string.h>
#include <limits.h>
#include "rx8803.h"
#include "lcd.h"

/****************
 *
 * DEFINES
 *
 ****************/


#define WTF(_X_) do {Serial.print("Something weird on ");Serial.print(__LINE__);Serial.print(": ");Serial.println(_X_);} while(0);

#define PULSE_BUFFER_SIZE 16
#define NUMBER_LANES 2

#define TIMER_CLK_PIN 2
#define STARTER_BUTTON_PIN 9  // button input
#define STARTER_LIGHT_PIN 3   // This is the inhibit signal that stops car from moving
#define GATE_LIGHT_PIN 4      // This is what drives the sensor LEDs

#define LANE_1_FALSE_START_PIN A0
#define LANE_2_FALSE_START_PIN A1

#define LANE_1_FINISH_PIN A2
#define LANE_2_FINISH_PIN A3

#define LANE_1_WIN_LIGHT_PIN A6 // BUG: A6 cannot output :(
#define LANE_2_WIN_LIGHT_PIN A7 // BUG: A7 cannot output :(

#define LONG_PRESS_DURATION 2000

#define ERROR_MESSAGE_MS 5000

//Debug symbols.
#define DEBUG 1
#define LOG_DEBUG 9
#define LOG_EVENTS 8

#define RX_BUFF_LEN (32)

/**************************
 *
 * STRUCTS, ENUMS
 *
 *************************/

enum timer_state {
  STATE_IDLE,             
  STATE_IDLE_WAIT,        
  STATE_CALIBRATE_ANALOG, 
  STATE_CALIBRATE_SETTLE,
  STATE_STARTING,         
  STATE_STARTING_WAIT,    
  STATE_RUNNING,          
  STATE_RUNNING_WAIT,     
  STATE_PRE_START_FAILED, 
  STATE_FALSE_START,      
  STATE_FINISHED,         
  STATE_FINISHED_WAIT,    
  STATE_TEST_GATE,        
  STATE_TEST_CALIBRATE_ANALOG,
  STATE_TEST_MONITOR_ANALOG,
  NUM_TIMER_STATES
};

typedef enum{
  STATE_BUTTON_UP,
  STATE_BUTTON_DOWN,
  STATE_BUTTON_LONG,
} button_state;

enum win_light_state {
  STATE_WIN_LIGHT_WAIT,
  STATE_WIN_LIGHT_WON,
};

struct discrete_input {
  long debounce_delay;
  bool last_value;
  long count;
  //bool value;
  int pin;
  button_state state;
  int short_presses;
  int long_presses;
  unsigned long button_down_time;
  bool invert;
};

struct analog_gate {
  int pulse_index;
  volatile int16_t pulse_buffer[PULSE_BUFFER_SIZE];
  int last_raw_value;
  unsigned long break_time;
  int pin;
  int threshold;
  int valid_readings;
  bool value;
  char gate;
};

struct race_lane {
  struct analog_gate false_start_sensor;
  struct analog_gate finish_line_sensor;
  int reaction_time;
  long unsigned int finish_time;
  char title;
};


/****************
 *
 * GLOBAL VARS
 *
 ****************/


// receiving commands
bool stringComplete = false;
char buff[RX_BUFF_LEN] = {0};
int buff_idx = 0;

race_lane lane[NUMBER_LANES];

char* c = new char[80];

//struct analog_gate *analog_gates[ANALOG_CHANNEL_COUNT];

enum timer_state state;
enum win_light_state win_state;

unsigned long step_time;
bool test_last_value = false;
//int active_analog_channel;

discrete_input start_button;

int sensor_value = 0;
int cycle = 0;
int change_counter = 0;

volatile int current_channel = 0;
volatile bool light_state = false;

unsigned long sweep_time;
unsigned long last_time;
unsigned long last_lcd_update;

//const char* states[] = {
//  "STATE_IDLE",
//  "STATE_IDLE_WAIT",
//  "STATE_CALIBRATE_ANALOG",
//  "STATE_CALIBRATE_SETTLE",
//  "STATE_STARTING",
//  "STATE_STARTING_WAIT",
//  "STATE_RUNNING",
//  "STATE_RUNNING_WAIT",
//  "STATE_PRE_START_FAILED",
//  "STATE_FALSE_START",
//  "STATE_FINISHED",
//  "STATE_FINISHED_WAIT",
//  "STATE_TEST_GATE",
//  "STATE_TEST_CALIBRATE_ANALOG",
//  "STATE_TEST_MONITOR_ANALOG"
//};


/****************
 *
 * FUNCTIONS
 *
 ****************/

//void announce_state() {
//  static enum timer_state last;
//  static int first = 1;
//
//  if ((last != state) || first) {
//    Serial.print("STATE: "); Serial.println(states[state]);
//    first = 0;
//  }
//  last = state;
//}

void setup() {
  Serial.begin(9600); //119200
  // pin configurations:
  pinMode(TIMER_CLK_PIN, INPUT);
  pinMode(STARTER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LANE_1_FALSE_START_PIN, INPUT);
  pinMode(LANE_2_FALSE_START_PIN, INPUT);
  pinMode(LANE_1_FINISH_PIN, INPUT);
  pinMode(LANE_2_FINISH_PIN, INPUT);

  pinMode(GATE_LIGHT_PIN, OUTPUT);
  pinMode(STARTER_LIGHT_PIN, OUTPUT);
  pinMode(LANE_1_WIN_LIGHT_PIN, OUTPUT);
  pinMode(LANE_2_WIN_LIGHT_PIN, OUTPUT);


  rx8803_init(TIMER_CLK_PIN);
  lcd_init();



  start_button.debounce_delay = 50000; //us
  start_button.pin = STARTER_BUTTON_PIN;
  start_button.invert = true;
  start_button.short_presses = 0;
  start_button.long_presses = 0;
  start_button.state = STATE_BUTTON_UP;

  lane[0].false_start_sensor.pin = LANE_1_FALSE_START_PIN;
  lane[0].false_start_sensor.pulse_index = 0;
  lane[0].false_start_sensor.value = 0;
  lane[0].false_start_sensor.threshold = 0;
  lane[0].false_start_sensor.gate = 's';
  lane[0].false_start_sensor.valid_readings = 0;

  lane[0].finish_line_sensor.pin = LANE_1_FINISH_PIN;
  lane[0].finish_line_sensor.pulse_index = 0;
  lane[0].finish_line_sensor.value = 0;
  lane[0].finish_line_sensor.threshold = 0;
  lane[0].finish_line_sensor.gate = 'f';
  lane[0].finish_line_sensor.valid_readings = 0;

  lane[0].title = '1';

  lane[1].false_start_sensor.pin = LANE_2_FALSE_START_PIN;
  lane[1].false_start_sensor.pulse_index = 0;
  lane[1].false_start_sensor.value = 0;
  lane[1].false_start_sensor.threshold = 0;
  lane[1].false_start_sensor.gate = 's';
  lane[1].false_start_sensor.valid_readings = 0;

  lane[1].finish_line_sensor.pin = LANE_2_FINISH_PIN;
  lane[1].finish_line_sensor.pulse_index = 0;
  lane[1].finish_line_sensor.value = 0;
  lane[1].finish_line_sensor.threshold = 0;
  lane[1].finish_line_sensor.gate = 'f';
  lane[1].finish_line_sensor.valid_readings = 0;

  lane[1].title = '2';

  last_time = micros();
  winLightReset();
  lcd_message("MRGTimer v0.1");
  Serial.println("System Reset");
  
  state = STATE_IDLE;
  Serial.println("STATE: STATE_IDLE");
  
  delay(3000);
}


void loop() {
  bool b;

//  announce_state();

  //get the sweep time in us.
  sweep_time = micros() - last_time;
  last_time = micros();

  //condition the start button
  debounce(&start_button);

  
  if (start_button.long_presses > 0) {
    start_button.long_presses = 0;
    state = STATE_IDLE;
    Serial.println("STATE: STATE_IDLE");
  }

  //condition the inputs.
  for (int i = 0; i < NUMBER_LANES; i++) {
    switch(state) {
      case STATE_RUNNING:
      case STATE_RUNNING_WAIT:
        // only read start gate if we havnt crossed it yet
        if (lane[i].reaction_time == -1) {
          analogMonitor(&(lane[i].false_start_sensor));
        } else {
          analogMonitor(&(lane[i].finish_line_sensor));
        }
        break;
        
      case STATE_STARTING_WAIT:
        analogMonitor(&(lane[i].false_start_sensor));
        break;
        
      case STATE_IDLE:
      case STATE_IDLE_WAIT:  
      case STATE_STARTING:
      case STATE_CALIBRATE_ANALOG:
      case STATE_CALIBRATE_SETTLE:
      case STATE_TEST_GATE:
      case STATE_TEST_CALIBRATE_ANALOG:
      case STATE_TEST_MONITOR_ANALOG:
        analogMonitor(&(lane[i].finish_line_sensor));
        analogMonitor(&(lane[i].false_start_sensor));
        break;
      default:
        break;
    }
  }

  // flip the light.
  light_state = !light_state;// && (state != STATE_IDLE_WAIT);
  digitalWrite(GATE_LIGHT_PIN, light_state);
  delay(1); //wait for the light state to change **IMPORTANT**


  //*************************************************
  //State machine processing.
  //*************************************************

  
  if (STATE_IDLE == state) {
    winLightReset();
    start_button.long_presses = 0;
    start_button.short_presses = 0;

    state = STATE_IDLE_WAIT;
    Serial.println("STATE: STATE_IDLE_WAIT");
    
    digitalWrite(STARTER_LIGHT_PIN, HIGH);
    lcd_message("Ready");
    
  }

  else if (STATE_IDLE_WAIT == state) {
    if (0 < start_button.short_presses) {
      // Start button pressed, transition to the starting state

      state = STATE_CALIBRATE_ANALOG;
      lcd_message("Calibrating...");
      cycle = 0;
    }
  }

  else if (STATE_CALIBRATE_ANALOG == state) {
    cycle++; // number of cycles/samples collected
    if (cycle < PULSE_BUFFER_SIZE) {
      b = 1;
      for (int i = 0; i < NUMBER_LANES; i++ ) {
        calibrateAnalog(&lane[i].false_start_sensor);
        calibrateAnalog(&lane[i].finish_line_sensor);
        if(lane[i].false_start_sensor.threshold < 10){
          lcd_message("L%cS GATE FAULT");
          b = 0;
        }
        else if(lane[i].finish_line_sensor.threshold < 10){
          lcd_message("L%cF GATE FAULT");
          b = 0;
        }
      }
      if(!b){
        Serial.println("STATE: PRE_START_FAILED");
        state = STATE_PRE_START_FAILED;
      } else {
        state = STATE_CALIBRATE_SETTLE;
        Serial.println("STATE: STATE_CALIBRATE_SETTLE");
        
        lcd_message("Settling...");
        step_time = millis();
      }
    }
  }

  else if (STATE_CALIBRATE_SETTLE == state) {
    b = 1;
    for (int i = 0; i < NUMBER_LANES; i++) {
      b = b && (lane[i].finish_line_sensor.valid_readings == PULSE_BUFFER_SIZE) &&
        (lane[i].false_start_sensor.valid_readings == PULSE_BUFFER_SIZE);
    }
    if(b){
      state = STATE_STARTING;
      Serial.println("STATE: STATE_STARTING");
    }
  }

  else if (STATE_STARTING == state) {
    start_button.long_presses = 0;
    start_button.short_presses = 0;

    // Verify that all the lights are on and feedback is correct before starting the race.
    b = 1;
    for (int i = 0; i < NUMBER_LANES; i++ ) {
      if (!lane[i].finish_line_sensor.value) {
        sprintf(c, "lane %c finish line sensor obstructed", lane[i].title);
        lcd_message("L%c Finish fail");
        Serial.println(c);
        b = 0;
      }
      if (!lane[i].false_start_sensor.value) {
        sprintf(c, "lane %c false start sensor obstructed", lane[i].title);
        lcd_message("L%c Start fail");
        Serial.println(c);
        b = 0;
      }
    }

    if (b) {
      // All sensors reading correctly
      step_time = random(3, 5)*1000; // This is when the race will start.
      state = STATE_STARTING_WAIT;
      Serial.println("STATE: STATE_STARTING_WAIT");
      sprintf(c, "Start delay is %i ms", step_time);
      Serial.println(c);
      step_time += millis();

    } else {
      if (millis() > step_time) {
        step_time = millis() + ERROR_MESSAGE_MS;
        state = STATE_PRE_START_FAILED;
        Serial.println("STATE: STATE_STARTING - Sensors not reading correctly");
      }
    }
  }
  else if (STATE_STARTING_WAIT == state) {
    if (millis() > step_time) {
      // start the race.
      state = STATE_RUNNING;
      Serial.println("STATE: STATE_RUNNING");
    } else {
      unsigned long countdown = step_time - millis();
      sprintf(c, "%d...", (1000+countdown)/1000);
      lcd_message(c);
    }

    // on the lookout for false starts.
    for (int i = 0; i < NUMBER_LANES; i++ ) {
      if (0 == lane[i].false_start_sensor.value) {
        //print a message indicating false start
        sprintf(c, "L%c FALSE START", lane[i].title);
        Serial.println(c);
        
        state = STATE_FALSE_START;
        Serial.println("STATE: STATE_FALSE_START");
        
        step_time = millis() + ERROR_MESSAGE_MS;
        lcd_message(c);
      }
    }
  }

  else if (STATE_RUNNING == state) {
    lcd_message("GO!!!!!");
    start_button.long_presses = 0;
    start_button.short_presses = 0;

    for (int i = 0; i < NUMBER_LANES; i++ ) {
      lane[i].reaction_time = -1;
      lane[i].finish_time = ULONG_MAX;
    }

    state = STATE_RUNNING_WAIT;
    Serial.println("STATE: STATE_RUNNING_WAIT");
    step_time = millis();

    // drop the flag.
    digitalWrite(STARTER_LIGHT_PIN, LOW);
    rx8803_start_counter();
    last_lcd_update = 0;
  }

  else if (STATE_RUNNING_WAIT == state) {
    bool done = true;
    
    for (int i = 0; i < NUMBER_LANES; i++ ) {
      // Measure reaction time.
      if ((-1 == lane[i].reaction_time) && (0 == lane[i].false_start_sensor.value)) {
        lane[i].reaction_time = lane[i].false_start_sensor.break_time - step_time;
        sprintf(c, "Lane %c reacted in %d ms", lane[i].title, lane[i].reaction_time);
        Serial.println(c);
      }

      // Measure finish time.
      if ((ULONG_MAX == lane[i].finish_time) && (0 == lane[i].finish_line_sensor.value)){
          lane[i].finish_time = rx8803_get_count();
          winLightLatch(i);
          sprintf(c, "Lane %c: %d ms", lane[i].title, lane[i].finish_time);
          Serial.println(c);
          lcd_result(i, lane[i].finish_time);
      }

      // Make sure both lanes have finished.
      if (ULONG_MAX == lane[i].finish_time) {
        done = false;
      }
    }
    
    if (done) {
      rx8803_stop_counter();
      
      Serial.println("STATE: STATE_RUNNING_WAIT");
      state = STATE_FINISHED;
    } else {
      int time_row = 0;
      if (ULONG_MAX != lane[0].finish_time) {
        time_row = 1;
      }
      if((millis() - last_lcd_update) > 100){
        lcd_elapsed(time_row, rx8803_get_count());
        last_lcd_update = millis();
      }
    }
  }

  else if (STATE_FINISHED == state) {
    start_button.long_presses = 0;
    start_button.short_presses = 0;
    
    Serial.println("STATE: STATE_FINISHED_WAIT");
    state = STATE_FINISHED_WAIT;
  }

  else if (STATE_FINISHED_WAIT == state) {
    if (0 < start_button.short_presses) {
      // Start button pressed, transition to the starting state
      state = STATE_IDLE;
      Serial.println("STATE: STATE_IDLE");
    }
  }

  else if (STATE_PRE_START_FAILED == state) {
    lcd_message("Cal fail :(");
    // Not able to sucessfully see all the light gates.
    // ensure we are displaying the message for a while
    if (millis() > step_time) {
      state = STATE_IDLE;
      Serial.println("STATE: STATE_IDLE");
    }
  }

  else if (STATE_FALSE_START == state) {
    // Somebody jumped the gun.
    // lcd already has message, wait for keypress
    if (0 < start_button.short_presses) {
      // Start button pressed, transition to the starting state
      state = STATE_IDLE;
      Serial.println("STATE: STATE_IDLE");
    }
  }

  // this is populated by serialEvent()
  if (stringComplete) {
    processCommand();
    stringComplete = false;
  }
}

void calibrateAnalog(struct analog_gate *ag) {
  int i;
  int j;
  
  // find out the average difference reading.
  j = 0;
  for (i = 1; i < PULSE_BUFFER_SIZE; i++) {
      j += ag->pulse_buffer[i];
  }
  j /= PULSE_BUFFER_SIZE;

  // The threshold is the value of change at which the light is considered not visible.
  // probably should calculate the noise level on the signal and use that but that's for
  // another day.
  ag->threshold = j / 2;

  if (DEBUG) {
    sprintf(c, "calibration for analog pin: %i ", ag->pin);
    Serial.println(c);

    sprintf(c, "Gate: %c Threshold: %d ", ag->gate, ag->threshold);
    for (i = 0; i < PULSE_BUFFER_SIZE; i++) {
      j = (ag->pulse_index + PULSE_BUFFER_SIZE - i) % PULSE_BUFFER_SIZE;
      sprintf(c + strlen(c), "%d ", ag->pulse_buffer[j]);
    }
    Serial.println(c);
    ag->valid_readings = 0;
  }
}

void analogMonitor(struct analog_gate *ag) {
  int i;
  int k;

  // Record the current light level and store the difference between it and the last light level.
  // Store the current light level in the last_value for the next evaluation.

  i = analogRead(ag->pin);
  ag->pulse_index++;
  ag->pulse_index %= PULSE_BUFFER_SIZE;
  ag->pulse_buffer[ag->pulse_index] = abs(ag->last_raw_value - i);
  ag->last_raw_value = i;

  // Count the number of times that change is detected between samples.
  k = 0;
  for (i = 0; i < PULSE_BUFFER_SIZE; i++) {
    if (ag->pulse_buffer[i] > ag->threshold) {
      // Saw a big enough change in value
      k++;
    }
  }
  ag->valid_readings++;
  if(ag->valid_readings > PULSE_BUFFER_SIZE){
    ag->valid_readings = PULSE_BUFFER_SIZE;
    i = 1;
  }else{
    i = 0;
  }
  
  // See if we should transition from high to low.
  if(i && (ag->value == 1) && ((PULSE_BUFFER_SIZE - k) >  (PULSE_BUFFER_SIZE * 3 / 4))){
    // 3/4 of the buffer saw no change. flip.
    ag->value = 0;
    i = 1;
  }
  
  // See if we should transition from low to high.
  else if(i && (ag->value == 0) && k > (PULSE_BUFFER_SIZE * 3 / 4)){
    // 3/4 of the buffer saw change. flip.
    ag->value = 1;
    i = 1;
  }
  
  else{
    i = 0;
  }
  
  if(i && DEBUG){
    sprintf(c, "pin %d (%c) to %d k: %d", ag->pin, ag->gate, ag->value, k);
    Serial.println(c);
  }
}

void debounce(struct discrete_input *di) {
  bool b;
  b = digitalRead(di->pin);
  b = b ^ di->invert;

  if (b && di->last_value) {
    di->count += sweep_time;
  }
  else if (!b && di->last_value) {
    di->count -= sweep_time / 2;
  }
  else if (b && !di->last_value) {
    di->count += sweep_time /  2;
  }
  else if (!b && !di->last_value) {
    di->count -= sweep_time;
  }

  // clamp the di.count variable.
  if (di->count < 0) {
    di->count = 0;
  }
  if (di->count > di->debounce_delay) {
    di->count = di->debounce_delay;
  }

  // flip the bit if the count exceeds the threshold.
  if (STATE_BUTTON_DOWN == di->state) {
    // Check for a DOWN->UP transition
    if (0 == di->count) {
      //di->value = true;
      if (DEBUG) {
        sprintf(c, "DOWN->UP: Pin %i short press", di->pin);
        Serial.println(c);
      }
      di->short_presses++;
      di->state = STATE_BUTTON_UP;
    }

    // Check for a DOWN->LONG transition
    else if (di->debounce_delay == di->count) {
      if (millis() - di->button_down_time > LONG_PRESS_DURATION) {
        if (DEBUG) {
          sprintf(c, "DOWN->LONG: Pin %i - long press", di->pin);
          Serial.println(c);
        }
        di->long_presses++;
        di->state = STATE_BUTTON_LONG;
      }
    }
  }

  else if (STATE_BUTTON_UP == di->state) {
    // Check for a UP->DOWN transition.
    if (di->debounce_delay == di->count) {
      //di->value = false;
      if (DEBUG) {
        sprintf(c, "UP->DOWN: Digital pin %i to %d", di->pin, b);
        Serial.println(c);
      }
      di->state = STATE_BUTTON_DOWN;
      di->button_down_time = millis();
    }
  }

  else if (STATE_BUTTON_LONG == di->state) {
    // Check for a LONG->UP transition.
    if (0 == di->count) {
      //di->value = false;
      if (DEBUG) {
        sprintf(c, "LONG->UP: Digital pin %i to %d", di->pin, b);
        Serial.println(c);
      }
      di->state = STATE_BUTTON_UP;
    }
  }

  di->last_value = b;
}

// only turn on the winning late e.g. this is stateful
void winLightLatch(int i) {
  if (win_state == STATE_WIN_LIGHT_WAIT) {
    win_state = STATE_WIN_LIGHT_WON;
    digitalWrite( (i == 0) ? LANE_1_WIN_LIGHT_PIN : LANE_2_WIN_LIGHT_PIN , HIGH);
  }
}

void winLightReset() {
  win_state = STATE_WIN_LIGHT_WAIT;
  digitalWrite(LANE_1_WIN_LIGHT_PIN, LOW);
  digitalWrite(LANE_2_WIN_LIGHT_PIN, LOW);
}

// every command must be one line
void serialEvent() {
  static char lastChar;
  while (Serial.available()) {

    // get the new byte:
    char inChar = (char)Serial.read();

    // commands can end in \r or \n or any combo
    if (inChar == '\r' || inChar == '\n') {
      stringComplete = true;
    } else {
      buff[buff_idx] = inChar;
      buff_idx++;
      if (buff_idx == RX_BUFF_LEN) {
        // this really shouldnt happen....
        WTF("shouldnt see input buffer max out");
        reset_buff();
      }
    }
    // if the incoming character is a carriage return (0x0D), set a flag so the main loop can
    // do something about it:
    lastChar = inChar;
  }
}


void ack() {
  Serial.println("ACK");
}

void nack() {
  Serial.println("NACK");
}

void reset_buff() {
  memset(buff,0,RX_BUFF_LEN);
  buff_idx = 0;
}

void processCommand() {
  int new_state = 0;
  int valid = 0;

  int lane, light_state;

  unsigned long int ms;

  // "state x".  Set state to x.  must match enum
  // returns "ACK" on valid, "NACK" otherwise
  if (sscanf(buff, "state %d", &new_state) == 1)  {
    //Serial.print("Got state "); Serial.println(new_state);
    if ( new_state < NUM_TIMER_STATES ) {
      state = new_state;
      valid = 1;
      ack();
    }
  }

  // "start_light x ".  Set start light state (0 = off, 1 = on)
  // returns "ACK" on valid, "NACK" otherwise
  if (sscanf(buff, "start_light %d ", &light_state) == 1)  {
    //Serial.print("Got state "); Serial.println(new_state);
    if ( light_state == 1 || light_state == 0) {
      digitalWrite(STARTER_LIGHT_PIN, (light_state == 1) ? HIGH : LOW);
      valid = 1;
      ack();
    }
  }

  // "win_light x y".  Set win light state x (lane, 0 or 1) to y (0 = off, 1 = on)
  // returns "ACK" on valid, "NACK" otherwise
  if (sscanf(buff, "win_light %d %d", &lane, &light_state) == 2)  {
    if ((lane == 0 || lane == 1 ) && (light_state == 1 || light_state == 0)) {
      //Serial.print("lane: "); Serial.println(lane);
      //Serial.print("light_state: "); Serial.println(light_state);
      digitalWrite((lane == 0) ? LANE_1_WIN_LIGHT_PIN : LANE_2_WIN_LIGHT_PIN,
                   (light_state == 1) ? HIGH : LOW);
      valid = 1;
      ack();
    }
  }

  // "lcd_result x y".  show lane result (lane, 0 or 1), ms
  // returns "ACK" on valid, "NACK" otherwise
  if (sscanf(buff, "lcd_result %d %lu", &lane, &ms) == 2)  {
    if ((lane == 0 || lane == 1 )) {
      lcd_result(lane,ms);
      valid = 1;
      ack();
    }
  }

  // "beams": return beam readings
  // returns "a b c d" for beam detect l1start l1end l2start l2end as decimals
  if (strncmp(buff,"beams",5)==0) {
    Serial.print(analogRead(LANE_1_FALSE_START_PIN)); Serial.print(" ");
    Serial.print(analogRead(LANE_1_FINISH_PIN)); Serial.print(" ");
    Serial.print(analogRead(LANE_2_FALSE_START_PIN)); Serial.print(" ");
    Serial.println(analogRead(LANE_2_FINISH_PIN));
    valid = 1;
  }



  // "get_state": return the current state
  // returns "state x" where x is decimal rep of state
  if (strncmp(buff,"get_state",9)==0) {
    Serial.print("state ");Serial.println(state);
    valid = 1;
  }

  // "start_timer": start the race timer
  // resets timer count
  if (strncmp(buff,"start_timer",11) == 0) {
    rx8803_start_counter();
    ack();
    valid = 1;
  }

  // "stop_timer": stop the race timer
  if (strncmp(buff,"stop_timer",10) == 0) {
    rx8803_stop_counter();
    ack();
    valid = 1;
  }

  // "lcd_clear": stop the race timer
  if (strncmp(buff,"lcd_clear",9) == 0) {
    lcd_clear();
    ack();
    valid = 1;
  }


  // "get_timer_count": return the current timer count
  if (strncmp(buff,"get_timer_count",16) == 0) {
    Serial.print("time: "); Serial.print( (rx8803_get_count()*1000)/32768); Serial.println(" ms");
    ack();
    valid = 1;
  }


  if (!valid) {
    nack();
  }

  // cleanup
  reset_buff();
}
