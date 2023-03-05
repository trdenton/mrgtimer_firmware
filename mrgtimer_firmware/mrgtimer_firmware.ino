#include <string.h>
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
#define STARTER_LIGHT_PIN 3   // Indicate that the race has started
#define GATE_LIGHT_PIN 4      // This is what drives the sensor LEDs

#define LANE_1_FALSE_START_PIN A0
#define LANE_2_FALSE_START_PIN A1

#define LANE_1_FINISH_PIN A2
#define LANE_2_FINISH_PIN A3

#define LANE_1_WIN_LIGHT_PIN A6
#define LANE_2_WIN_LIGHT_PIN A7

#define LONG_PRESS_DURATION 2000

//Debug symbols.
#define DEBUG 0
#define LOG_DEBUG 9
#define LOG_EVENTS 8

#define RX_BUFF_LEN (256)

/**************************
 *
 * STRUCTS, ENUMS
 *
 *************************/

enum timer_state {
  STATE_IDLE,             // 0  
  STATE_IDLE_WAIT,        // 1
  STATE_CALIBRATE_ANALOG, // 2
  STATE_STARTING,         // 3
  STATE_STARTING_WAIT,    // 4
  STATE_RUNNING,          // 5
  STATE_RUNNING_WAIT,     // 6
  STATE_PRE_START_FAILED, // 7
  STATE_FALSE_START,      // 8
  STATE_FINISHED,         // 9
  STATE_FINISHED_WAIT,    // 10
  STATE_TEST_GATE,        // 11
  STATE_TEST_CALIBRATE_ANALOG,  // 12
  STATE_TEST_MONITOR_ANALOG,    // 13
  NUM_TIMER_STATES
};

enum button_state {
  STATE_BUTTON_UP,
  STATE_BUTTON_DOWN,
  STATE_BUTTON_LONG,
};

struct discrete_input {
  long debounce_delay;
  bool last_value;
  long count;
  bool value;
  int pin;
  int state;
  int short_presses;
  int long_presses;
  unsigned long button_down_time;
  bool invert;
};

struct analog_gate {
  int pulse_index;
  volatile int16_t pulse_buffer[PULSE_BUFFER_SIZE];
  int threshold;
  unsigned long break_time;
  int pin;
  bool calibrate;
  bool value;
  char gate;
};

struct race_lane {
  struct analog_gate false_start_sensor;
  struct analog_gate finish_line_sensor;
  int reaction_time;
  int finish_time;
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

const char* states[] = {
  "STATE_IDLE",
  "STATE_IDLE_WAIT",
  "STATE_CALIBRATE_ANALOG",
  "STATE_STARTING",
  "STATE_STARTING_WAIT",
  "STATE_RUNNING",
  "STATE_RUNNING_WAIT",
  "STATE_PRE_START_FAILED",
  "STATE_FALSE_START",
  "STATE_FINISHED",
  "STATE_FINISHED_WAIT",
  "STATE_TEST_GATE",
  "STATE_TEST_CALIBRATE_ANALOG",
  "STATE_TEST_MONITOR_ANALOG"
};


/****************
 *
 * FUNCTIONS
 *
 ****************/

void announce_state() {
  static enum timer_state last;
  static int first = 1;

  if ((last != state) || first) {
    Serial.print("STATE: "); Serial.println(states[state]);
    first = 0;
  }
  last = state;
}

void setup() {
  Serial.begin(9600); //119200
  // pin configurations:
  pinMode(TIMER_CLK_PIN, INPUT);
  pinMode(STARTER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LANE_1_FALSE_START_PIN, INPUT_PULLUP);
  pinMode(LANE_2_FALSE_START_PIN, INPUT_PULLUP);
  pinMode(LANE_1_FINISH_PIN, INPUT_PULLUP);
  pinMode(LANE_2_FINISH_PIN, INPUT_PULLUP);

  pinMode(GATE_LIGHT_PIN, OUTPUT);
  pinMode(STARTER_LIGHT_PIN, OUTPUT);
  pinMode(LANE_1_WIN_LIGHT_PIN, OUTPUT);
  pinMode(LANE_2_WIN_LIGHT_PIN, OUTPUT);


  rx8803_init(TIMER_CLK_PIN);
  lcd_init();

  state = STATE_IDLE;

  start_button.debounce_delay = 50000; //us
  start_button.pin = STARTER_BUTTON_PIN;
  start_button.invert = true;
  start_button.short_presses = 0;
  start_button.long_presses = 0;
  start_button.state = STATE_BUTTON_UP;

  lane[0].false_start_sensor.pin = LANE_1_FALSE_START_PIN;
  lane[0].false_start_sensor.pulse_index = 0;
  lane[0].false_start_sensor.value = true;
  lane[0].false_start_sensor.threshold = 0;
  lane[0].false_start_sensor.gate = 's';

  lane[0].finish_line_sensor.pin = LANE_1_FINISH_PIN;
  lane[0].finish_line_sensor.pulse_index = 0;
  lane[0].finish_line_sensor.value = true;
  lane[0].finish_line_sensor.threshold = 0;
  lane[0].finish_line_sensor.gate = 'f';

  lane[0].title = '1';

  last_time = micros();
  lcd_message("MRGTimer v0.1");
  delay(3000);
}


void loop() {
  bool b;

  announce_state();

  //get the sweep time in us.
  sweep_time = micros() - last_time;
  last_time = micros();

  //condition the start button
  debounce(&start_button);

  //condition the inputs.
  for (int i = 0; i < NUMBER_LANES; i++) {
    analogMonitor(&(lane[i].finish_line_sensor));
    analogMonitor(&(lane[i].false_start_sensor));
  }

  //flip the light.
  light_state = !light_state && state != STATE_IDLE_WAIT;
  digitalWrite(GATE_LIGHT_PIN, light_state);
  delay(1); //wait for the light to come on **IMPORTANT**


  //*************************************************
  //State machine processing.
  //*************************************************
  if (STATE_IDLE == state) {
    digitalWrite(STARTER_LIGHT_PIN, HIGH);
    start_button.long_presses = 0;
    start_button.short_presses = 0;

    state = STATE_IDLE_WAIT;
    lcd_message("IDLE");
  }

  else if (STATE_IDLE_WAIT == state) {
    if (0 < start_button.short_presses) {
      lcd_message("Calibrating...");
      // Start button pressed, transition to the starting state

      state = STATE_CALIBRATE_ANALOG;
      cycle = 0;
    }
  }

  else if (STATE_CALIBRATE_ANALOG == state) {
    cycle++;
    if (cycle > PULSE_BUFFER_SIZE) {
      for (int i = 0; i < NUMBER_LANES; i++ ) {
        calibrateAnalog(&lane[i].finish_line_sensor);
        calibrateAnalog(&lane[i].false_start_sensor);
      }
      state = STATE_STARTING;
      step_time = millis();
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
        Serial.println(c);
        b = 0;
      }
      if (!lane[i].false_start_sensor.value) {
        sprintf(c, "lane %c false start sensor obstructed", lane[i].title);
        Serial.println(c);
        b = 0;
      }
    }

    if (b) {
      // All sensors reading correctly
      step_time = random(3000, 5000); // This is when the race will start.[]6
      state = STATE_STARTING_WAIT;
      sprintf(c, "Start delay is %i ms", step_time);
      Serial.println(c);
      step_time += millis();

    } else {
      if (millis() > step_time) {
        state = STATE_PRE_START_FAILED;
        Serial.println("STATE: STATE_STARTING - Sensors not reading correctly");
        //Write a handy message detailing why we failed.
      }
    }
  }

  else if (STATE_STARTING_WAIT == state) {
    if (millis() > step_time) {
      // start the race.
      state = STATE_RUNNING;
    }

    // on the lookout for false starts.
    for (int i = 0; i < NUMBER_LANES; i++ ) {
      if (0 == lane[i].false_start_sensor.value) {
        //print a message indicating false start
        sprintf(c, "Lane: %c FALSE START", lane[i].title);
        Serial.println(c);
        state = STATE_FALSE_START;
      }
    }
  }

  else if (STATE_RUNNING == state) {
    start_button.long_presses = 0;
    start_button.short_presses = 0;

    for (int i = 0; i < NUMBER_LANES; i++ ) {
      lane[i].reaction_time = -1;
      lane[i].finish_time = -1;
    }

    state = STATE_RUNNING_WAIT;
    step_time = millis();

    // drop the flag.
    digitalWrite(STARTER_LIGHT_PIN, LOW);
  }

  else if (STATE_RUNNING_WAIT == state) {
    bool done = true;

    if (start_button.long_presses > 0) {
      state = STATE_IDLE;
    }

    for (int i = 0; i < NUMBER_LANES; i++ ) {
      // measure the reaction time
      // sprintf(c, "Lane %c false start line: %d", lane[i].title, lane[i].false_start_sensor.value);
      // Serial.println(c);

      if (0 == lane[i].false_start_sensor.value) {
        if (-1 == lane[i].reaction_time) {
          lane[i].reaction_time = lane[i].false_start_sensor.break_time - step_time;
          sprintf(c, "Lane %c reacted in %d ms", lane[i].title, lane[i].reaction_time);
          Serial.println(c);
        }
      }

      // measure the finish time.
      // sprintf(c, "Lane %c finish line: %d", lane[i].title, lane[i].finish_line_sensor.value);
      // Serial.println(c);

      if (0 == lane[i].finish_line_sensor.value) {
        if (-1 == lane[i].finish_time) {
          lane[i].finish_time = lane[i].finish_line_sensor.break_time - step_time;
          sprintf(c, "Lane %c finished in %d ms", lane[i].title, lane[i].finish_time);
          Serial.println(c);
        }
      }

      if (-1 == lane[i].finish_time) {
        done = false;
      }
    }
    if (done) {
      state = STATE_FINISHED;
    }
  }

  else if (STATE_FINISHED == state) {
    start_button.long_presses = 0;
    start_button.short_presses = 0;
    state = STATE_FINISHED_WAIT;
  }

  else if (STATE_FINISHED_WAIT == state) {
    if (0 < start_button.short_presses) {
      // Start button pressed, transition to the starting state
      state = STATE_IDLE;
    }
  }

  else if (STATE_PRE_START_FAILED == state) {
    // Not able to sucessfully see all the light gates.
    state = STATE_IDLE;
  }

  else if (STATE_FALSE_START == state) {
    // Somebody jumped the gun.
    state = STATE_IDLE;
  }

  // this is populated by serialEvent()
  if (stringComplete) {
    processCommand();
    stringComplete = false;
  }
}

void calibrateAnalog(struct analog_gate *ag) {
  int max_reading = ag->pulse_buffer[0];
  int min_reading = ag->pulse_buffer[0];
  int i;
  int j;

  for (i = 1; i < PULSE_BUFFER_SIZE; i++) {
    if (ag->pulse_buffer[i] > max_reading) {
      max_reading = ag->pulse_buffer[i];
    }
    if (ag->pulse_buffer[i] < min_reading) {
      min_reading = ag->pulse_buffer[i];
    }
  }
  ag->threshold = (max_reading - min_reading) * 3 / 4;

  if (DEBUG) {
    sprintf(c, "calibration for analog pin: %i ", ag->pin);
    Serial.println(c);

    sprintf(c, "V: %d  %d (%c)- ", ag->value, ag->threshold, ag->gate);
    for (i = 0; i < PULSE_BUFFER_SIZE; i++) {
      j = (ag->pulse_index + PULSE_BUFFER_SIZE - i) % PULSE_BUFFER_SIZE;
      sprintf(c + strlen(c), "%d ", ag->pulse_buffer[j]);
    }
    Serial.println(c);

    sprintf(c, "min: %i max: %i", min_reading, max_reading);
    Serial.println(c);
  }
}

void analogMonitor(struct analog_gate *ag) {
  int avg_all;
  int avg_low;
  int i;
  int k;
  int j;

  ag->pulse_index++;
  ag->pulse_index %= PULSE_BUFFER_SIZE;
  ag->pulse_buffer[ag->pulse_index] = analogRead(ag->pin);

  if (ag->threshold > 0) {
    // determine the average value
    avg_all = 0;
    for (i = 0; i < PULSE_BUFFER_SIZE; i++) {
      avg_all += ag->pulse_buffer[i];
    }

    avg_all /= PULSE_BUFFER_SIZE;
    avg_low += ag->threshold / 4;

    // determine the average of the samples below the threshold.
    k = 0;
    for (i = 0; i < PULSE_BUFFER_SIZE; i++) {
      if (ag->pulse_buffer[i] < avg_low) {
        j += ag->pulse_buffer[i];
        k++;
      }
    }
    avg_low = j / k;

    // determine when the last sample to be above 75% of the threshold.
    k = 0;
    for (i = 0; i < PULSE_BUFFER_SIZE; i++) {
      j = (ag->pulse_index + PULSE_BUFFER_SIZE - i) % PULSE_BUFFER_SIZE; //decrement the counter with rollover
      if (ag->pulse_buffer[j] > avg_low + ag->threshold) {
        //sprintf(c, "index %i: %i > threshold %i, %i readings old (started at %i)", j, ag->pulse_buffer[j], ag->threshold * 3 / 4 + avg_low, k, i);
        //Serial.println(c);
        break;
      }
      k++;
    }

    //Check and see if we've seen a recent pulse
    if (k > 6) {
      ag->break_time = millis();
      ag->value = 0;
      if (DEBUG && 1 == ag->value) {
        sprintf(c, "pin %d (%c) to FALSE", ag->pin, ag->gate);
        Serial.println(c);
      }
    } else {
      ag->value = 1;
      if (DEBUG && 0 == ag->value) {
        sprintf(c, "pin %d (%c) to TRUE", ag->pin, ag->gate);
        Serial.println(c);
      }
    }
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

  // flip the bit if the count exceeds the thresdhold.
  if (STATE_BUTTON_DOWN == di->state) {
    if (di->debounce_delay == di->count) {
      di->value = true;
      if (DEBUG) {
        sprintf(c, "Pin %i short press", di->pin);
        Serial.println(c);
      }
      di->state = STATE_BUTTON_UP;
      di->short_presses++;
    }

    else if (0 == di->count) {
      if (millis() - di->button_down_time > LONG_PRESS_DURATION) {
        if (DEBUG) {
          sprintf(c, "Pin %i - long press", di->pin);
          Serial.println(c);
        }
        di->long_presses++;
        di->state = STATE_BUTTON_LONG;
      }
    }
  }

  else if (STATE_BUTTON_UP == di->state) {
    if (0 == di->count) {
      di->value = false;
      if (DEBUG) {
        sprintf(c, "Digital pin %i to %d", di->pin, di->value);
        Serial.println(c);
      }
      di->state = STATE_BUTTON_DOWN;
      di->button_down_time = millis();
    }
  }

  else if (STATE_BUTTON_LONG == di->state) {
    if (di->debounce_delay == di->count && !di->value) {
      di->value = true;
      if (DEBUG) {
        sprintf(c, "Digital pin %i to %d", di->pin, di->value);
        Serial.println(c);
      }
      di->state = STATE_BUTTON_UP;
    }
  }

  di->last_value = b;
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
      digitalWrite(STARTER_LIGHT_PIN, (light_state == 1) ? LOW : HIGH);
      valid = 1;
      ack();
    }
  }

  // "win_light x y".  Set win light state x (lane, 0 or 1) to y (0 = off, 1 = on)
  // returns "ACK" on valid, "NACK" otherwise
  if (sscanf(buff, "win_light %d %d", &lane, &light_state) == 2)  {
    if ((lane == 0 || lane == 1 ) && (light_state == 1 || light_state == 0)) {
      digitalWrite((lane == 0) ? LANE_1_WIN_LIGHT_PIN : LANE_2_WIN_LIGHT_PIN,
                   (light_state == 1) ? LOW : HIGH);
      valid = 1;
      ack();
    }
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

  // "get_timer_count": return the current timer count
  if (strncmp(buff,"get_timer_count",16) == 0) {
    Serial.print("time: "); Serial.print( rx8803_get_count()/33); Serial.println(" ms");
    ack();
    valid = 1;
  }


  if (!valid) {
    nack();
  }

  // cleanup
  reset_buff();
}
