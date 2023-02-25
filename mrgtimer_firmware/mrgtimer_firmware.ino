#include <string.h>
#include "rx8803.h"
#include "lcd.h"

/****************
 *
 * DEFINES
 *
 ****************/


#define PULSE_BUFFER_SIZE 16
#define NUMBER_LANES 1

#define TIMER_CLK_PIN 2

#define GATE_LIGHT_PIN 3
#define STARTER_LIGHT_PIN 4
#define STARTER_BUTTON_PIN 9

#define LIGHT_FALSE_START_PIN 6
#define LANE_1_FALSE_START_PIN 1
#define LANE_2_FALSE_START_PIN 2 // TODO i dont think this matches current schem

#define LIGHT_FINISH_PIN 9
#define LANE_1_FINISH_PIN 1
#define LANE_2_FINISH_PIN 4

#define LONG_PRESS_DURATION 2000

//Debug symbols.
#define DEBUG 0
#define LOG_DEBUG 9
#define LOG_EVENTS 8

/**************************
 *
 * STRUCTS, ENUMS
 *
 *************************/

enum timer_state {
  STATE_IDLE,
  STATE_IDLE_WAIT,
  STATE_CALIBRATE_ANALOG,
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


/****************
 *
 * FUNCTIONS
 *
 ****************/

void setup() {
  Serial.begin(9600);
  // pin configurations:
  pinMode(TIMER_CLK_PIN, INPUT);
  pinMode(STARTER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LANE_1_FALSE_START_PIN, INPUT_PULLUP);
  pinMode(LANE_2_FALSE_START_PIN, INPUT_PULLUP);
  pinMode(LANE_1_FINISH_PIN, INPUT_PULLUP);
  pinMode(LANE_2_FINISH_PIN, INPUT_PULLUP);

  pinMode(GATE_LIGHT_PIN, OUTPUT);
  pinMode(STARTER_LIGHT_PIN, OUTPUT);

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
    Serial.println("STATE: IDLE");
  }

  else if (STATE_IDLE_WAIT == state) {
    if (0 < start_button.short_presses) {
      // Start button pressed, transition to the starting state

      state = STATE_CALIBRATE_ANALOG;
      Serial.println("STATE: STATE_CALIBRATE_ANALOG");
      lcd_message("Calibrating...");
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
      Serial.println("STATE: STARTING");
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

    Serial.println("STATE: RUNNING");
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
      Serial.println("STATE: FINISHED");
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
