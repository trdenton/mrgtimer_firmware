#include <string.h>

#define PULSE_BUFFER_SIZE 16
#define NUMBER_LANES 1

#define GATE_LIGHT_PIN 3
#define STARTER_LIGHT_PIN 4
#define STARTER_BUTTON_PIN 5

#define LIGHT_FALSE_START_PIN 6
#define LANE_1_FALSE_START_PIN 1
#define LANE_2_FALSE_START_PIN 2

#define LIGHT_FINISH_PIN 9
#define LANE_1_FINISH_PIN 1
#define LANE_2_FINISH_PIN 4

#define STATE_IDLE 0
#define STATE_IDLE_WAIT 1
#define STATE_CALIBRATE_ANALOG 2
#define STATE_STARTING 3
#define STATE_STARTING_WAIT 4
#define STATE_RUNNING 5
#define STATE_RUNNING_WAIT 6
#define STATE_PRE_START_FAILED 7
#define STATE_FALSE_START 8
#define STATE_FINISHED 9
#define STATE_FINISHED_WAIT 10
#define STATE_TEST_GATE 11
#define STATE_TEST_CALIBRATE_ANALOG 12
#define STATE_TEST_MONITOR_ANALOG 13

#define STATE_BUTTON_UP 0
#define STATE_BUTTON_DOWN 1
#define STATE_BUTTON_LONG 2

#define LONG_PRESS_DURATION 2000

//Debug symbols.
#define DEBUG 0
#define LOG_DEBUG 9
#define LOG_EVENTS 8


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

race_lane lane[NUMBER_LANES];
char* c = new char[80];

//struct analog_gate *analog_gates[ANALOG_CHANNEL_COUNT];

int state;

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

void setup() {
  Serial.begin(2000000);
  // pin configurations:
  pinMode(STARTER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LANE_1_FALSE_START_PIN, INPUT_PULLUP);
  pinMode(LANE_2_FALSE_START_PIN, INPUT_PULLUP);
  pinMode(LANE_1_FINISH_PIN, INPUT_PULLUP);
  pinMode(LANE_2_FINISH_PIN, INPUT_PULLUP);

  pinMode(GATE_LIGHT_PIN, OUTPUT);
  pinMode(STARTER_LIGHT_PIN, OUTPUT);

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
}


void loop() {
  bool b;

  //get the sweep time in us.
  sweep_time = micros() - last_time;
  last_time = micros();

  //sprintf(c, "Sweep: %u", sweep_time);
  //Serial.println(c);

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
        sprintf(c, "lane %c false start sensor obstructed");
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

  /*
    sprintf(c, "ch %d ", ag->pin);
    for (int i = 0; i < PULSE_BUFFER_SIZE; i++) {
    k = (ag->pulse_index + PULSE_BUFFER_SIZE - i) % PULSE_BUFFER_SIZE;
    sprintf(c + strlen(c), "%d ", ag->pulse_buffer[k]);
    }
    sprintf(c + strlen(c), " (%02d) -  ", ag->pulse_index);
    Serial.println(c);
  */

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
    //sprintf(c, "cycles since last pulse: %d", k);
    //Serial.println(c);
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
    //sprintf(c, "added %lu to pin %d count. now: %ld", sweep_time, di->pin, di->count);
    //Serial.println(c);
  }
  else if (!b && di->last_value) {
    di->count -= sweep_time / 2;
  }
  else if (b && !di->last_value) {
    di->count += sweep_time /  2;
  }
  else if (!b && !di->last_value) {
    di->count -= sweep_time;
    //sprintf(c, "subtracted %lu to pin %d count. now: %ld", sweep_time, di->pin, di->count);
    //Serial.println(c);
  }

  // clamp the di.count variable.
  if (di->count < 0) {
    di->count = 0;
  }
  if (di->count > di->debounce_delay) {
    di->count = di->debounce_delay;
  }

  // flip the bit if the count exceeds the thresdhold.
  //sprintf(c, "Pin %d count is: %ld, value is %ld", di->pin, di->count, di->value);
  //sprintf(c, "count is %ld", di->count);
  //Serial.println(c);
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


/*
  lane[1].false_start_sensor.pin = LANE_2_FALSE_START_PIN;
  lane[1].false_start_sensor.pulse_index = 0;
  lane[1].false_start_sensor.value = true;

  lane[1].finish_line_sensor.pin = LANE_2_FINISH_PIN;
  lane[1].finish_line_sensor.pulse_index = 0;
  lane[1].finish_line_sensor.value = true;
  lane[1].title = '2';
*/

/*
  //pinMode(11, OUTPUT);
  //TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  //TCCR2B = _BV(CS22);
  //OCR2A = 200;
  //OCR2B = 60;

  // Init ADC free-run mode; f = ( 16MHz/prescaler ) / 13 cycles/conversion
  ADMUX  = analog_gates[0]->pin; // Channel sel, right-adj, use AREF pin
  ADCSRA = _BV(ADEN)  | // ADC enable
         _BV(ADSC)  | // ADC start
         _BV(ADATE) | // Auto trigger
         _BV(ADIE)  | // Interrupt enable
         _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0); // 128:1 / 13 = 9615 Hz
  ADCSRB = 0;                // Free run mode, no high MUX bit

  DIDR0      = 1 << LANE_1_FALSE_START_PIN // Turn off digital input for ADC pins
           | 1 << LANE_1_FINISH_PIN
           | 1 << LANE_2_FALSE_START_PIN
           | 1 << LANE_2_FINISH_PIN;

  TIMSK0 = 0;  // Timer0 off

  sei(); // Enable interrupts
*/

/*
  Sampling interupt
   - Cycles through the sensor pins, taking FFT_N audio samples and a single
     sample of all others.
*/

/*ISR(ADC_vect) {

  digitalWrite(GATE_LIGHT_PIN, light_state);

  analog_gates[current_channel]->pulse_index++;
  analog_gates[current_channel]->pulse_index %= PULSE_BUFFER_SIZE;
  analog_gates[current_channel]->pulse_buffer[analog_gates[current_channel]->pulse_index] = ADC;
  analog_gates[current_channel]->processed = false; //queue this channel for state re-processing

  if (!light_state) {
    //light is now off, go to next channel.

    //change channels
    current_channel++;
    current_channel %= ANALOG_CHANNEL_COUNT;
    light_state = false;

    // Turn off interrupts to report back and switch pins
    ADCSRA &= ~_BV(ADIE);
    ADMUX = bit (REFS0) | (analog_gates[current_channel]->pin & 0x07);
    ADCSRA |= _BV(ADIE);
  }
*/

/*
   else if (STATE_TEST_CALIBRATE_ANALOG == state) {
   delay(1);
   cycle++;
   if (cycle > PULSE_BUFFER_SIZE) {
     calibrateAnalog(&lane[0].false_start_sensor);
     state = STATE_TEST_MONITOR_ANALOG;
     Serial.println("STATE: STATE_TEST_MONITOR_ANALOG");
   }
   }

   else if (STATE_TEST_MONITOR_ANALOG == state) {
   if (lane[0].false_start_sensor.value != test_last_value) {
     test_last_value = lane[0].false_start_sensor.value;

     sprintf(c, "%lu - V: %d (%d) %d - ", current_time, lane[0].false_start_sensor.value, change_counter, lane[0].false_start_sensor.threshold);
     for (int i = 0; i < PULSE_BUFFER_SIZE; i++) {
       int j = (lane[0].false_start_sensor.pulse_index + PULSE_BUFFER_SIZE + i -1) % PULSE_BUFFER_SIZE;
       sprintf(c + strlen(c), "%d ", lane[0].false_start_sensor.pulse_buffer[j]);
     }
     Serial.println(c);
     change_counter++;
   }


   if (0 < start_button.short_presses) {
     start_button.short_presses = 0;
     Serial.println("STATE: IDLE");
     state = STATE_IDLE_WAIT;
   }
   }
*/


