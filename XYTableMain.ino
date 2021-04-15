// XYTableMain.ino
// Authors: Maxx Wilson
// Initial Creation Date: 4/13/21
// Decription: Arduino Code to run ReNeu XY Table, including bounding limit switches
// on both axes, two DC Motors w/ PWM and Direction Control, and an Electromagnet.
// Inputs include a button to actuate electromagnet and a two channel analog joystick.
// Last Revision: 4/14/21

#include <stdint.h>
#include <stdbool.h>

//#define SERIAL_DEBUG true

// Actuator Pins
#define X_MOTOR_FWD_PIN 0
#define X_MOTOR_BWD_PIN 1
#define X_MOTOR_ENA_PIN 3

#define Y_MOTOR_FWD_PIN 2
#define Y_MOTOR_BWD_PIN 4
#define Y_MOTOR_ENA_PIN 5

#define EM_FWD_PIN 2
#define EM_BWD_PIN 4
#define EM_ENA_PIN 5

// Limit Switch Pins
#define X_RIGHT_LIM_PIN 9
#define X_LEFT_LIM_PIN 10

#define Y_LOWER_LIM_PIN 11
#define Y_UPPER_LIM_PIN 12

// User Input Pins
#define CLICKPEN_BTN_PIN 13

#define JOYSTICK_Y_CHANNEL_PIN A4
#define JOYSTICK_X_CHANNEL_PIN A5

// Switch Debounce Variables
#define DEBOUNCE_TIME 10 // Milliseconds
#define EM_PULSE_DURATION 500
bool debouncing = false;
bool pulse = false;
bool first_press = true;
uint16_t debounce_start_time = 0;
uint16_t pulse_start_time = 0;

// Actuator Variables
#define X_MOTOR_MIN_POWER 100.
#define Y_MOTOR_MIN_POWER 100.

int16_t x_motor_speed = 0;
int16_t y_motor_speed = 0;

typedef struct ActuatorOut {
  uint8_t fwd_pin;
  uint8_t bwd_pin;
  uint8_t ena_pin;
} AcuatorOut;

const ActuatorOut X_MOTOR_PINS = {X_MOTOR_FWD_PIN, X_MOTOR_BWD_PIN, X_MOTOR_ENA_PIN};
const ActuatorOut Y_MOTOR_PINS = {Y_MOTOR_FWD_PIN, Y_MOTOR_BWD_PIN, Y_MOTOR_ENA_PIN};
const ActuatorOut ELECTROMAGNET_PINS = {EM_FWD_PIN, EM_BWD_PIN, EM_ENA_PIN};

// User Input Definitions and Variables
#define JOYSTICK_THRESHOLD 50
#define JOYSTICK_INIT_SAMPLE_COUNT 64

int16_t joystick_x_channel;
int16_t joystick_y_channel;

int8_t x_channel_sign = 0;
int8_t y_channel_sign = 0;

float j2m_scale_x = (255. - X_MOTOR_MIN_POWER) / (512. - JOYSTICK_THRESHOLD);
float j2m_scale_y = (255. - Y_MOTOR_MIN_POWER) / (512. - JOYSTICK_THRESHOLD);
int16_t j2m_offset_x = X_MOTOR_MIN_POWER - JOYSTICK_THRESHOLD * j2m_scale_x;
int16_t j2m_offset_y = Y_MOTOR_MIN_POWER - JOYSTICK_THRESHOLD * j2m_scale_y;

//------------ActuatorInit-----------
// Function to initialize the direction and enable pins of an actuator.
// Inputs:  actuator      struct representing actuator configuration
// Outputs: none
void ActuatorInit(const ActuatorOut *actuator) {
  pinMode((*actuator).fwd_pin, OUTPUT);
  pinMode((*actuator).bwd_pin, OUTPUT);
  pinMode((*actuator).ena_pin, OUTPUT);
}

//------------ClipValue-----------
// Function to bound values between two limits. If value is higher/lower than a limit, it is set to that limit.
// Inputs:  *val          pointer to current value
//          upper_lim     upper limit for value
//          lower_lim     lower limit for value
// Outputs: none
void ClipValue(int16_t *val, int16_t lower_lim, int16_t upper_lim) {
  (*val) = (*val > upper_lim) ? upper_lim : *val;
  (*val) = (*val < lower_lim) ? lower_lim : *val;
}

//------------SetActuator-----------
// Function to set the direction and magnitude of an actuator. Negative magnitude implies
// Counter-Clockwise rotation for a motor and opposing polarity for an electromagnet
// Inputs:  mag           magnitude of actuator effort between -255 and 255
//          actuator      struct representing actuator configuration
// Outputs: none
void SetActuator(int16_t mag, const ActuatorOut *actuator) {
  // Clip to 8-bit value
  ClipValue(&mag, -255, 255);

  if (mag > 0) { // Forward Direction
    digitalWrite((*actuator).bwd_pin, LOW);
    digitalWrite((*actuator).fwd_pin, HIGH);
  }
  else if (mag < 0) { // Backward Direction
    digitalWrite((*actuator).fwd_pin, LOW);
    digitalWrite((*actuator).bwd_pin, HIGH);
  }
  else { // Zero Magnitude
    digitalWrite((*actuator).fwd_pin, LOW);
    digitalWrite((*actuator).bwd_pin, LOW);
  }

  // Send actuator command
  analogWrite((*actuator).ena_pin, abs(mag));
}

void setup() {

  #ifdef SERIAL_DEBUG
  Serial.begin(9600);
  #endif

  /*
    // Determine joystick resting values
    for (int i = 0; i < JOYSTICK_INIT_SAMPLE_COUNT; i++) {
    joystick_x_offset += analogRead(JOYSTICK_X_CHANNEL_PIN);
    joystick_y_offset += analogRead(JOYSTICK_Y_CHANNEL_PIN);
    }

    joystick_x_offset /= JOYSTICK_INIT_SAMPLE_COUNT;
    joystick_y_offset /= JOYSTICK_INIT_SAMPLE_COUNT;
  */
  
  // Initialize each actuator with two directional pins and an enable pin. The enable pin determines the
  // speed of the motor, the directional pins determine which way it rotates. Direction pins should NEVER both be high.
  ActuatorInit(&X_MOTOR_PINS);
  ActuatorInit(&Y_MOTOR_PINS);
  ActuatorInit(&ELECTROMAGNET_PINS);

  // Initialize each switch with input pullup for negative logic, meaning when you press the button, the voltage goes low (GND)
  pinMode(Y_UPPER_LIM_PIN, INPUT_PULLUP);
  pinMode(Y_LOWER_LIM_PIN, INPUT_PULLUP);
  pinMode(X_LEFT_LIM_PIN, INPUT_PULLUP);
  pinMode(X_RIGHT_LIM_PIN, INPUT_PULLUP);
  //pinMode(CLICKPEN_BTN_PIN, INPUT_PULLUP);

  // Initialize timer variables for user switch input
  debounce_start_time = millis();
  pulse_start_time = millis();
}

void loop() {

  // Get zero mean joystick values
  joystick_x_channel = analogRead(JOYSTICK_X_CHANNEL_PIN) - 512;
  joystick_y_channel = analogRead(JOYSTICK_Y_CHANNEL_PIN) - 512;

  // Map joystick values to motor values so that it starts at min power to move motor.
  // Set offset sign based on sign of input value
  x_channel_sign = (joystick_x_channel > 0) ? 1 : -1;
  x_motor_speed =  j2m_scale_x * joystick_x_channel + j2m_offset_x * x_channel_sign;
  x_motor_speed = (abs(joystick_x_channel) > JOYSTICK_THRESHOLD) ? x_motor_speed : 0;

  y_channel_sign = (joystick_y_channel > 0) ? 1 : -1;
  y_motor_speed = j2m_scale_y * joystick_y_channel + j2m_offset_y * y_channel_sign;
  y_motor_speed = (abs(joystick_y_channel) > JOYSTICK_THRESHOLD) ? y_motor_speed : 0;

  // Check limits for each axis and prevent motor from stalling against hard stop
  if (!digitalRead(X_RIGHT_LIM_PIN)) {
    ClipValue(&x_motor_speed, -255, 0);
  }
  if (!digitalRead(X_LEFT_LIM_PIN)) {
    ClipValue(&x_motor_speed, 0, 255);
  }
  if (!digitalRead(Y_UPPER_LIM_PIN)) {
    ClipValue(&y_motor_speed, -255, 0);
  }
  if (!digitalRead(Y_LOWER_LIM_PIN)) {
    ClipValue(&y_motor_speed, 0, 255);
  }

  /*
  #ifdef SERIAL_DEBUG
  Serial.print(x_motor_speed);
  Serial.print("\t");
  Serial.println(y_motor_speed);
  #endif
  */
  
  // Set Motors with filtered input commands
  SetActuator(x_motor_speed, &X_MOTOR_PINS);
  SetActuator(y_motor_speed, &Y_MOTOR_PINS);

  // Electromagnet User Control
  if (!digitalRead(CLICKPEN_BTN_PIN) && first_press){ // First button press
    if(!debouncing) {
      
      debounce_start_time = millis();
      debouncing = true;
      
      Serial.println("DEBOUNCE");
      
    }
    else if(debouncing && (millis() > (debounce_start_time + DEBOUNCE_TIME)) && !pulse){ // After switch signal stabilizes
      pulse = true;
      first_press = false;
      pulse_start_time = millis();
      
      SetActuator(255, &ELECTROMAGNET_PINS);

      Serial.println("PULSE");
    }
  }

  if(pulse && (millis() > (pulse_start_time + EM_PULSE_DURATION))){ // After pulse duration ends
      debouncing = false;
      pulse = false;

      SetActuator(0, &ELECTROMAGNET_PINS);
  }

  if (digitalRead(CLICKPEN_BTN_PIN) && !first_press){ // After button is released
    first_press = true;
  }
}
