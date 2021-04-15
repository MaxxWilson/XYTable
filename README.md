# ReNeu Drawing Table
This repository contains the code for an XY Drawing Table, designed by Maxx Wilson, Melissa Cruz, Hannah Stanton, and Nathan Moore as part of ME 266K Senior Design in 2021 for the ReNeu Robotics Lab.

The Drawing Table consists of two orthogonal motorized planar axes with a 25kg electromagnet and two permanent magnets at its end effector. The structure is contained within a wooden box with a whiteboard material as its top face. A cart with rollers can be placed on this face such that it interacts with the magnets on the end-effector to actuate a marker both in the XY plane, and between an engaged and disengaged state through the use of a "clickpen" mechanism. The device is operated by a two-axis joystick to move the marker cart in the X and Y Axes, and a push button to actuate the electromagnet and clickpen.

The code runs on an Arduino Uno.

## Interface Overview
Inputs include:
- 4x limit switch digital inputs to define workspace bounds
- Push button switch for user to actuate clickpen
- Dual-channel joystick, where each channel is a 10k Ohm potentiometer

All switches are negative logic, where a press sends a digital LOW to the controller. The Arduino 10-bit ADC results in joystick values ranging from 0 to 1023.

Outputs include:
- 2x 12V 200 RPM DC Brushed motors
- 1x 25kg Electromagnet

Each output actuator interfaces with an L298N H-Bridge to control magnitude and direction. The Arduino has three digital outputs to each actuator controller, where two use digital HIGH or LOW to control direction, and the third uses PWM to control effective power to the motor, and thus its speed.

All pins are configured at the top of the code:

```c
// Actuator Pins
#define X_MOTOR_FWD_PIN 0
#define X_MOTOR_BWD_PIN 1
#define X_MOTOR_ENA_PIN 3

#define Y_MOTOR_FWD_PIN 2
#define Y_MOTOR_BWD_PIN 4
#define Y_MOTOR_ENA_PIN 5

#define EM_FWD_PIN 8
#define EM_BWD_PIN 7
#define EM_ENA_PIN 6

// Limit Switch Pins
#define X_RIGHT_LIM_PIN 9
#define X_LEFT_LIM_PIN 10

#define Y_LOWER_LIM_PIN 11
#define Y_UPPER_LIM_PIN 12

// User Input Pins
#define CLICKPEN_BTN_PIN 13

#define JOYSTICK_Y_CHANNEL_PIN A4
#define JOYSTICK_X_CHANNEL_PIN A5
```

## Joystick Motor Relation
A deadband threshold is applied to zero joystick magnitudes beneath a certain threshold. Above this threshold, a linear equation is defined to produce maximally responsive and intuitive motor action from joystick inputs. These two actions are implemented using a piecewise function to map the joystick input to the motor outputs.

<p align="center"> <img width="750" height="750" src="https://github.com/MaxxWilson/XYTable/blob/main/4_15%20Motor%20Joystick%20Mapping.PNG"> </p>

First, the maximum motor and joystick values are identified for a single channel and direction (sign). This results in motor values between 0 and 255, and joystick values from 0 to 512. Then, a joystick threshold is experimentally determined by evaluating the maximum magnitudes that the joystick may come to rest at upon release. Finally, the minimum duty cycle (power) to move the motor is determined by incrementing the value until the motor moves on each axis. In code, the scale is valid for both positive and negative ranges, but the offset must match the sign of the input variable to properly mirror the piecewise function about the diagonal.

#### Definitions
```c
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
```
#### Usage
```c
// Map joystick values to motor values so that it starts at min power to move motor.
// Set offset sign based on sign of input value
x_channel_sign = (joystick_x_channel > 0) ? 1 : -1;
x_motor_speed =  j2m_scale_x * joystick_x_channel + j2m_offset_x * x_channel_sign;
x_motor_speed = (abs(joystick_x_channel) > JOYSTICK_THRESHOLD) ? x_motor_speed : 0;

y_channel_sign = (joystick_y_channel > 0) ? 1 : -1;
y_motor_speed = j2m_scale_y * joystick_y_channel + j2m_offset_y * y_channel_sign;
y_motor_speed = (abs(joystick_y_channel) > JOYSTICK_THRESHOLD) ? y_motor_speed : 0;
```

## Motor Power Limiting
Each axis has a limit switch at its two bounds so that the software does not allow a user to continue applying power against the boundary of the workspace. All switches and button inputs are configured as negative logic, meaning a button press will drive the input voltage to zero. These must be configured with a pullup resistor to work properly. This can be done in hardware, but is more conveniently implemented in software by using existing Arduino hardware.

```c
  // Initialize each switch with input pullup for negative logic, meaning when you press the button, the voltage goes low (GND)
  pinMode(Y_UPPER_LIM_PIN, INPUT_PULLUP);
  pinMode(Y_LOWER_LIM_PIN, INPUT_PULLUP);
  pinMode(X_LEFT_LIM_PIN, INPUT_PULLUP);
  pinMode(X_RIGHT_LIM_PIN, INPUT_PULLUP);
  pinMode(CLICKPEN_BTN_PIN, INPUT_PULLUP);
```
When a limit switch is pressed, motor power is clipped such that the maximum power in the direction of the switch is 0, without affecting actuation in the other direction. 

```c
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
```

```c
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
```

## Actuator Abstraction
All three actuators share a similar interface through the L298N Motor Driver. Thus, a struct type was defined containing the two direction pins and the enable pin. This struct is then used for each actuator definition.

```c
typedef struct ActuatorOut {
  uint8_t fwd_pin;
  uint8_t bwd_pin;
  uint8_t ena_pin;
} AcuatorOut;

const ActuatorOut X_MOTOR_PINS = {X_MOTOR_FWD_PIN, X_MOTOR_BWD_PIN, X_MOTOR_ENA_PIN};
const ActuatorOut Y_MOTOR_PINS = {Y_MOTOR_FWD_PIN, Y_MOTOR_BWD_PIN, Y_MOTOR_ENA_PIN};
const ActuatorOut ELECTROMAGNET_PINS = {EM_FWD_PIN, EM_BWD_PIN, EM_ENA_PIN};
```

The address of each actuators struct can then be passed to general actuation functions to maintain code modularity and minimize bugs.
### Actuator Init
A single actuator initialization is defined to configure the three pins as outputs.

#### Definition

```c
//------------ActuatorInit-----------
// Function to initialize the direction and enable pins of an actuator.
// Inputs:  actuator      struct pointer representing actuator configuration
// Outputs: none
void ActuatorInit(const ActuatorOut *actuator) {
  pinMode((*actuator).fwd_pin, OUTPUT);
  pinMode((*actuator).bwd_pin, OUTPUT);
  pinMode((*actuator).ena_pin, OUTPUT);
}
```

#### Usage

```c
  // Initialize each actuator with two directional pins and an enable pin. The enable pin determines the
  // speed of the motor, the directional pins determine which way it rotates. Direction pins should NEVER both be high. 
  ActuatorInit(&Y_MOTOR_PINS);
  ActuatorInit(&ELECTROMAGNET_PINS);

  // The X motor uses the same ports as the Serial Transmit and Recieve, so to debug it must be disabled
  #ifndef DEBUG
  ActuatorInit(&X_MOTOR_PINS);
  #endif
```

### Set Actuator
A single function is defined to set actuator direction and magnitude. The magnitude will always be positive and thus unsigned, the direction pins will determine the polarity of the voltage sent to the motor. This function abstracts the seperate direction and enable pins to allow a user to simply set a signed speed and have direction inferred, where positive is "forward" and negative is "backward", The function also clips input speeds larger than 8-bits to avoid overflow.

#### Definition

```c
//------------SetActuator-----------
// Function to set the direction and magnitude of an actuator. Negative magnitude implies
// Counter-Clockwise rotation for a motor and opposing polarity for an electromagnet
// Inputs:  mag           magnitude of actuator effort between -255 and 255
//          actuator      struct pointer representing actuator configuration
// Outputs: none
void SetActuator(int16_t mag, const ActuatorOut *actuator) {
  
  // Clip to 255 and avoid overflow in 8-bit value
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
```
#### Set Actuator Usage

```c
    // Set Motors with filtered input commands
  SetActuator(y_motor_speed, &Y_MOTOR_PINS);

  #ifndef DEBUG
  SetActuator(x_motor_speed, &X_MOTOR_PINS);
  #endif
```

```c
  SetActuator(255, &ELECTROMAGNET_PINS);
```
#### Motor Enable
The electromagnet pulls a substantial amount of power when it is pulsed. As the pulse is typically short, we should disable the two motors for the duration of the pulse to lwoer peak amperage. enable_motors is defined as a boolean value that can be toggled by the electromagnet code to zero the motors during the pulse. This block occurs directly prior to the SetActuator Calls.

```c
  if(!enable_motors){
    x_motor_speed = 0;
    y_motor_speed = 0;
  }
```

## Electromagnet Actuation and Debouncing
Electromagnet control is implemented in four steps, including the button input debounce, the pulse start, the pulse end, and the button input reset.

A debouncer is implemented on the push button to filter user input into a usable digital signal. On the first press signal of the button in any given cycle, the debouncing mode variable is set to true and the time at which the button is pressed is stored. If the debouncing period passes and the button remains pressed, there is confidence that the signal is a legitimate user input. The first_press variable is set to false so that the electromagnet will only be actuated once for everytime the button is pressed (the button cannot be held). The electromagnet is then set high to begin the pulse, and the pulse start time is stored.

```c
  // Electromagnet User Control
  if (!digitalRead(CLICKPEN_BTN_PIN) && first_press){ // First button press
    if(!debouncing) {
      
      debouncing = true;
      debounce_start_time = millis();
      
      #ifdef DEBUG
      Serial.println("DEBOUNCE");
      #endif
    }
    else if(debouncing && (millis() > (debounce_start_time + DEBOUNCE_TIME)) && !pulse){ // After switch signal stabilizes
      pulse = true;
      first_press = false;
      debouncing = false;
      
      pulse_start_time = millis();

      enable_motors = false;
      SetActuator(255, &ELECTROMAGNET_PINS);

      #ifdef DEBUG
      Serial.println("PULSE");
      #endif
    }
  }
```

Similar to the debouncer, the pulse will wait a certain duration before ending the pulse.

```c
  if(pulse && (millis() > (pulse_start_time + EM_PULSE_DURATION))){ // After pulse duration ends
      pulse = false;

      SetActuator(0, &ELECTROMAGNET_PINS);
      enable_motors = true;
  }
```

Finally, when the button is released, the first_press variable is reset so that the user can once again activate the electromagnet.

```c
  if (digitalRead(CLICKPEN_BTN_PIN) && !first_press){ // After button is released
    first_press = true;
  }
```

## Debugging
### Serial
This project uses every digital pin on the Arduino Uno, including pins 0 (Rx) and 1 (Tx), used for Serial communication. This means, there is no way to recieve usable information through the Serial Monitor/Plotter from the Arduino as long as pins 0 and 1 are in use. Thus, the X Motor ActuationInit and SetActuator calls are wrapped in a debug ifdef. To activate debugging and disable these calls automatically, comment out the DEBUG definition at the top of the file.

```c
//#define DEBUG
```

```c
  // The X motor uses the same ports as the Serial Transmit and Recieve, so to debug it must be disabled
  #ifndef DEBUG
  ActuatorInit(&X_MOTOR_PINS);
  #endif
```

```c
  #ifndef DEBUG
  SetActuator(x_motor_speed, &X_MOTOR_PINS);
  #endif
```
For serial debugging, refer to the Arduino reference online.

### Actuator Directions
If you find that any actuators are going the wrong directions, it is easier to simple flip the two direction pin definitions at the top of the code rather than to rewire. This is fine. The only thing to be careful of is that <B>the two direction pins can never be set high at the same time or it may damage the H-Bridge and Arduino. </B> If constructing DIY, it is highly recommended to test each motor individually first to ensure correct polarities.
