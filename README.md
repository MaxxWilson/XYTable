# ReNeu Drawing Table
This repository contains the code for an XY Drawing Table, designed by Maxx Wilson, Melissa Cruz, Hannah Stanton, and Nathan Moore as part of ME 266K Senior Design in 2021 for the ReNeu Robotics Lab.

The Drawing Table consists of two orthogonal motorized planar axes with a 25kg electromagnet and two permanent magnets at its end effector. The structure is contained within a wooden box with a whiteboard material as its top face. A cart with rollers can be placed on this face such that it interacts with the magnets on the end-effector to actuate a marker both in the XY plane, and between an engaged and disengaged state through the use of a "clickpen" mechanism. The device is operated by a two-axis joystick to move the marker cart in the X and Y Axes, and a push button to actuate the electromagnet and clickpen.

The code runs on an Arduino Uno.

## Interfaces
Inputs include:
- 4x limit switch digital inputs to define workspace bounds
- Push button switch for user to actuate clickpen
- Dual-channel joystick, where each channel is a 10k Ohm potentiometer

All switches are negative logic, where a press sends a digital LOW to the controller. the Arduino 10-bit ADC results in joystick values ranging from 0 to 1023.

Outputs include:
- 2x 12V 200 RPM DC Brushed motors
- 1x 25kg Electromagnet

Each output actuator interfaces with an L298N H-Bridge to control magnitude and direction. The Arduino has three digital outputs to each actuator controller, where two use digital HIGH or LOW to control direction, and the third uses PWM to control effective power to the motor, and thus its speed.

## Joystick Motor Relation
A deadband threshold is applied to zero joystick magnitudes beneath a certain threshold. Above this threshold, a linear equation is defined to produce maximally responsive and intuitive motor action from joystick inputs. These two actions are implemented using a piecewise function to map the joystick input to the motor outputs.

<p align="center"> <img width="750" height="750" src="https://github.com/MaxxWilson/XYTable/blob/main/4_15%20Motor%20Joystick%20Mapping.PNG"> </p>

First, the maximum motor and joystick values are identified for a single channel and direction (sign). This results in motor values between 0 and 255, and joystick values from 0 to 512. Then, a joystick threshold is experimentally determined by evaluating the maximum magnitudes that the joystick may come to rest at upon release. Finally, the minimum duty cycle (power) to move the motor is determined by incrementing the value until the motor moves on each axis.

Define:

<p align="center"> SCALE = (MOTOR_MAX_POWER - MOTOR_MIN_POWER)/(JOYSTICK_MAX_VALUE - JOYSTICK_THRESHOLD) <br> <br> OFFSET = MOTOR_MIN_POWER - JOYSTICK_THRESHOLD * SCALE; <br> <br> For INPUT < THRESHOLD, OUTPUT = 0; <br> <br> For THRESHOLD <= INPUT <= JOYSTICK_MAX_VALUE, OUTPUT = SCALE * INPUT + OFFSET;<div align="left">
