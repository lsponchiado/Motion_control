# Motion_control

## Table of contents
* [Introduction](#Introduction)
* [Scope of functionalities](#Scope-of-functionalities)
* [Classes description](#Classes-description)

## Introduction

The library is composed of 3 classes (Encoder, Axis and Controller).

Using these classes you can include encoders and motor control in your project with almost no efford. Also, by using the Controller class you can include P, PI, PD or PID controllers in your project with great convenience.

Below is a brief description of the functionalities of each class:

### Class Encoder

* Read values from up to 10 encoders simultaneously
* It is compatible with 1 or 2 channels incremental encoders
* For 2-channels encoders you can also increase the resolution for up to 4 times
* It is necessary to use interrupts

### Class Axis

* Control motors using PWM signal
* It is compatible with simple PWM control signal and PWM + H-Bridge
* You can include phisical safety switches to cut off the movement (up to 10 axes, interrupts needed)

### Class Controller
