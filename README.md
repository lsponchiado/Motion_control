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

## Scope of functionalities

## Classes description

### Encoder

#### Creating a new object

By just defining the channels pins you can read position and speed using pulses and pulses / second.
If you also define a resolution and a pitch, you have the option to also read position using mm, degree and rotations and speed using mm/s, degree/s, Hz and RPM.

One channel configuration (can't detect direction):
```
int channel1 = 2;       // phisical inputs
int resolution = 600;   // pulses per revolution
double pitch = 40.0;    // 40 mm per revolution

Encoder encoder(channel1);                      
//Encoder encoder(channel1, resolution, pitch);
```

Two channel configuration (can detect direction):
```
int channel1 = 2, channel2 = 3;   // phisical inputs
int resolution = 600;             // pulses per revolution
double pitch = 40.0;              // 40 mm per revolution
int mode = 1;                     // x1 (mode 1), x2 (mode 2), x4 (mode 3)

Encoder encoder(channel1, channel2);
//Encoder encoder(channel1, channel2, mode);
//Encoder encoder(channel1, channel2, resolution, pitch);
//Encoder encoder(channel1, channel2, resolution, pitch, mode);
```

```
// Reset the current value of the encoder
void setReference();

// Set the current value of the encoder as the given value
void setReference(double value);

// Set a new resolution
void setPulsesPerRev(int32_t value);

// Set a new pitch value
void setPitchPerRev(double value);

// Set a given value as trigger 
void setSpeedTrigger(int32_t value);
```

### Axis

### Controller
