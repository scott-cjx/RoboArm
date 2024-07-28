# RoboArm

This repository contains resources for the RoboArm project for workshops @NTU-Mecatrons hold.

## Getting Started

### Mechanical
[Insert CAD image here]

### Electrical 
[Insert Schematic for connection here]

### Software
For those using the Arduino IDE, download and open `main.ino`.
\* note that relevant libraries might have to be installed for project to work

For those using PlatformIO, open repository as a project. it should work out of the box

#### Phases

There are multiple levels to the project:
1. Reading (analog) inputs from Joystick
2. Sending (PWM) outputs to servo via Servo library
3. Combining 1 & 2 to control arm from joystick 
    > example [here](/src/main.ino)
4. Reading NFC/RFID tag
5. Combining 3 & 4 to control arm from joystick with enable/disabling via NFC/RFID

