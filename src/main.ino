/*
 * Copyright (C) 2024 Scott CJX
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// ignore if using Arduino IDE
#if PLATFORMIO
#include <Arduino.h>
#endif /* PLATFORMIO */

// download/ install servo library
#include <Servo.h>

/**
 * @frames:
 * Rotation Base: RotZ
 * - servo: 0
 * Arm Axis 1: Ax1
 * - servo: 1
 * Arm Axis 2: Ax2
 * - servo: 2
 * Gripper Effector: Eff1
 * - servo: 3
 */

// controls def
#define PINDEF_JOYX             A0
#define PINDEF_JOYY             A1
#define PINDEF_JOYBTN           A2              // button switches mode on which serv to control 

/**
 * @config:
 * RotZ will always be controlled by JOYX (left right motion)
 * 
 * Rest will be controlled depending on @control_state:
 * @control_state: 0
 * @control: Eff1
 * @control_state: 1
 * @control: Ax1
 * @control_state: 2
 * @control: Ax2
 */

// output def
#define PINDEF_SERVO_AX1        3
#define PINDEF_SERVO_AX2        5
#define PINDEF_SERVO_ROT        6
#define PINDEF_SERVO_EFF        9

#define ADC_MAX                 1023            // depends on resolution of ADC, uno is 10bit, thus max = (2^10)-1 = 1024
#define DEF_JOY_MID             ADC_MAX/2       // assuming joy returns to center, its rest pos is ADC_MAX/2, not in use
#define JOY_TO_DEGCHANGE_SENS   10              // this determines the sens of joy to change servo pos move per update, experiment with this
#define BTN_PRESSED             1               // change this depending on setup of btn
#define NUM_SERVS               4
#define START_DEG               90              // init degree of stepper

const bool isUnlocked = 1;                      // this is for next version, that allows lock and unlock from rfid, not implemented yet
int control_state = 1;                          // @ref config
int state_joy_x, state_joy_y;
bool state_btn = 0, state_btn_last = 0;
int req_deg_servs[NUM_SERVS-1];                 // init new array to store requested degs of all motors
Servo myservos[NUM_SERVS-1];

void change_state();
void read_inputs();
void perform_action();

void setup() {
    // change mapping here too
    myservos[0].attach(PINDEF_SERVO_ROT);
    myservos[1].attach(PINDEF_SERVO_AX1);
    myservos[2].attach(PINDEF_SERVO_AX2);
    myservos[3].attach(PINDEF_SERVO_EFF);

    pinMode(PINDEF_JOYX, INPUT);
    pinMode(PINDEF_JOYY, INPUT);
    pinMode(PINDEF_JOYBTN, INPUT_PULLUP);           // connect btn: A: PINDEF_JOYBTN, G: Gnd

    for (int i = 0; i < NUM_SERVS-1; i++)
    {
        // init all servs and their requests to starting degree
        req_deg_servs[i] = START_DEG;
    }

    // update the servos
    for (int i = 0; i < NUM_SERVS-1; i++)
    {
        myservos[i].write(req_deg_servs[i]);
    }
}

void loop() {
    read_inputs();
    perform_action();

    /**
     * this delay is important too, experiment with it.
     * this delay affects:
     * - amount of updates the joystick will +/- persec. making it more sensitive. thus decreasing
     *  delay requires lowering @param JOY_TO_DEGCHANGE_SENS
     * 
     */
    delay(250);
}


void change_state()
{
    if (control_state == NUM_SERVS)
    {
        control_state = 1;                          // this is as control_state = 0 is rot, it will not be controlled here
    }
    else
    {
        control_state++;
    }
}

void read_inputs()
{
    state_joy_x = analogRead(PINDEF_JOYX);
    state_joy_y = analogRead(PINDEF_JOYY);
    state_btn_last = state_btn;
    state_btn = digitalRead(PINDEF_JOYBTN);

    // if the last state is NOT pressed, and now state IS pressed, then its a new press (hopefully -> cos debounce)
    if (state_btn_last != BTN_PRESSED && state_btn == BTN_PRESSED)
    {
        change_state();
    }
}

void perform_action()
{
    // static means it only initializes once, instead of reinniting each cycle.
    static int centered_x, centered_y;          // -90 -> +90
    static int req_x, req_y;                    // 0 -> JOY_TO_DEGCHANGE_SENS [use centered_xx for dir]

    if (!isUnlocked) return;

    // conv joystick (0->180) to (-90->+90)
    centered_x = state_joy_x - DEF_JOY_MID;
    centered_y = state_joy_y - DEF_JOY_MID;

    // conv joystick (0->180) --> (0->JOY_TO_DEGCHANGE_SENS), this is how much to move the req of serv deg
    req_x = map(abs(centered_x), 0, DEF_JOY_MID, 0, JOY_TO_DEGCHANGE_SENS);
    req_y = map(abs(centered_y), 0, DEF_JOY_MID, 0, JOY_TO_DEGCHANGE_SENS);

    // consider the direction of joystick off from center
    if (centered_x < 0) req_x *= -1;
    if (centered_y < 0) req_y *= -1;

    req_deg_servs[0] += req_x;                  // use x_req to offset the requested for rotZ

    req_deg_servs[control_state] += req_y;      // use y_req to offset the requested for whichever the controlstate @

    // Update selected servo based on control_state
    for (int i = 0; i < NUM_SERVS-1; i++)
    {
        myservos[i].write(req_deg_servs[i]);
    }
}
