// Motor.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot. Lab 13 solution
// Daniel Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

// *******Lab 13 solution*******

// ------------Motor_Init------------
// Initialize GPIO pins for output, which will be
// used to control the direction of the motors and
// to enable or disable the drivers.
// The motors are initially stopped, the drivers
// are initially powered down, and the PWM speed
// control is uninitialized.
// Input: none
// Output: none


/*
 * Motor Pins
 * Right P5.5, P2.6, P3.6
 * Left P5.4, P2.7, P3.7
 *
 * Motor Direction bits
 * Left Forward - P5->OUT &= ~0x10;
 * Right Forward - P5->OUT &= ~0x20;
 * Left Reverse- P5->OUT |= 0x10;
 * Right Reverse - P5->OUT |= 0x20;
 * Both forward - P5->OUT &= ~0x30;
 * both reverse - P5->OUT |= 0x30;
 *
 * Left on - P2->OUT |= 0x80;
 * Right on - P2->OUT |= 0x40;
 * Left off - P2->OUT &= ~0x80;
 * Right off - P2->OUT &= ~0x40;
 * Both on - P2->OUT |= 0xC0;
 * Both off - P2->OUT &= ~0xC0;
 *
 * Left Awake - P3->OUT |= 0x80;
 * Right Awake - P3->OUT |= 0x40;
 * Left Sleep - P3->OUT &= ~0x80;
 * Right Sleep - P3->OUT &= ~0x40;
 * Both Awake - P3->OUT &= ~0xC0;
 * Both Sleep - P3->OUT |= 0xC0;
 */

//Initialise the Pins for the motor controller
void InitPort2(void){
    P2->SEL0 |= 0xC0;
    P2->SEL1 &= ~0xC0;    // 1) configure P2.6-P2.7 as GPIO
    P2->DIR |= 0xC0;      // 2) make P2.6-P2.7 out
    P2->DS |= 0xC0;       // 3) activate increased drive strength
    P2->OUT &= ~0xC0;     //    Turn off the output
}
void InitPort3(void){
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;    // 1) configure P3.6-P3.7 as GPIO
    P3->DIR |= 0xC0;      // 2) make P3.6-P3.7 out
    P3->DS |= 0xC0;       // 3) activate increased drive strength
    P3->OUT &= ~0xC0;     //    Turn off the output
}
void InitPort5(void){
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;    // 1) configure P5.4-P5.5 as GPIO
    P5->DIR |= 0x30;      // 2) make P5.4-P5.5 out
    P5->DS |= 0x30;       // 3) activate increased drive strength
    P5->OUT &= ~0x30;     //    Turn off the output
}

void Motor_Init(void){
  // write this as part of Lab 13
    //initialise the outputs
    InitPort2();
    InitPort3();
    InitPort5();
    PWM_Init34(15000, 500, 500);
    P3->OUT &= ~0xC0;
}

// ------------Motor_Stop------------
// Stop the motors, power down the drivers, and
// set the PWM speed control to 0% duty cycle.
// Input: none
// Output: none
void Motor_Stop(void){
  // write this as part of Lab 13
    PWM_Duty3(0);   //set to 0 to turn off motor
    PWM_Duty4(0);   //set to 0 to turn off motor
//    P2->SEL0 &= ~0xC0;
//    P2->SEL1 &= ~0xC0;    // 1) configure P2.6-P2.7 as GPIO
//    P2->OUT &= ~0xC0;   // Disable motor
//    P3->OUT |= 0xC0;
//    P5->OUT &= ~0xC0;   // Direction
}

// ------------Motor_Forward------------
// Drive the robot forward by running left and
// right wheels forward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Forward(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    P5->OUT &= ~0x30;
    P3->OUT |= 0xC0;    // Awake state
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);
  
}

// ------------Motor_Right------------
// Turn the robot to the right by running the
// left wheel forward and the right wheel
// backward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Right(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    P5->OUT &= ~0x20;
    P3->OUT |= 0xC0;    // Awake state
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);

}

// ------------Motor_Left------------
// Turn the robot to the left by running the
// left wheel backward and the right wheel
// forward with the given duty cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Left(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    P5->OUT &= ~0x10;
    P3->OUT |= 0xC0;    // Awake state
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);

}

// ------------Motor_Backward------------
// Drive the robot backward by running left and
// right wheels backward with the given duty
// cycles.
// Input: leftDuty  duty cycle of left wheel (0 to 14,998)
//        rightDuty duty cycle of right wheel (0 to 14,998)
// Output: none
// Assumes: Motor_Init() has been called
void Motor_Backward(uint16_t leftDuty, uint16_t rightDuty){ 
  // write this as part of Lab 13
    P5->OUT |= 0x30;
    P3->OUT |= 0xC0;    // Awake state
    PWM_Duty4(leftDuty);
    PWM_Duty3(rightDuty);

}
