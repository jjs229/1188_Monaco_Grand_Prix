// MotorSimple.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot.
// Student starter code for Lab 12, uses Systick software delay to create PWM
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

#include <stdint.h>
#include "msp.h"
#include "../inc/SysTick.h"
#include "../inc/Bump.h"
//**************RSLK1.1***************************
// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)
// *******Lab 12 solution*******

//Initialise the Pins for the motor controller
void InitPort2(void){
    P2->SEL0 &= ~0xC0;
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

//timer for delays
void SysTick_Wait1us(uint32_t delay){
    // write this code
    uint32_t i;
    for(i=0; i<delay; i++){
      SysTick_Wait(48);  // wait 1us (assumes 48 MHz clock)
    }
}


void Motor_InitSimple(void){

// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs

  // write this as part of Lab 12
    //P5.4, P5.5 - Direction Left, Direction Right (initialise to 0)
    //P2.7, P2.6 - Enable Left, Enable Right (initialise to 0)
    //P3.7, P3.6 - Sleep Left, Sleep Right (initialise to 1 or NC)

    //initialise the outputs
    InitPort2();
    InitPort3();
    InitPort5();
    P3->OUT &= ~0xC0;
    SysTick_Init();


}

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

void Motor_StopSimple(void){
// Stops both motors, puts driver to sleep
// Returns right away
    P2->OUT &= ~0xC0;   // Disable motor
    P3->OUT &= ~0xC0;   // Sleep State
    P5->OUT &= ~0xC0;   // Direction
}
void Motor_ForwardSimple(uint16_t duty, uint32_t time){
// Drives both motors forward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Stop the motors and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    P3->OUT |= 0xC0;    // Awake state
    P5->OUT &= ~0x30;   // Both Forward Direction


    // For loop works since the pwm period is 10ms and we are
    // taking time in 10ms increments
    uint32_t i;
    for(i = 0; i<time; i++)
    {

        //We spin the motor here for pwm
        P2->OUT |= 0xC0;   // Enable motor
        SysTick_Wait1us(duty);
        //we turn off the motor here for pwm
        P2->OUT &= ~0xC0;   // disable motor
        SysTick_Wait1us(10000-duty);

    }

    P2->OUT &= ~0xC0;   // Disable motors once done
    P3->OUT &= ~0xC0;   // Put to Sleep State


}
void Motor_BackwardSimple(uint16_t duty, uint32_t time){
// Drives both motors backward at duty (100 to 9900)
// Runs for time duration (units=10ms), and then stops
// Runs even if any bumper switch is active
// Returns after time*10ms


    P3->OUT |= 0xC0;    // Awake state
    P5->OUT |= 0x30;   // Both Reverse Direction


    // For loop works since the pwm period is 10ms and we are
    // taking time in 10ms increments
    uint32_t i;
    for(i = 0; i<time; i++)
    {

        //We spin the motor here for pwm
        P2->OUT |= 0xC0;   // Enable motor
        SysTick_Wait1us(duty);
        //we turn off the motor here for pwm
        P2->OUT &= ~0xC0;   // disable motor
        SysTick_Wait1us(10000-duty);

    }

    P2->OUT &= ~0xC0;   // Disable motors once done
    P3->OUT &= ~0xC0;   // Put to Sleep State

}
void Motor_LeftSimple(uint16_t duty, uint32_t time){
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    P3->OUT &= ~0xC0;    // Both Sleep
    P3->OUT |= 0x80;    // Left Awake state
    P5->OUT &= ~0x10;   // Left forward


    // For loop works since the pwm period is 10ms and we are
    // taking time in 10ms increments
    uint32_t i;
    for(i = 0; i<time; i++)
    {

        //We spin the motor here for pwm
        P2->OUT |= 0xC0;   // Enable motor
        SysTick_Wait1us(duty);
        //we turn off the motor here for pwm
        P2->OUT &= ~0xC0;   // disable motor
        SysTick_Wait1us(10000-duty);

    }

    P2->OUT &= ~0xC0;   // Disable motors once done
    P3->OUT &= ~0xC0;   // Put to Sleep State

}
void Motor_RightSimple(uint16_t duty, uint32_t time){
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)
// Runs for time duration (units=10ms), and then stops
// Stop the motor and return if any bumper switch is active
// Returns after time*10ms or if a bumper switch is hit

    P3->OUT &= ~0xC0;    // Both Sleep
    P3->OUT |= 0x40;    // Right Awake state
    P5->OUT &= ~0x20;   // Right forward


    // For loop works since the pwm period is 10ms and we are
    // taking time in 10ms increments
    uint32_t i;
    for(i = 0; i<time; i++)
    {

        //We spin the motor here for pwm
        P2->OUT |= 0xC0;   // Enable motor
        SysTick_Wait1us(duty);
        //we turn off the motor here for pwm
        P2->OUT &= ~0xC0;   // disable motor
        SysTick_Wait1us(10000-duty);

    }

    P2->OUT &= ~0xC0;   // Disable motors once done
    P3->OUT &= ~0xC0;   // Put to Sleep State

}

