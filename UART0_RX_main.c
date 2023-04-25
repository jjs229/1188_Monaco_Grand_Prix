// Lab11_UARTmain.c
// Runs on MSP432
// Test the functions in UART0.c by printing various
// Busy-wait device driver for the UART UCA0.
// Daniel Valvano
// May 24, 2019

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

#include "msp.h"
#include "..\inc\Clock.h"
#include "..\inc\UART0.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Bump.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\Motor.h"
#include "..\inc\PWM.h"
#include <string.h>
#include <stdio.h>
#include "..\inc\CortexM.h"


void initMadMax(void){
    // Initialize Port 3.6 3.7    0b 1100 0000
        P3->SEL0 &= ~0xC0;
        P3->SEL1 &= ~0xC0;   // configure 5.3 as GPIO
        P3->DIR |= 0xC0;     // make P5.3 out
        P3->OUT &= ~0xC0;
}


void mapBits(char* x){
    int N;
    if(x[0] != 0x0A){
        char ch = x[0];
        printf("ch: %c\n", ch);
        N = (int)(ch)+0;
    }


    switch (N) {
    case 0x64: // forward
       Motor_Forward(1200, 1200);
       LaunchPad_Output(GREEN);
       break;

    case 0x73: // stop
       Motor_Stop();
       LaunchPad_Output(RED);
       break;
    default:
        LaunchPad_Output(RED+GREEN+BLUE);
       // all bits are high (no LEDs on)
       break;
    }
}



volatile uint32_t Time;
uint8_t Bumper;
int Semaphore = 0;
void SysTick_Handler(void){

    //    uint8_t newMsg = 0;
        uint16_t max = 9;
        char command[5] = "00000";
        UART0_OutString("hi");
        UART0_OutString("vvvvv");

        UART0_InString(command, max);
        char* data = strstr(command, "TX="); // Find the position of "TX="
        if (data != NULL) {
            data += strlen(" TX="); // Move the pointer past "TX="
            printf("Data: %s\n", data);
        }
    //    get BT
        mapBits(command);

    Time = Time + 1; // every 1ms
    if(Time%10==1){
        Bumper = Bump_Read();
        Semaphore =1;
      }

  if(Time%10==2){
    if (Semaphore == 1){
        if (Bumper != 0xFF){
            Motor_Stop();
            Clock_Delay1ms(1000);
            Motor_Forward(7500,7500);
        }
        Semaphore = 0;
    }
  }
}


void main(void){
    Time = 0;
//    uint32_t data=0;
    Clock_Init48MHz();     // set system clock to 48 MHz
//    LaunchPad_Init();      // for debugging // P1.0 is red LED on LaunchPad
    initMadMax();
    Bump_Init();      // bump switches
    Motor_Init();
    UART0_Init();
    SysTick_Init(600, 2);  // set up SysTick for 8 Hz interrupts
    EnableInterrupts(); //cortexM lib// set system clock to 48 MHz
    //inlclude all inits







  while(1){
//      read BT
    WaitForInterrupt();


    Clock_Delay1ms(1000);





    //put nav program here
  }
}



