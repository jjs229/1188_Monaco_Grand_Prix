//*****************************************************************************
// test main for Robot with OPT3101 time of flight sensor
// MSP432 with RSLK Max and OPT3101
// Daniel and Jonathan Valvano
// July 6, 2020
//****************************************************************************
/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2020
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2020, Jonathan Valvano, All rights reserved.

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
// see opt3101.h for OPT3101 hardware connections
// see Nokia5110.h LCD hardware connections
// see SSD1306.h for OLED hardware connections
// see UART0.h for UART0 hardware connections

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/I2CB1.h"
#include "../inc/CortexM.h"
#include "../inc/opt3101.h"
#include "../inc/LaunchPad.h"
#include "../inc/UART0.h"
//#include "BumpInt.c"

#include "..\inc\Bump.h"
#include "..\inc\SysTickInts.h"
#include "..\inc\Motor.h"
#include "..\inc\PWM.h"
#include <string.h>
#include <stdio.h>

#define Init UART0_Init
#define Clear UartClear
#define SetCursor UartSetCur
#define OutString UART0_OutString
#define OutChar UART0_OutChar
#define OutUDec UART0_OutUDec


uint32_t Distances[3];
uint32_t FilteredDistances[3];
uint32_t Amplitudes[3];
uint32_t TxChannel;
uint32_t StartTime;
uint32_t TimeToConvert; // in msec
uint32_t i;
uint32_t Error;
int32_t UR, UL;  // PWM duty 0 to 14,998
int32_t SetPoint = 250; // mm //was 250
int32_t LeftDistance,CenterDistance,RightDistance; // mm
uint32_t ON = 0;
char hope;


//Controller
#define PWMNOMINAL 14999
#define SWING 2000
#define DESIRED 250
#define TOOFAR 400 // was 400

// assumes track is 500mm
int32_t Mode=0; // 0 stop, 1 run
int32_t Ki=1;  // integral controller gain
int32_t Kp=4;  // proportional controller gain //was 4
int32_t status = 0;


//-----------------Bluetooth stuff--------------------------------------------------------------
void mapBits(char* x){
    int N;
    if(x[0] != 0x0A){
        char ch = x[0];
        printf("ch: %c\n", ch);
        N = (int)(ch)+0;
    }


    switch (N) {
    case 0x64: // forward
        ON = 1;
       break;

    case 0x73: //stop:
       Motor_Stop();
       ON = 0;
       break;
    default:
       break;
    }
}

//----------------UART--------------------------------------------------------------------------

void UartSetCur(uint8_t newX, uint8_t newY)
{
  if(newX == 6){
    UART0_OutString("\n\rTxChannel= ");
    UART0_OutUDec(newY-1);
    UART0_OutString(" Distance= ");
  }else{
    UART0_OutString("\n\r");
  }
}
void UartClear(void){UART0_OutString("\n\r");};

bool pollDistanceSensor(void)
{
  if(OPT3101_CheckDistanceSensor())
  {
    TxChannel = OPT3101_GetMeasurement(Distances,Amplitudes);
    return true;
  }
  return false;
}

int32_t Left(int32_t left){
return (1247*left)/2048 + 22;
}

int32_t Right(int32_t right){
  return  (right*(59*right + 7305) + 2348974)/32768;
}

void Controller_Right(void){ // runs at 100 Hz
  if(Mode){
    if((RightDistance>DESIRED)){
      SetPoint = (RightDistance)/2;
    }else{
      SetPoint = DESIRED;
    }
    /*if(LeftDistance < RightDistance ){
      Error = LeftDistance-SetPoint;
    }else {
      Error = SetPoint-RightDistance;
    }*/

    Error = SetPoint-RightDistance;
    //UR = UR + Ki*Error;      // adjust right motor
    UR = PWMNOMINAL+Kp*Error; // proportional control
    UR = UR + Ki*Error;      // adjust right motor
    UL = PWMNOMINAL-Kp*Error; // proportional control
    if(UR < (PWMNOMINAL-SWING)){UR = PWMNOMINAL-SWING;} // 3,000 to 7,000
    if(UR > (PWMNOMINAL+SWING)){UR = PWMNOMINAL+SWING;}
    if(UL < (PWMNOMINAL-SWING)){UL = PWMNOMINAL-SWING;} // 3,000 to 7,000
    if(UL > (PWMNOMINAL+SWING)){UL = PWMNOMINAL+SWING;}

    //turns left if the center measurement and right measurement is small enough that we will hit the wall if we don't turn
    if((RightDistance<250) && (CenterDistance <250)){
        UL = 0;
        UR = PWMNOMINAL;
    }

    Motor_Forward(UL,UR);

  }
}

volatile uint32_t Time;
uint8_t Bumper;
int Semaphore = 0;
uint16_t max = 9;
char command[5] = "00000";

void bump(){
    Bumper = BumpInt_Read();
    if (Bumper != 0xED){
        Motor_Stop();
        Clock_Delay1ms(1000);
    }

}
void main(void)
{ // busy-wait implementation
  uint32_t channel = 1;
  Clock_Init48MHz();
  Motor_Init();

  //Odometry_Init(0, 0, 0);
  I2CB1_Init(30); // baud rate = 12MHz/30=400kHz

  SysTick->LOAD = 0x00FFFFFF;           // maximum reload value
  SysTick->CTRL = 0x00000005;

  Clear();
  OPT3101_Init();
  OPT3101_Setup();
  OPT3101_CalibrateInternalCrosstalk();
  OPT3101_StartMeasurementChannel(channel);
  OPT3101_ArmInterrupts(&TxChannel, Distances, Amplitudes);

  BumpInt_Init();

  LPF_Init(100,8);
  LPF_Init2(100,8);
  LPF_Init3(100,8);

  StartTime = SysTick->VAL;
  UART0_Init();
  EnableInterrupts();
  Mode = 1;
  while(1)
  {
 //     TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48000; // msec
      while(ON == 0){
          hope = UART0_InChar();
          if(hope == 'd'){ON = 1;}
      }

      while(ON == 1){
      TimeToConvert = ((StartTime-SysTick->VAL)&0x00FFFFFF)/48000;
      if(TxChannel <= 2)
      {
          if(TxChannel==0){
                  if(Amplitudes[0] > 1000){
                    LeftDistance = FilteredDistances[0] = Left(LPF_Calc(Distances[0]));
                  }else{
                    LeftDistance = FilteredDistances[0] = 500;
                  }

          }else if(TxChannel==1){
                  if(Amplitudes[1] > 1000){
                    CenterDistance = FilteredDistances[1] = LPF_Calc2(Distances[1]);
                  }else{
                    CenterDistance = FilteredDistances[1] = 500;
                  }
          }else {
                  if(Amplitudes[2] > 1000){
                    RightDistance = FilteredDistances[2] = Right(LPF_Calc3(Distances[2]));
                  }else{
                    RightDistance = FilteredDistances[2] = 500;
                  }
                }

          TxChannel = 3;
          channel = (channel+1)%3;
          OPT3101_StartMeasurementChannel(channel);
          StartTime = SysTick->VAL;
          i=i+1;
          }
          Controller_Right();

              if(i >= 100){
                i = 0;
          //      SetCursor(3, 5);
          //      OutUDec(SetPoint);
          //      SetCursor(3, 6);
          //      OutSDec(Error);
          //      SetCursor(3, 7);
          //      OutUDec(UL); OutChar(','); OutUDec(UR);
              }

          bump();

          hope = ((char)(EUSCI_A0->RXBUF));
          if(hope == 's'){ON = 0;}
          //WaitForInterrupt();
      }
    } //While(1)
}// Full Function
