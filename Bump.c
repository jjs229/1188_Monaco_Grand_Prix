

// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

#include <stdint.h>
#include "msp.h"
#include "..\inc\Motor.h"
#include "..\inc\PWM.h"

uint32_t temp;
void PORT4_IRQHandler(void){
    P4->IFG &= ~0x010;// clr flag
    P4->OUT |= 0xFF;

    // write this as part of Lab 14
}


// Initialize Bump sensors
// Make six Port 4 pins inputs
// Activate interface pullup
// pins 7,6,5,3,2,0
void Bump_Init(void){
    P4->SEL0 &= ~0x00;  //ez INIT
    P4->SEL1 &= ~0x00;
    P4->DIR &= 0x00;
    P4->REN |= 0xFF;
  
}
// Read current state of 6 switches
// Returns a 6-bit positive logic result (0 to 63)
// bit 5 - 0
uint8_t Bump_Read(void){

//    FUCK THI BROKEN ASS BUMP SENSOR
    // BUMPERS THAT DONT FUCKING WORK
//    2 IN FROM LEFT original
//    all the way to left 4.22
//    3rd in from left 4.24
    return P4->IN|0xE2;
}

