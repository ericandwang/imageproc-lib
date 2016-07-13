/*
* Copyright (c) 2009 - 2010, Regents of the University of California
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* - Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
* - Neither the name of the University of California, Berkeley nor the names
*   of its contributors may be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
*
* Synchronous Software Servo (RC PWM) Module
*
* by Humphrey Hu
*
* v.alpha
*
* Revisions:
*  Humphrey Hu             2011-9-23      Initial implementation
*  Humphrey Hu             2012-6-21      Switched to normalized float inputs
*
*/

// Justin Yim 2016-7-12
#if 0
#include "sync_servo.h"
#include "pwm.h"

void servoSetup(void) {
/* Holds the  PWM interrupt configuration value*/
    unsigned int config;
/* Holds the value to be loaded into dutycycle register */
    unsigned int period;
/* Holds the value to be loaded into special event compare register */
    unsigned int sptime;
/* Holds PWM configuration value  */
    unsigned int config1;
/* Holds the value be loaded into PWMCON1 register */
    unsigned int config2;
/* Holds the value to configure the special event trigger
   postscale and dutycycle */
    unsigned int config3;
/* The value of ‘dutycyclereg’ determines the duty cycle
   register(PDCx) to be written */
    unsigned int dutycyclereg;
    unsigned int dutycycle;
    unsigned char updatedisable;
 
/* Configure PWM interrupt enable/disable and set interrupt
   priorities */
    config = (PWM_INT_DIS & PWM_INT_PR1
             & PWM_FLTA_DIS_INT & PWM_FLTA_INT_PR0
             & PWM_FLTB_DIS_INT & PWM_FLTB_INT_PR0);
    ConfigIntMCPWM( config );
/* Configure PWM to generate square wave of 50% duty  cycle */
    dutycyclereg  = 1;
    dutycycle     = 0x3FFF;
    updatedisable = 0;
 
    SetDCMCPWM(dutycyclereg,dutycycle,updatedisable);
    period = 0x7fff;
    sptime = 0x0;
    config1 = (PWM_EN & PWM_IDLE_STOP & PWM_OP_SCALE16
               & PWM_IPCLK_SCALE16 &
                 PWM_MOD_UPDN);
    config2 = (PWM_MOD4_IND & 
               PWM_PDIS4H & PWM_PDIS3H & PWM_PDIS2H & PWM_PDIS1H & 
               PWM_PEN4L & PWM_PDIS3L & PWM_PDIS2L & PWM_PDIS1L);
    config3  = (PWM_SEVOPS1 & PWM_OSYNC_PWM &  PWM_UEN);
    OpenMCPWM(period,sptime,config1,config2,config3);
}

void servoSet(float ang) {

}

void servoStart(void) {

}

void servoStop(void) {

}
#endif


#if 1
#include "sync_servo.h"
#include "timer.h"
#include "outcompare.h"

void servoSetup(void) {
    /*
    unsigned int match_value;
    ConfigIntTimer3(T3_INT_PRIOR_1 & T3_INT_OFF);
    WriteTimer3(0);
    match_value = 0xFFFF;
    OpenTimer3(T3_ON & T3_GATE_OFF & T3_IDLE_STOP &
               T3_PS_1_64 &
               T3_SOURCE_INT, match_value);
    */

    // Manually set up timer
    T3CON = 0x8020; //prescale 1:64
    PR3 = 12500; // 1:50
    TMR3  = 0x0000;

#if 1
/* Holds the value at which OCx Pin to be driven high */
    unsigned int pulse_start;
/* Holds the value at which OCx Pin to be driven low */
    unsigned int pulse_stop;
/* Turn off OC2 module */
    CloseOC2();
/* Configure output compare1 interrupt */
    ConfigIntOC2(OC_INT_OFF & OC_INT_PRIOR_5);
/* Configure OC2 module for required pulse width */
    pulse_start = 938;
    pulse_stop = 938;

/* Configure Output Compare module to 'initialize OCx pin
low and generate continuous pulse'mode */
    OpenOC2(OC_IDLE_CON & OC_TIMER3_SRC &
            OC_PWM_FAULT_PIN_DISABLE,
            pulse_stop, pulse_start);
#endif
    /*
    // Manually set up output compare
    OC2CON = 0x200E;
    OC2R = 0x1FFF;
    OC2RS = 0x1FFF;
    */
}

void servoSet(float ang) {
    OC2RS = (int)(625+625*ang);
}

void servoStart(void) {

}

void servoStop(void) {

}

/* This is ISR corresponding to OC1 interrupt */
/*
void __attribute__((__interrupt__)) _OC1Interrupt(void)
{
  IFS0bits.OC1IF = 0;
}
*/

#endif

#if 0
#include "sync_servo.h"

void servoSetup(void) {
    // Initialize Output Compare Module in PWM mode
    OC2CONbits.OCM = 0b000;
    OC2R=100;
    OC2RS=200;
    OC2CONbits.OCTSEL = 0;
    OC2R= 100;
    OC2CONbits.OCM = 0b110;
    // Initialize Timer2
    T2CONbits.TON = 0;
    T2CONbits.TCS = 0;
    T2CONbits.TGATE = 0;
    T2CONbits.TCKPS = 0b00;
    TMR2 = 0x00;
    PR2 = 500;
    
    /*
    // Define a Buffer in DMA RAM to store duty cycle information
    unsigned int BufferA[256] __attribute__((space(dma)));
    // Setup and Enable DMA Channel
    DMA0CONbits.AMODE = 0b00;
    DMA0CONbits.MODE = 0b00;
    DMA0CONbits.DIR = 0;
    DMA0PAD = (int)&OC1RS;
    DMA0REQ = 7;
    DMA0CNT = 255;
    DMA0STA = __builtin_dmaoffset(&BufferA);
    IFS0bits.DMA0IF = 0;
    IEC0bits.DMA0IE = 1;
    DMA0CONbits.CHEN = 1;
    */
    // Enable Timer
    T2CONbits.TON = 1;
}


// DMA Interrupt Handler
void __attribute__((__interrupt__)) _DMA0Interrupt(void) {
    IFS0bits.DMA0IF = 0; // Clear the DMA0 Interrupt Flag 
}

void servoSet(float ang) {

}

void servoStart(void) {

}

void servoStop(void) {

}
#endif


#if 0

#include "sync_servo.h"
#include "utils.h"
#include "timer.h"

#define SERVO_A                 _RE6 // _LATE1 // JY edits

#define FCY                     (40000000)
#define PWM_FREQUENCY           (50)
#define PWM_PERIOD              (5000) // 40 MIPS/(prescale * frequency)
// JY edits (12500)

#define PULSE_MAX_LENGTH        400 // JY edits (1250)
#define PULSE_MIN_LENGTH        200 // JY edits (625)
#define PULSE_AMPLITUDE         ((PULSE_MAX_LENGTH - PULSE_MIN_LENGTH)/2) // 312
#define PULSE_ZERO_LENGTH       ((PULSE_MAX_LENGTH + PULSE_MIN_LENGTH)/2) // 937

// =========== Function Prototypes ============================================ 
static void setupTimer4(unsigned int frequency);

// =========== Static Variables ===============================================
static unsigned char pin_is_high;
static unsigned int pulse_length, setpoint;
static int current_setpoint;

// =========== Public Methods =================================================
void servoSetup(void) {

    setupTimer4(PWM_FREQUENCY);

    //SERVO_A_TRIS = 0; // JY edits (added for output configure)

    pin_is_high = 0;
    SERVO_A = 0;
    current_setpoint = 0;
    setpoint = PULSE_ZERO_LENGTH;
    pulse_length = PULSE_ZERO_LENGTH;

    servoStop();

}

// Set is in +- 1.0, Track is in +- 312
void servoSet(float set) {

    if(set > 1.0) { set = 1.0; }
    else if(set < -1.0) { set = -1.0; }
    
    setpoint = (unsigned int)(PULSE_ZERO_LENGTH + (int)(set*PULSE_AMPLITUDE));

}

void servoStart(void) {

    WriteTimer4(0);
    _T4IE = 1;

}

void servoStop(void) {

    _T4IE = 0;
}

// =========== Private Functions ===============================================

void __attribute__((interrupt, no_auto_psv)) _T4Interrupt(void) {

    if(pin_is_high) {
        
        SERVO_A = 0; // End pulse
        pin_is_high = 0;
        
        WriteTimer4(0); // Wait until next pulse start
        PR4 = PWM_PERIOD - pulse_length;
    } else {
                
        pulse_length = setpoint; // Buffer pulse length
        WriteTimer4(0);
        PR4 = pulse_length;
        
        SERVO_A = 1; // Begin pulse
        pin_is_high = 1;
    }

    _T4IF = 0;
    
}

void setupTimer4(unsigned int frequency) {

    unsigned int con_reg, period;

    // prescale 1:64
    con_reg =     T4_ON & 
    T4_IDLE_STOP & 
    T4_GATE_OFF & 
    T4_PS_1_64 & 
    T4_SOURCE_INT &
    T4_32BIT_MODE_OFF;

    // period value = Fcy/(prescale*Ftimer)
    period = FCY/(64*frequency); 

    OpenTimer4(con_reg, period);
    ConfigIntTimer4(T4_INT_PRIOR_5 & T4_INT_ON);

}

#endif
