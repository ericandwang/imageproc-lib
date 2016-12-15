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
* Ouput Compare Servo (RC PWM) Module
*   Replacing Synchronous Software Servo (RC PWM) Module
*
* by Justin Yim
*   overwriting module by Humphrey Hu
*
* Revisions:
*  Justin Yim               2016-7-13      Initial implementation
*
*/

#include "sync_servo.h"
#include "timer.h"
#include "outcompare.h"

/* 
 * This module in original implemented uses Timer 3 and Output Compare 2
 *
 * Reference material:
 * dsPIC33FJXXXMCX06A/X08A/X10A Data Sheet:
 *      http://ww1.microchip.com/downloads/en/DeviceDoc/70594C.pdf
 * dsPIC33F/PIC24H Family Reference Manual sections 11 and 13:
 *      http://ww1.microchip.com/downloads/en/DeviceDoc/70205D.pdf
 *      http://www.microchip.com.tw/Data_CD/Reference%20Manuals/16-Bits%20Family%20Reference%20Manual/dsPIC33F%20FRM%20Section%2013.%20Output%20Compare%20(DS70209A).pdf
 *  16-bit Compiler Peripheral Libraries documentation:
 *      file:///Applications/microchip/xc16/v1.24/docs/periph_libs/dsPIC30F_dsPIC33F_PIC24H_dsPIC33E_PIC24E_Timers_Help.htm#_Toc154543222
 *      file:///Applications/microchip/xc16/v1.24/docs/periph_libs/dsPIC30F_dsPIC33F_dsPIC33E_PIC24H_PIC24E_OC_Library_Help.htm#_Toc154481094
 * ImageProc board schematic:
 *      https://github.com/biomimetics/imageproc_pcb
 */

unsigned int pulseLen = 938; // 938 = 1.5ms at 40MIPS, 1:64 prescale

void servoSetup(void) {
    // Set up timer
    // Configure by manually setting registers
    T3CON = 0x8020; // timer on and prescale 1:64
    PR3 = 12500; // 50Hz frequency at 40MIPS, 1:64 prescale

    // Configure output compare
    /*
    // Configure with function
    unsigned int pulse_start, pulse_stop;
    pulse_start = pulseLen;
    pulse_stop = pulseLen;
   
    CloseOC2();
    OpenOC2(OC_IDLE_CON & OC_TIMER3_SRC &
            OC_PWM_FAULT_PIN_DISABLE,
            pulse_stop, pulse_start);
    */
    //*
    // Configure by manually setting registers
    OC2CON = 0x000E; // run in idle, use timer 3, PWM with fault disabled
    //OC2R = 938; // in PWM mode OC2R is read only
    OC2RS = pulseLen; // 1.5ms pulse = neutral position
    //*/
    
    // Disable interrupt
    ConfigIntOC2(OC_INT_OFF & OC_INT_PRIOR_5);

}

void servoSet(float ang) {
    pulseLen = (unsigned int)(938+312*ang); // 1ms to 2ms
    OC2RS = pulseLen;
}

void servoStart(void) {
    OC2RS = pulseLen;
}

void servoStop(void) {
    OC2RS = 0;
}

