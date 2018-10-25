/* ************************************************************************** */
/** Project 3 Mid-Stage

  @Company
 CPEG Team 1
 * Alex Chacko & Matias Saavedra

  @File Name
 Proj3.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */

// PIC32MX370F512L Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FSRSSEL = PRIORITY_7     // Shadow Register Set Priority Select (SRS Priority 7)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2        // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20         // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1       // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = PRIPLL           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = XT            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2           // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576        // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is Disabled)
#pragma config JTAGEN = OFF              // JTAG Enable (JTAG Port Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF                // Program Flash Write Protect (Disable)
#pragma config BWP = OFF                // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

#ifndef _SUPPRESS_PLIB_WARNING
#define _SUPPRESS_PLIB_WARNING
#endif

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */
#include <xc.h>
#include "config.h"
#include <plib.h>
#include <stdio.h>
#include "utils.h"
#include "acl.h"
#include "adc.h"
#include "aic.h"
#include "btn.h"
#include "i2c.h"
#include "lcd.h"
#include "ssd.h"
#include "swt.h"
#include "uart.h"
#include <math.h>
#include "rgbled.h"

/* TODO:  Include other files here if needed. */
#define SYS_FREQ (80000000L)
#define INTSEC 10
#define CORE_TICK_RATE (SYS_FREQ/2/INTSEC)

int potVal;
enum displayState {XY, ZX, YZ};
int position = XY;
int buttonLock = 0;

char accelDisplay[80];
float xVal = 1.303;
float yVal = -4.70;
float zVal = 0.132; 
int xPrecision;
int yPrecision;
int zPrecision;

int main(void){
    BTN_Init();
    //RGBLED_Init();
    ADC_Init();
    LCD_Init();
    SSD_Init();
    
    LCD_WriteStringAtPos("Team: 1 SENS: 2G",0,0);
    update_SSD(0);
    
    while(1){
        if (BTN_GetValue('C') && !buttonLock) { //If BTNC is pressed stop timer and shifting
            delay_ms(50);
            buttonLock = 1;
        } 
        else if (BTN_GetValue('L') && !buttonLock) {//If BTNL shift mode to left
            delay_ms(50);
            if(position == XY){
                position = YZ;
            }
            else{
                position--;
            }
            buttonLock = 1;
        } 
        else if (BTN_GetValue('R') && !buttonLock) {//If BTNR shift mode to right
            delay_ms(50);
            if(position == YZ){
                position = XY;
            }
            else{
                position++;
            }
            buttonLock = 1;
        } 
        else if (BTN_GetValue('U') && !buttonLock) {//If BTNU set counter to count up
            delay_ms(50);
            //do stuff
            buttonLock = 1;
        } 
        else if (BTN_GetValue('D') && !buttonLock) {//If BTND reset counter to 0sec
            delay_ms(50);
            //do stuff
            buttonLock = 1;
        }
        if (buttonLock && !BTN_GetValue('C') && !BTN_GetValue('L') && !BTN_GetValue('R')
                && !BTN_GetValue('U') && !BTN_GetValue('D')) {
            delay_ms(50);
            buttonLock = 0;
        }
        
        if(xVal < 0){
            xPrecision = 2;
        }
        else{
            xPrecision = 3;
        }
        if(yVal < 0){
            yPrecision = 2;
        }
        else{
            yPrecision = 3;
        }
        if(zVal < 0){
            zPrecision = 2;
        }
        else{
            zPrecision = 3;
        }
        
        switch(position){
            case XY:
                sprintf(accelDisplay,"X:%.*f", xPrecision, xVal);//sprintf(accelDisplay,"X:%.3f Y:%.3f", xVal, yVal);
                break;
            case ZX:    
                sprintf(accelDisplay,"Z:%.*f X:%.*f", zVal, xVal, zPrecision, xPrecision);
                break;
            case YZ:    
                sprintf(accelDisplay,"Y:%.*f Z:%.*f", yVal, zVal, yPrecision, zPrecision);
                break;
        }
        
        LCD_WriteStringAtPos(accelDisplay, 1,0);
    }
}

/*void __ISR(_CORE_TIMER_VECTOR, ipl5) _CoreTimerHandler(void){
    mCTClearIntFlag();
    
    UpdateCoreTimer(CORE_TICK_RATE);
}*/

void update_SSD(int value) {
    int hunds, tens, ones, tenths;
    char SSD1 = 0b0000000; //SSD setting for 1st SSD (LSD)
    char SSD2 = 0b0000000; //SSD setting for 2nd SSD
    char SSD3 = 0b0000000; //SSD setting for 3rd SSD 
    char SSD4 = 0b0000000; //SSD setting for 4th SSD (MSD)
    hunds = floor(value / 1000);
    if (hunds > 0)
        SSD4 = hunds; //SSD4 = display_char[thous];
    else
        SSD4 = 17; //blank display
    tens = floor((value % 1000) / 100);
    if (hunds == 0 && tens == 0)
        SSD3 = 17; //blank display
    else
        SSD3 = tens;
    SSD2 = ones = floor(value % 100 / 10);
    SSD1 = tenths = floor(value % 10);
    SSD_WriteDigits(SSD1, SSD2, SSD3, SSD4, 0, 1, 0, 0);
}

void delay_ms(int ms) {
    int i, counter;
    for (counter = 0; counter < ms; counter++) {
        for (i = 0; i < 300; i++) {
        } //software delay ~1 millisec 
    }
}