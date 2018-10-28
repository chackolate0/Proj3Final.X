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

#define NUMBER_DATA_POINTS 30
#define SPIFLASH_PROG_ADDR 0x100
#define SPIFLASH_PROG_SIZE  3*NUMBER_DATA_POINTS*sizeof(signed short int)


int potVal;
enum displayState {XY, ZX, YZ};
int position = XY;
int buttonLock = 0;

char accelDisplay[80];
char sensitivityDisplay[80] = "Team 1: SENS: 2G";
float xVal = 1.303;
float yVal = -4.70;
float zVal = 0.132; 
int xPrecision;
int yPrecision;
int zPrecision;

int sensitivity = 2;

short int xshorVal;
short int yshorVal;
short int zshorVal;

int potVal;
int sampleRate;

char xyzSPIVals[180];
char reading = 0;

int main(void){
    BTN_Init();
    //RGBLED_Init();
    LED_Init();
    ADC_Init();
    LCD_Init();
    SSD_Init();
    ACL_Init();
    
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR); 
    OpenCoreTimer(CORE_TICK_RATE); //CoreTimer used for tenths of second capture  
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_5 | CT_INT_SUB_PRIOR_0));
    INTEnableSystemMultiVectoredInt();  //Enable Timer interrupts
    
    update_SSD(0);
    LCD_WriteStringAtPos(sensitivityDisplay, 0, 0);
    while(1){
        potVal = ADC_AnalogRead(2);
        sampleRate = (potVal * (10 - 1)) / 1023 + 1;
        
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
            if(sensitivity < 8){
                sensitivity*=2;
            } 
            buttonLock = 1;
        } 
        else if (BTN_GetValue('D') && !buttonLock) {//If BTND reset counter to 0sec
            delay_ms(50);
            if(sensitivity > 2){
                sensitivity /= 2;
            }
            buttonLock = 1;
        }
        if (buttonLock && !BTN_GetValue('C') && !BTN_GetValue('L') && !BTN_GetValue('R')
                && !BTN_GetValue('U') && !BTN_GetValue('D')) {
            delay_ms(50);
            buttonLock = 0;
        }
        
        if(xVal < 0){
            xPrecision = 1;
        }
        else{
            xPrecision = 10;
        }
        if(yVal < 0){
            yPrecision = 1;
        }
        else{
            yPrecision = 10;
        }
        if(zVal < 0){
            zPrecision = 1;
        }
        else{
            zPrecision = 10;
        }
        
        switch(position){
            case XY:
                sprintf(accelDisplay,"X:%.3f Y:%.3f", xVal, yVal);//sprintf(accelDisplay,"X:%.3f Y:%.3f", xVal, yVal);
                update_SSD((int)(zVal*100 * zPrecision));
                break;
            case ZX:    
                sprintf(accelDisplay,"Z:%.3f X:%.3f", zVal, xVal);
                update_SSD((int)(yVal*100 * yPrecision));
                break;
            case YZ:    
                sprintf(accelDisplay,"Y:%.3f Z:%.3f", yVal, zVal);
                update_SSD((int)(xVal*100 * xPrecision));
                break;
        }
        sprintf(sensitivityDisplay, "Team: 1 SENS: %dG", sensitivity);
        LCD_WriteStringAtPos(sensitivityDisplay, 0, 0);
        LCD_WriteStringAtPos(accelDisplay, 1,0);
    }
}

// void __ISR(_CORE_TIMER_VECTOR, ipl5) _CoreTimerHandler(void){
//     mCTClearIntFlag();
//     //ACL_ReadRawValues(posValues);
//     float xyzGvals[3];
//     ACL_ReadGValues(xyzGvals);
//     /*xshorVal=(((unsigned short)posValues[0]<<4)+((posValues[1])>>4));
//     yshorVal=(((unsigned short)posValues[2]<<4)+((posValues[3])>>4));
//     zshorVal=(((unsigned short)posValues[4]<<4)+((posValues[5])>>4));
//     if(xshorVal & (1<<11)){
//     xshorVal |= 0xF000; //xVal=(short)~(xVal)+1; 
//     }
//     if(yshorVal & (1<<11)){
//     yshorVal |= 0xF000; //yVal=(short)~(yVal)+1;
//     }
//     if(zshorVal & (1<<11)){
//     zshorVal |= 0xF000; //zVal=(short)~(zVal)+1;
//     }*/
//     xVal = xyzGvals[0];
//     yVal = xyzGvals[1];
//     zVal = xyzGvals[2];
//     UpdateCoreTimer(CORE_TICK_RATE* sampleRate);
// }

void __ISR(_CORE_TIMER_VECTOR, ipl5) _CoreTimerHandler(void){
    mCTClearIntFlag();
    float xyzGvals[3];
    ACL_ReadGValues(xyzGvals);
    xVal = xyzGvals[0];
    yVal = xyzGvals[1];
    zVal = xyzGvals[2];

    if(xVal>yVal && xVal>zVal)
        RGBLED_SetValue(255,0,0);
    else if(yVal>xVal  && yVal>zVal)
        RGBLED_SetValue(0,255,0);
    else if(zVal>xVal && zVal>yVal)
        RGBLED_SetValue(0,0,255);
    
    if(SWT_GetValue(1)){
        
        SPIFLASH_EraseAll();
        SPIFLASH_Read(SPIFLASH_PROG_ADDR,)
        SPIFLASH_ProgramPage(SPIFLASH_PROG_ADDR, xyzSPIVals,SPIFLASH_PROG_SIZE);
    }
    
    UpdateCoreTimer(CORE_TICK_RATE* sampleRate);
}

void update_SSD(int value) {
    int hunds, tens, ones, tenths;
    int dec1, dec2;
    char SSD1 = 0b0000000; //SSD setting for 1st SSD (LSD)
    char SSD2 = 0b0000000; //SSD setting for 2nd SSD
    char SSD3 = 0b0000000; //SSD setting for 3rd SSD 
    char SSD4 = 0b0000000; //SSD setting for 4th SSD (MSD)
    if (value < 0){
        SSD4 = 17;
        value = -1 * value;
        dec1 = 0;
        dec2 = 1;
    }
    else{
        dec1 = 1;
        dec2 = 0;
        hunds = floor(value / 1000);
        if (hunds > 0)
            SSD4 = hunds; //SSD4 = display_char[thous];
        else
            SSD4 = 0;
    }
    tens = floor((value % 1000) / 100);
    if (hunds == 0 && tens == 0)
        SSD3 = 0;
    else
        SSD3 = tens;
    SSD2 = ones = floor(value % 100 / 10);
    SSD1 = tenths = floor(value % 10);
    SSD_WriteDigits(SSD1, SSD2, SSD3, SSD4, 0, 0, dec2, dec1);
}

void delay_ms(int ms) {
    int i, counter;
    for (counter = 0; counter < ms; counter++) {
        for (i = 0; i < 300; i++) {
        } //software delay ~1 millisec 
    }
}