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

/* TODO:  Include other files here if needed. */
#define SYS_FREQ (80000000L)
#define INT_SEC 10
#define CORE_TICK_RATE (SYS_FREQ/2/INT_SEC)

int potVal;

int main(void){
    OpenCoreTimer(CORE_TICK_RATE);
    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    mConfigIntCoreTimer(CT_INT_ON | CT_INT_PRIOR_5 | CT_INT_SUB_PRIOR_0);
    INTEnableSystemMultiVectoredInt();
    ACL_Init();
    AIC_Init();
    BTN_Init();
    LCD_Init();
    SSD_Init();
    SWT_Init();
    //UART_Init(baud);
    char coords[80];
    char sensitivity;
    char stringVal[80];
    int accelX=10;
    int accelY=6;
    int accelZ=14;
    enum states{XY, ZX, YZ};
    int axisState = XY;
    unsigned char accelData[06];
    
    while(1){
        sprintf(stringVal,"Team: 1  SENS: %dG", sensitivity);
        LCD_WriteStringAtPos(stringVal, 0, 0);
        potVal=AIC_Val()*(900/1024)+100;
        I2C_Init(potVal);
        if(BTN_GetValue('R')){
            delay_ms(1000);
            if(axisState == YZ)
                axisState = XY;
            else
                axisState++;
        }
        else if(BTN_GetValue('L')){
            delay_ms(1000);
            if(axisState == XY)
                axisState = YZ;
            else
                axisState--;
        }

        if(BTN_GetValue('U')){
            ACL_ReadRawValues(accelData);
        }

        switch(axisState){
            case XY: sprintf(coords, "X:%.3f Y:%.3f", accelX, accelY);
            break;
            case ZX: sprintf(coords, "Z:%.3f X: %.3f", accelZ, accelX);
            break;
            case YZ: sprintf(coords, "Y:%.3f Z: %.3f", accelY, accelZ);
            break;
        }
        LCD_WriteStringAtPos(coords,1,0);
    }
    
    
}

void __ISR(_CORE_TIMER_VECTOR, ipl5) _CoreTimerHandler(void){
    mCTClearIntFlag();
    
    UpdateCoreTimer(CORE_TICK_RATE);
}

void delay_ms(int ms) {
    int i, counter;
    for (counter = 0; counter < ms; counter++) {
        for (i = 0; i < 300; i++) {
        } //software delay ~1 millisec 
    }
}