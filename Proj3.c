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
 PROJECT 3
 */

// PIC32MX370F512L Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FSRSSEL = PRIORITY_7 // Shadow Register Set Priority Select (SRS Priority 7)
#pragma config PMDL1WAY = ON        // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON         // Peripheral Pin Select Configuration (Allow only one reconfiguration)

// DEVCFG2
#pragma config FPLLIDIV = DIV_2 // PLL Input Divider (2x Divider)
#pragma config FPLLMUL = MUL_20 // PLL Multiplier (20x Multiplier)
#pragma config FPLLODIV = DIV_1 // System PLL Output Clock Divider (PLL Divide by 1)

// DEVCFG1
#pragma config FNOSC = PRIPLL       // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config FSOSCEN = ON         // Secondary Oscillator Enable (Enabled)
#pragma config IESO = ON            // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = XT         // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF       // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FPBDIV = DIV_2       // Peripheral Clock Divisor (Pb_Clk is Sys_Clk/2)
#pragma config FCKSM = CSDCMD       // Clock Switching and Monitor Selection (Clock Switch Disable, FSCM Disabled)
#pragma config WDTPS = PS1048576    // Watchdog Timer Postscaler (1:1048576)
#pragma config WINDIS = OFF         // Watchdog Timer Window Enable (Watchdog Timer is in Non-Window Mode)
#pragma config FWDTEN = ON          // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25 // Watchdog Timer Window Size (Window Size is 25%)

// DEVCFG0
#pragma config DEBUG = OFF       // Background Debugger Enable (Debugger is Disabled)
#pragma config JTAGEN = OFF      // JTAG Enable (JTAG Port Disabled)
#pragma config ICESEL = ICS_PGx1 // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config PWP = OFF         // Program Flash Write Protect (Disable)
#pragma config BWP = OFF         // Boot Flash Write Protect bit (Protection Disabled)
#pragma config CP = OFF          // Code Protect (Protection Disabled)

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
#include "led.h"

/* TODO:  Include other files here if needed. */
#define SYS_FREQ (80000000L)
#define INTSEC 10
#define CORE_TICK_RATE (SYS_FREQ / 2 / INTSEC)

#define NUMBER_DATA_POINTS 30
#define SPIFLASH_PROG_ADDR 0x100
#define SPIFLASH_PROG_SIZE 3 * NUMBER_DATA_POINTS * sizeof(signed short int)


enum aclState {
    XY,
    ZX,
    YZ
};

int posXYZ = XY;
int btnLck = 0;

char aclDisp[80];
char sensitivityDisplay[80] = "Team 1: SENS: 2G";
unsigned char x = 1.303;
unsigned char y = -4.70;
unsigned char z = 0.132;
int xPrec;
int yPrec;
int zPrec;

int sensitivity = 2;


int potVal;
int sampleRate;

//variables for sw1
char xyzSPIVals[180];
char xyzSPIOut[180];
char reading = 0;
int counter = 0;
int flashes = 0;
char rawVals[6];

//variables for sw2
char sw2 = 0;
unsigned char gVals[80];

//variables for uart
char uartMsg[80];
int uartCount = 0;
char sw7 = 0;

int main(void) {
    BTN_Init();
    RGBLED_Init();
    LED_Init();
    ADC_Init();
    LCD_Init();
    SSD_Init();
    ACL_Init();
    SWT_Init();
    UART_Init(9600);
    SPIFLASH_Init();

    INTConfigureSystem(INT_SYSTEM_CONFIG_MULT_VECTOR);
    OpenCoreTimer(CORE_TICK_RATE); //CoreTimer used for tenths of second capture
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_5 | CT_INT_SUB_PRIOR_0));
    INTEnableSystemMultiVectoredInt(); //Enable Timer interrupts

    update_SSD(0);
    LCD_WriteStringAtPos(sensitivityDisplay, 0, 0);
    while (1) {
        potVal = ADC_AnalogRead(2);
        sampleRate = (potVal * (10 - 1)) / 1023 + 1;

            if (BTN_GetValue('L') && !btnLck) { 
            delay_ms(50);
            if (posXYZ == XY) {
                posXYZ = YZ;
            } else {
                posXYZ--;
            }
            btnLck = 1;
        } else if (BTN_GetValue('R') && !btnLck) { 
            delay_ms(50);
            if (posXYZ == YZ) {
                posXYZ = XY;
            } else {
                posXYZ++;
            }
            btnLck = 1;
        } else if (BTN_GetValue('U') && !btnLck) { 
            delay_ms(50);
            if (sensitivity < 8) {
                sensitivity *= 2;
                delay_ms(100);
                // switch (sensitivity)
                // {
                // case 2:
                //     ACL_SetRange(0);
                // case 4:
                //     ACL_SetRange(1);
                // case 8:
                //     ACL_SetRange(2);
                // }
            }
            btnLck = 1;
        } else if (BTN_GetValue('D') && !btnLck) { 
            delay_ms(50);
            if (sensitivity > 2) {
                sensitivity /= 2;
                delay_ms(100);
                // switch (sensitivity)
                // {
                // case 2:
                //     ACL_SetRange(0);
                // case 4:
                //     ACL_SetRange(1);
                // case 8:
                //     ACL_SetRange(2);
                // }
            }
            btnLck = 1;
        }
        if (btnLck && !BTN_GetValue('C') && !BTN_GetValue('L') && !BTN_GetValue('R') && !BTN_GetValue('U') && !BTN_GetValue('D')) {
            delay_ms(50);
            btnLck = 0;
        }

        if (x < 0) {
            xPrec = 1;
        } else {
            xPrec = 10;
        }
        if (y < 0) {
            yPrec = 1;
        } else {
            yPrec = 10;
        }
        if (z < 0) {
            zPrec = 1;
        } else {
            zPrec = 10;
        }

        switch (posXYZ) {
            case XY:
                sprintf(aclDisp, "X:%.3f Y:%.3f", x, y);
                update_SSD((int) (z * 100 * zPrec));
                break;
            case ZX:
                sprintf(aclDisp, "Z:%.3f X:%.3f", z, x);
                update_SSD((int) (y * 100 * yPrec));
                break;
            case YZ:
                sprintf(aclDisp, "Y:%.3f Z:%.3f", y, z);
                update_SSD((int) (x * 100 * xPrec));
                break;
        }

        //SW1 Controls
        if (reading == 1 && counter <= 180) {
            if (flashes == 0) {
                flashes++;
                LCD_WriteStringAtPos("Erasing Flash", 0, 0);
                SPIFLASH_EraseAll();
            } else if (flashes < 31) {
                int i = 0;
                LCD_WriteStringAtPos("Writing to Flash", 0, 0);
                for (i = 0; i < 36; i++) {
                    xyzSPIVals[6 * (flashes - 1) + i/6] = rawVals[i/6];
                    counter++;
                }
                flashes++;
            } else if (counter == 180) {
                SPIFLASH_ProgramPage(SPIFLASH_PROG_ADDR, xyzSPIVals, SPIFLASH_PROG_SIZE);
                LCD_WriteStringAtPos("Written to Flash", 0, 0);
                delay_ms(5000);
            }
        }
        uartCount = 0;

        //SW2 Controls
        if (sw2 == 1) {
            int i;
            int data = 0;
            int btnLock = 0;
            char disp2[80];
            unsigned char xTemp[2];
            unsigned char yTemp[2];
            unsigned char zTemp[2];
            SPIFLASH_Read(SPIFLASH_PROG_ADDR, xyzSPIOut, SPIFLASH_PROG_SIZE);

            for (i = 0; i < 90; i = i + 6) {
                xTemp[0] = xyzSPIVals[i/2];
                xTemp[1] = xyzSPIVals[i/2 + 1];
                yTemp[0] = xyzSPIVals[i/2 + 2];
                yTemp[1] = xyzSPIVals[i/2 + 3];
                zTemp[0] = xyzSPIVals[i/2 + 4];
                zTemp[1] = xyzSPIVals[i/2 + 5];
                xyzSPIOut[i] = ACL_ConvertRawToValueG(xTemp);
                xyzSPIOut[i + 1] = ACL_ConvertRawToValueG(yTemp);
                xyzSPIOut[i + 2] = ACL_ConvertRawToValueG(zTemp);
            }

            while (sw2 == 1) {
                if (BTN_GetValue('U') && !btnLock) { //If BTNU set counter to count up
                    delay_ms(50);
                    if (data < 87) {
                        data += 3;
                    }
                    btnLock = 1;
                } else if (BTN_GetValue('D') && !btnLock) { //If BTND reset counter to 0sec
                    delay_ms(50);
                    if (data > 0) {
                        data -= 3;
                    }
                    btnLock = 1;
                } else if (BTN_GetValue('L') && !btnLock) { //If BTNL shift mode to left
                    delay_ms(50);
                    if (posXYZ == XY) {
                        posXYZ = YZ;
                    } else {
                        posXYZ--;
                    }
                    btnLock = 1;
                } else if (BTN_GetValue('R') && !btnLock) { //If BTNR shift mode to right
                    delay_ms(50);
                    if (posXYZ == YZ) {
                        posXYZ = XY;
                    } else {
                        posXYZ++;
                    }
                    btnLock = 1;
                }
                sprintf(disp2, "Team: 1 SET: %d ", (data / 3) + 1);
                LCD_WriteStringAtPos(disp2, 0, 0);
                if (xyzSPIOut[data + 1] < 0) {
                    yPrec = 1;
                } else {
                    yPrec = 10;
                }
                if (xyzSPIOut[data] < 0) {
                    xPrec = 1;
                } else {
                    xPrec = 10;
                }
                if (xyzSPIOut[data + 2] < 0) {
                    zPrec = 1;
                } else {
                    zPrec = 10;
                }

                switch (posXYZ) {
                    case XY:
                        sprintf(aclDisp, "X:%.3f Y:%.3f", ACL_ConvertRawToValueG(x), ACL_ConvertRawToValueG(y));
                        LCD_WriteStringAtPos(aclDisp, 1, 0);
                        update_SSD((int) (data + 2 * 100 * zPrec));
                        break;
                    case ZX:
                        sprintf(aclDisp, "Z:%.3f X:%.3f", ACL_ConvertRawToValueG(z), ACL_ConvertRawToValueG(x));
                        LCD_WriteStringAtPos(aclDisp, 1, 0);
                        update_SSD((int) (data + 1 * 100 * yPrec));
                        break;
                    case YZ:
                        sprintf(aclDisp, "Y:%.3f Z:%.3f", ACL_ConvertRawToValueG(y), ACL_ConvertRawToValueG(z));
                        LCD_WriteStringAtPos(aclDisp, 1, 0);
                        update_SSD((int) (data * 100 * xPrec));
                        break;
                }
            }
        }

        while (SWT_GetValue(6)) {
            sprintf(uartMsg, "%d,%6.4f,%6.4f,%6.4f\n\r", uartCount, x, y, z);
            UART_PutString(uartMsg);
            uartCount++;
        }

        if (sw7) {
            int i;
            int data = 0;
            int btnLock = 0;
            char disp2[80];
            unsigned char x[2];
            unsigned char y[2];
            unsigned char z[2];
            SPIFLASH_Read(SPIFLASH_PROG_ADDR, xyzSPIOut, SPIFLASH_PROG_SIZE);

            for (i = 0; i < 180; i = i + 6) {
                x[0] = xyzSPIVals[i];
                x[1] = xyzSPIVals[i + 1];
                y[0] = xyzSPIVals[i + 2];
                y[1] = xyzSPIVals[i + 3];
                z[0] = xyzSPIVals[i + 4];
                z[1] = xyzSPIVals[i + 5];
                xyzSPIOut[(i / 2)] = ACL_ConvertRawToValueG(x);
                xyzSPIOut[(i / 2) + 1] = ACL_ConvertRawToValueG(y);
                xyzSPIOut[(i / 2) + 2] = ACL_ConvertRawToValueG(z);
            }

            while (sw7 == 1) {
                if (BTN_GetValue('U') && !btnLock) { //If BTNU set counter to count up
                    delay_ms(50);
                    if (data < 87) {
                        data += 3;
                    }
                    btnLock = 1;
                } else if (BTN_GetValue('D') && !btnLock) { //If BTND reset counter to 0sec
                    delay_ms(50);
                    if (data > 0) {
                        data -= 3;
                    }
                    btnLock = 1;
                } else if (BTN_GetValue('L') && !btnLock) { //If BTNL shift mode to left
                    delay_ms(50);
                    if (posXYZ == XY) {
                        posXYZ = YZ;
                    } else {
                        posXYZ--;
                    }
                    btnLock = 1;
                } else if (BTN_GetValue('R') && !btnLock) { //If BTNR shift mode to right
                    delay_ms(50);
                    if (posXYZ == YZ) {
                        posXYZ = XY;
                    } else {
                        posXYZ++;
                    }
                    btnLock = 1;
                }
                sprintf(disp2, "Team: 1 SET: %d ", (data / 3) + 1);
                LCD_WriteStringAtPos(disp2, 0, 0);
                if (xyzSPIOut[data] < 0) {
                    xPrec = 1;
                } else {
                    xPrec = 10;
                }
                if (xyzSPIOut[data + 1] < 0) {
                    yPrec = 1;
                } else {
                    yPrec = 10;
                }
                if (xyzSPIOut[data + 2] < 0) {
                    zPrec = 1;
                } else {
                    zPrec = 10;
                }

                switch (posXYZ) {
                    case XY:
                        sprintf(aclDisp, "X:%.3f Y:%.3f", xyzSPIOut[data], xyzSPIOut[data + 1]);
                        LCD_WriteStringAtPos(aclDisp, 1, 0);
                        update_SSD((int) (z[data + 2] * 100 * zPrec));
                        break;
                    case ZX:
                        sprintf(aclDisp, "Z:%.3f X:%.3f", xyzSPIOut[data + 2], xyzSPIOut[data]);
                        LCD_WriteStringAtPos(aclDisp, 1, 0);
                        update_SSD((int) (y[data + 1] * 100 * yPrec));
                        break;
                    case YZ:
                        sprintf(aclDisp, "Y:%.3f Z:%.3f", xyzSPIOut[data + 1], xyzSPIOut[data + 2]);
                        LCD_WriteStringAtPos(aclDisp, 1, 0);
                        update_SSD((int) (x[data] * 100 * xPrec));
                        break;
                }
            sprintf(uartMsg, "%d,%6.4f,%6.4f,%6.4f\n\r", uartCount, x, y, z);
            UART_PutString(uartMsg);
            uartCount++;
            }
        }

        sprintf(sensitivityDisplay, "Team: 1 SENS: %dG", sensitivity);
        LCD_WriteStringAtPos(sensitivityDisplay, 0, 0);
        LCD_WriteStringAtPos(aclDisp, 1, 0);

        LED_SetValue(1, SWT_GetValue(1));
        LED_SetValue(2, SWT_GetValue(2));
        LED_SetValue(6, SWT_GetValue(6));
        LED_SetValue(7, SWT_GetValue(7));
    }
}



void __ISR(_CORE_TIMER_VECTOR, ipl5) _CoreTimerHandler(void) {
    ACL_ReadRawValues(rawVals);
    mCTClearIntFlag();
    float xyzGvals[3];
    ACL_ReadGValues(xyzGvals);
    x = xyzGvals[0];
    y = xyzGvals[1];
    z = xyzGvals[2];

    if (fabs(x) > fabs(y) && fabs(x) > fabs(z))
        RGBLED_SetValue(255, 0, 0);
    else if (fabs(y) > fabs(x) && fabs(y) > fabs(z))
        RGBLED_SetValue(0, 255, 0);
    else if (fabs(z) > fabs(x) && fabs(z) > fabs(y))
        RGBLED_SetValue(0, 0, 255);

    if (SWT_GetValue(1) && !SWT_GetValue(2) && reading == 0) {
        reading = 1;
    }
    if (SWT_GetValue(1) == 0 && reading == 1) {
        flashes = 0;
        counter = 0;
        reading = 0;
    }
    if (SWT_GetValue(2) && !SWT_GetValue(1) && sw2 == 0) {
        sw2 = 1;
    }
    if (SWT_GetValue(2) == 0) {
        sw2 = 0;
    }
    if (SWT_GetValue(7)) {
        sw7 = 0;
    }

    UpdateCoreTimer(CORE_TICK_RATE * sampleRate);
}

void update_SSD(int value) {
    int hunds, tens, ones, tenths;
    int dec1, dec2;
    char SSD1 = 0b0000000; //SSD setting for 1st SSD (LSD)
    char SSD2 = 0b0000000; //SSD setting for 2nd SSD
    char SSD3 = 0b0000000; //SSD setting for 3rd SSD
    char SSD4 = 0b0000000; //SSD setting for 4th SSD (MSD)
    if (value < 0) {
        SSD4 = 17;
        value = -1 * value;
        dec1 = 0;
        dec2 = 1;
    } else {
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