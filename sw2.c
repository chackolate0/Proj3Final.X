if (SWT_GetValue(2) && !SWT_GetValue(1))
{
    SPIFLASH_Read(SPIFLASH_PROG_ADDR, spiOut, SPIFLASH_PROG_SIZE);
    int i;
    int buttonLock2 = 0;
    int dataSet = 0;
    unsigned char x[2];
    unsigned char y[2];
    unsigned char z[2];
    for (i = 0; i < 180; i = i + 6)
    {
        x[0] = spiOut[i];
        x[1] = spiOut[i + 1];
        y[0] = spiOut[i + 2];
        y[1] = spiOut[i + 3];
        z[0] = spiOut[i + 4];
        z[1] = spiOut[i + 5];
        spiOutConverted[(i / 2)] = ACL_ConvertRawToValueG(x);
        spiOutConverted[(i / 2) + 1] = ACL_ConvertRawToValueG(y);
        spiOutConverted[(i / 2) + 2] = ACL_ConvertRawToValueG(z);
    }
    while (SWT_GetValue(2))
    {
        char topDisplay[80];
        sprintf(topDisplay, "Team: 23 SET: %d ", (dataSet / 3) + 1);
        LCD_WriteStringAtPos(topDisplay, 0, 0);
        if (spiOutConverted[dataSet] < 0)
        {
            xPrecision = 1;
        }
        else
        {
            xPrecision = 10;
        }
        if (spiOutConverted[dataSet + 1] < 0)
        {
            yPrecision = 1;
        }
        else
        {
            yPrecision = 10;
        }
        if (spiOutConverted[dataSet + 2] < 0)
        {
            zPrecision = 1;
        }
        else
        {
            zPrecision = 10;
        }
        if (BTN_GetValue('U') && !buttonLock2)
        {
            delay_ms(50);
            buttonLock2 = 1;
            if (dataSet < 87)
            {
                dataSet = dataSet + 3;
            }
        }
        else if (BTN_GetValue('D') && !buttonLock2)
        {
            delay_ms(50);
            buttonLock2 = 1;
            if (dataSet > 0)
            {
                dataSet = dataSet - 3;
            }
        }
        else if (BTN_GetValue('L') && !buttonLock2)
        {
            delay_ms(50);
            if (position == XY)
            {
                position = YZ;
            }
            else
            {
                position--;
            }
            buttonLock2 = 1;
        }
        else if (BTN_GetValue('R') && !buttonLock2)
        {
            delay_ms(50);
            if (position == YZ)
            {
                position = XY;
            }
            else
            {
                position++;
            }
            buttonLock2 = 1;
        }
        if (buttonLock2 && !BTN_GetValue('C') && !BTN_GetValue('L') && !BTN_GetValue('R') && !BTN_GetValue('U') && !BTN_GetValue('D'))
        {
            delay_ms(50);
            buttonLock2 = 0;
        }
        switch (position)
        {
        case XY:
            sprintf(accelDisplay, "X:%.3f Y:%.3f", spiOutConverted[dataSet], spiOutConverted[dataSet + 1]);
            LCD_WriteStringAtPos(accelDisplay, 1, 0);
            update_SSD((int)(spiOutConverted[dataSet + 2] * 100 * zPrecision));
            break;
        case ZX:
            sprintf(accelDisplay, "Z:%.3f X:%.3f", spiOutConverted[dataSet + 2], spiOutConverted[dataSet]);
            LCD_WriteStringAtPos(accelDisplay, 1, 0);
            update_SSD((int)(spiOutConverted[dataSet + 1] * 100 * yPrecision));
            break;
        case YZ:
            sprintf(accelDisplay, "Y:%.3f Z:%.3f", spiOutConverted[dataSet + 1], spiOutConverted[dataSet + 2]);
            LCD_WriteStringAtPos(accelDisplay, 1, 0);
            update_SSD((int)(spiOutConverted[dataSet] * 100 * xPrecision));
            break;
        }
    }
}