//*****************************************************************************
//
// Copyright (C) 2017 Universitat Oberta de Cataluya
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the
//  distribution.
//
//  Neither the name of Texas Instruments Incorporated nor the names of
//  its contributors may be used to endorse or promote products derived
//  from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#include "driverlib.h"
#include "math.h"

/* ADC temperature reference calibration value */
uint32_t adcRefTempCal_1_2v_30;
uint32_t adcRefTempCal_1_2v_85;

/* Initialization of the ADC14 to measure the temperature of the internal sensor */
void temperatureSensorInit(void)
{
    // Read the ADC temperature reference calibration value
    adcRefTempCal_1_2v_30 = TLV->ADC14_REF1P2V_TS30C;
    adcRefTempCal_1_2v_85 = TLV->ADC14_REF1P2V_TS85C;

    // Initialize the shared reference module
    // By default, REFMSTR=1 => REFCTL is used to configure the internal reference
    // If ref generator busy, WAIT
    while(REF_A->CTL0 & REF_A_CTL0_GENBUSY);

    // Enable internal 1.2V reference and turn it on
    REF_A->CTL0 |= REF_A_CTL0_VSEL_0 | REF_A_CTL0_ON;

    // Enable temperature sensor
    REF_A->CTL0 &= ~REF_A_CTL0_TCOFF;

    // Configure ADC in pulse sample mode; ADC14_CTL0_SC trigger
    // ADC ON, temperature sample period > 5us
    ADC14->CTL0 |= ADC14_CTL0_SHT0_6 | ADC14_CTL0_ON | ADC14_CTL0_SHP;

    // Enable internal temperature sensor
    ADC14->CTL1 |= ADC14_CTL1_TCMAP;

    // ADC input channel A22 for temperature sensing
    ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_INCH_22;

    // Wait for reference generator to settle
    while(!(REF_A->CTL0 & REF_A_CTL0_GENRDY));

    // ADC enable conversion
    ADC14->CTL0 |= ADC14_CTL0_ENC;
}

/* Read a value of temperature from the internal temperature sensor */
float temperatureSensorSample(void)
{
    float temperature;

    // Sampling and conversion start
    ADC14->CTL0 |= ADC14_CTL0_SC;

    // Wait until conversion finishes
    while(ADC14->CTL0 & ADC14_CTL0_BUSY);

    // Temperature in Celsius
    temperature = (((float) ADC14->MEM[0] - adcRefTempCal_1_2v_30) * (85 - 30)) /
            (adcRefTempCal_1_2v_85 - adcRefTempCal_1_2v_30) + 30.0f;

    return temperature;
}

/* Reverses a string 'str' of length 'len' */
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)

    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}

/* Converts a given integer x to string str[].  d is the number
  of digits required in output. If d is more than the number
  of digits in x, then 0s are added at the beginning */
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    /* If number of digits required is more, then add 0s at the beginning */
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}

/* Converts a floating point number to string. */
void ftoa(float n, char *res, int afterpoint)
{
    /* Extract integer part */
    int ipart = (int)n;

    /* Extract floating part */
    float fpart = n - (float)ipart;

    /* convert integer part to string */
    int i = intToStr(ipart, res, 0);

    /* check for display option after point */
    if (afterpoint != 0)
    {
        res[i] = '.';  /* add dot */

       /*  Get the value of fraction part upto given no.
         of points after dot. The third parameter is needed
         to handle cases like 233.007 */
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
