/*
 * acelerometer_driver.c
 *
 *  Created on: 26 oct. 2017
 *      Author: toni
 */

// Includes standard
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "accelerometer_driver.h"
#include "adc14_multiple_channel_no_repeat.h"

void init_Accel(void){
    init_ADC();
}

void Accel_read(float *values){
    uint16_t *Data;
    uint8_t i;

    //pedir datos ADC
    Data = ADC_read();

    //realizar conversion
    for (i=0;i<NUM_ADC_CHANNELS;i++){
        values[i] = ((float)Data[i]-CONVERSION_OFFSET)/CONVERSION_SCALE;
    }

}


