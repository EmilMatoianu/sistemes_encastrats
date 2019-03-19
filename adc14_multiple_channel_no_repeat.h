/*
 * adc14_multiple_channel_no_repeat.h
 *
 *  Created on: 26 oct. 2017
 *      Author: toni
 */

#ifndef ADC14_MULTIPLE_CHANNEL_NO_REPEAT_H_
#define ADC14_MULTIPLE_CHANNEL_NO_REPEAT_H_

#define NUM_ADC_CHANNELS 3

#include <stdint.h>

void init_ADC(void); //ATTENTION!!! -- Uses interrupt ISR ADC14_IRQHandler

uint16_t *ADC_read(void);



#endif /* ADC14_MULTIPLE_CHANNEL_NO_REPEAT_H_ */
