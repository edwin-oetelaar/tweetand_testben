#ifndef ANALOG_INPUT_H_INCLUDED
#define ANALOG_INPUT_H_INCLUDED

#include <inttypes.h>
#include <stdlib.h>
#include "stm32f4xx_conf.h"
/* hier code voor de analoge keypad input */

void adc_configure(void); // IO en ADC setup, 8 bit ADC
uint16_t adc_convert(void); // lees waarde van de ADC
uint8_t val2key(uint16_t acdvalue); // waarde van ADC naar key, ADC in 8 bit mode


#endif /* ANALOG_INPUT_H_INCLUDED */
