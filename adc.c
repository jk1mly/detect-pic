//  
//      JA1YTS:Toshiba Amature Radio Station
//      JK1MLY:Hidekazu Inaba
//  

// Radio Wave Detector for ATtin13A
//  (C)2016-2018 JA1YTS,JK1MLY All rights reserved.
// Redistribution and use in source and binary forms, with or without modification, 
// are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, 
// this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, 
// this list of conditions and the following disclaimer in the documentation and/or 
// other materials provided with the distribution.

/* ---------------------------------------------------------------------
<<< Reference >>>
 * PWM LED Brightness control for ATtiny13.
 * Datasheet for ATtiny13: http://www.atmel.com/images/doc2535.pdf
 * 
 * Pin configuration -
 * PB0/OC0A: LED-G output (Pin 5)
 * PB1/OC0B: LED-R output (Pin 6)
 * PB2/ADC1: DET input (Pin 7)
 * PB3/ADC3: NC (Pin 2)
 * PB4/ADC2: NC (Pin 3)
 * PB5/ADC0/RST: NC (Pin 1)
 *
 * ~100 bytes.
 * 
 * Find out more: http://bit.ly/1eBhqHc
 * -------------------------------------------------------------------*/



// 9.6 MHz, built in resonator
#define F_CPU 1200000
#define LEDG PB0 
#define LEDR PB1
#define BLNK 400
#define CAL0 4
#define LMT0 32
#define OFS0 2

#include <avr/io.h>
#include <util/delay.h>

int adc_read (void)
{
	// Start the conversion
	ADCSRA |= (1 << ADSC);
	
	// Wait for it to finish
	while (ADCSRA & (1 << ADSC));

	return ADCH;
}

void adc_setup (void)
{
    // Set the ADC input to PB2/ADC0 (00)
    // Set the ADC input to PB2/ADC1 (01)
    ADMUX |= (1 << MUX0);
    // Set the ADC input to PB4/ADC2 (10)
    // ADMUX |= (1 << MUX1);
    // Set the ADC input to PB3/ADC3 (11)
    //ADMUX |= (1 << MUX1);
    //ADMUX |= (1 << MUX0);
    // Set the ADC left adjust result
    ADMUX |= (1 << ADLAR);

    // Set the prescaler to clock/64 & enable ADC
    // At 1.2 MHz this is 19 kHz.
    // See ATtiny13 datasheet, Table 14.4.
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADEN);

}

int opamp_cal (void)
{
	int adc_cur;
	int adc_min;
	
	adc_min = adc_read();
	PORTB = PORTB |  (1 << LEDR);
	_delay_ms(BLNK);

	adc_cur = adc_read();
	if (adc_cur < adc_min) {
		adc_min = adc_cur;
	}
	PORTB = PORTB |  (1 << LEDG);
	_delay_ms(BLNK);

	adc_cur = adc_read();
	if (adc_cur < adc_min) {
		adc_min = adc_cur;
	}
	PORTB = PORTB & ~(1 << LEDR);
	_delay_ms(BLNK);

	adc_cur = adc_read();
	if (adc_cur < adc_min) {
		adc_min = adc_cur;
	}
	PORTB = PORTB & ~(1 << LEDG);
	_delay_ms(BLNK);

	adc_cur = adc_read();
	if (adc_cur < adc_min) {
		adc_min = adc_cur;
	}
	_delay_ms(BLNK);

	return(adc_min);
}

void pwm_setup (void)
{
    // Set Timer 0 prescaler to clock/8.
    // At 1.2 MHz this is 150 kHz.
    // See ATtiny13 datasheet, Table 11.9.
    TCCR0B |= (1 << CS01);

    // Set to 'Fast PWM' mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);

    // Clear OC0A output on compare match, upwards counting.
    TCCR0A |= (1 << COM0A1);

    // Clear OC0B output on compare match, upwards counting.
    TCCR0A |= (1 << COM0B1);
}

void pwm_write (int val)
{
	if(val > 160){
		val = 160;
	}
	
	if (val >= 128) {			//128- R only
		    TCCR0A &= ~(1 << COM0A1);
			TCCR0A |=  (1 << COM0B1);
			OCR0A = 255;
			OCR0B = 255 - (96 -(val - 128)*3 );
	} else if (val >= 32) {		//32-127 R+G
		    TCCR0A |=  (1 << COM0A1);
			TCCR0A |=  (1 << COM0B1);
			OCR0A = 255 - (128 + val);
			OCR0B = 255 - (255 -(val - 32));
	} else if (val >= 1 ) {		//1-31 G only
		    TCCR0A |=  (1 << COM0A1);
			TCCR0A &= ~(1 << COM0B1);
			OCR0A = 255 - ((32 - val)*2 - 64);
			OCR0B = 255;
	} else {					//0 light Off
		    TCCR0A &= ~(1 << COM0A1);
		    TCCR0A &= ~(1 << COM0B1);
		    OCR0A = 255;
		    OCR0B = 255;
	}
}
 
int main (void)
{
    int adc_in;
    int cal_ad;

    // LED ports are output.
    DDRB |= (1 << LEDG);  
    DDRB |= (1 << LEDR);

	// OPAMP offset adjust
    adc_setup();
    cal_ad = opamp_cal();

	// Abort maybe H/W broken
//	if((cal_ad > LMT0) || (cal_ad <1)){
	if(cal_ad > LMT0){
		PORTB = PORTB & ~(1 << LEDR);
		PORTB = PORTB & ~(1 << LEDG);
	    while (1) {
			PORTB = PORTB |  (1 << LEDR);
			_delay_ms(BLNK);
			PORTB = PORTB & ~(1 << LEDR);
			_delay_ms(BLNK);
		}
	}
	// offset
	if(cal_ad > CAL0){
		cal_ad -= CAL0;
		}else{
		cal_ad = 0;
	}

	// PWM port setup
    pwm_setup();
	// ADC CAL monitor
    pwm_write((cal_ad + 1) * 4);
    _delay_ms(BLNK * 2);
	// Setup End
    pwm_write(0);
    _delay_ms(BLNK);

    while (1) {
        // Get ADC value
        adc_in = adc_read() - cal_ad + OFS0;
        // set PWM counter
        pwm_write(adc_in);
    }
}