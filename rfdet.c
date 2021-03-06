/* 
 * RF Detector for PIC12F1612
 *
 *      JA1YTS:Toshiba Amature Radio Station
 *      JK1MLY:Hidekazu Inaba
 *
 *  (C)2022 JA1YTS,JK1MLY All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation and/or 
 * other materials provided with the distribution.
*/

#include <stdio.h>
#include <stdint.h>
#include <xc.h>
#include <pic.h>

#define _XTAL_FREQ 8000000

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection
#pragma config PWRTE = OFF      // Power-up Timer
#pragma config MCLRE = OFF      // MCLR Pin Function Select
#pragma config CP = OFF         // Flash Program Memory Code Protection
#pragma config BOREN = ON       // Brown-out Reset Enable
#pragma config CLKOUTEN = OFF   // Clock Out Enable
// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection
#pragma config PLLEN = OFF      // 4x PLL OFF
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable
#pragma config BORV = LO        // Brown-out Reset Voltage Selection
#pragma config LPBOR = OFF      // Low Power Brown-out Reset Enable
#pragma config LVP = OFF        // Low-Voltage Programming Enable
// CONFIG3
#pragma config WDTE = OFF       // Watchdog Timer

/* ---------------------------------------------------------------------
 * PIC Pin configuration -
 * VSS:    GND                 (Pin 8)
 * RA0/AN0/DA1/CCP2/CWGB/PDAT  (Pin 7)
 * RA1/AN1              /PCLK  (Pin 6)
 * RA2/AN2    /CCP1/CWGA       (Pin 5)
 * RA3                  /MCLR  (Pin 4)
 * RA4/AN3         /CWGB/CLKOUT(Pin 3)
 * RA5        /CCP1/CWGA/CLKIN (Pin 2)
 * VDD:   +2.3-5.5V            (Pin 1)
 * -------------------------------------------------------------------*/

/* old version ATtiny13A <Incompatibility>
 * PB0/OC0A: LED-G output (Pin 5)
 * PB1/OC0B: LED-R output (Pin 6)
 * PB2/ADC1: DET   input  (Pin 7)
*/

#define G_LT LATA0=1 
#define G_DK LATA0=0  
#define R_LT LATA2=1 
#define R_DK LATA2=0
#define T_SW RA3
#define G_PW CCPR2L 
#define R_PW CCPR1L  

#define BLNK 400
#define CAL0 4
#define LMT0 24
#define OFS0 4

void port_init(void) {
    /* CONFIGURE GPIO */ 
    OSCCON  = 0b01110000;    //SPL IRCF3:0 0 SCS1:0 
                //IRCF 1111=16MHz IRCF 1110=8MHz IRCF 1101=4MHz
    TRISA   = 0b00011000;    //Input(1)
    OPTION_REG = 0b00000000; //MSB WPUENn
    WPUA    = 0b00001000;    //PupOn(1)
    INTCON  = 0b00000000;
    LATA    = 0b00000000;
    ANSELA  = 0b00010000;    //ANSA RA4;AN3=DET
	ADCON0  = 0b00001101;    //0 CHS4:0 GO ON AN3
	ADCON1  = 0b10000000;    //ADFM ADCS2:0 00 ADPREF1:0
                //ADCS 000=F/2 100=F/4 001=F/8 101=F/101
	ADCON2  = 0b00000000;
}

void pwm_init(void) {
    CCP1CON = 0b11001100;   // EN OE OUT FMT MODE3:0 11xx=PWM
    CCP2CON = 0b11001100;
    CCPTMRS = 0b00000000;   //0000 C2TSEL1:0 C1TSEL1:0 00=TMR2
    CCPR1L  = 0;
    CCPR1H  = 0;
    CCPR2L  = 0;
    CCPR2H  = 0;
    
    // Timer configuration
    T2CLKCON  = 0b00000000; //0000 CS3:0 000=FOSC/4 001=FOSC
    PR2     = 255;      // Period
    TMR2    = 0;        // Module
    T2CON   = 0b10000010;   // ON CKPS2:0 OUTPS3:0
}

uint8_t adc_read(void)
{
	uint16_t det;
	uint8_t ret;
    det = 0;
    GO_nDONE = 1;
    while(GO_nDONE) ;
    det = ADRESH;
    det = ( det << 8 ) | ADRESL;
    
    ret =  (uint8_t)(det >> 2);
    return (ret);    
}

uint8_t opamp_cal (void)
{
	uint8_t adc_cur;
	uint8_t adc_min;
	
	adc_min = adc_read();
	R_LT;
	__delay_ms(BLNK);

	adc_cur = adc_read();
	if (adc_cur < adc_min) {
		adc_min = adc_cur;
	}
	G_LT;
	__delay_ms(BLNK);

	adc_cur = adc_read();
	if (adc_cur < adc_min) {
		adc_min = adc_cur;
	}
	R_DK;
	__delay_ms(BLNK);

	adc_cur = adc_read();
	if (adc_cur < adc_min) {
		adc_min = adc_cur;
	}
	G_DK;
	__delay_ms(BLNK);

	adc_cur = adc_read();
	if (adc_cur < adc_min) {
		adc_min = adc_cur;
	}
	__delay_ms(BLNK);

	return(adc_min);
}

void pwm_write (uint8_t val)
{
	if(val > 160){
		val = 160;
	}
	if (val >= 64) {			//R only
			G_PW = 1;
			R_PW = (uint8_t)((val-64)*2 + 50);
	} else if (val >= 16) {		// R+G
			G_PW = 10;
			R_PW = (uint8_t)(val - 14);
	} else if (val >= 1 ) {		//G only
			G_PW = (uint8_t)(val + 2);
			R_PW = 1;
	} else {					// Off
		    G_PW = 1;
		    R_PW = 1;
	}
}
 
void main (void)
{
    uint8_t cal_ad = 0;
    uint8_t led_val;

// LED ports are output.
    port_init();
// OPAMP offset adjust
    cal_ad = opamp_cal();

//  cal_ad = 1;     // for TEST
// Abort maybe H/W broken
	if((cal_ad > LMT0) && (T_SW == 1)){
		R_DK;
		G_DK;
	    while (1) {
			R_LT;
			__delay_ms(BLNK);
			R_DK;
			__delay_ms(BLNK);
		}
	}
// Offset calibration
	if(cal_ad > CAL0){
        cal_ad = (uint8_t)(cal_ad - CAL0);
//        cal_ad = (uint8_t)(cal_ad / 4);
    }else{
        cal_ad = 0;
	}

// Enable PWM output
    pwm_init();
    led_val = (uint8_t)((cal_ad + 1) * 4);

// ADC CAL monitor
    pwm_write(led_val);
    __delay_ms(BLNK * 2);


    while (T_SW == 0) {
        if(led_val >= 160){
            led_val = 0;
        }else{
            led_val = (uint8_t)(led_val + 10);
        }
        pwm_write(led_val);
        __delay_ms(BLNK);
    }

    while (1) {
    // Get ADC value
        led_val = adc_read();
        led_val = (uint8_t)(led_val - cal_ad + OFS0);
    // Set PWM counter
        pwm_write(led_val);
//      __delay_ms(BLNK);
        __delay_ms(1);
    }
}
