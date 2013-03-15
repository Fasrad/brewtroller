/*-------|---------|---------|---------|---------|---------|---------|---------|
brewtroller.c	

a simple temperature/boil controller for brewing or sous vide

by chaz miller 

Controller has 2 knobs, temperature setpoint and duty cycle.
With no temp probe, it acts as an open-loop 0-100% boil controller.
With temp probe connected it acts as a simple heat-only thermostat.
The duty cycle adjustment is still applied even in temp control mode. 

HARDWARE:
for ATMEGAxx8 set at 1MHz running at 5V. 
LM335 diode temp sensor on PC0 (10mV/K absolute)
Pot on PC1 (temperature setpoint knob)
Pot on PC2 (duty cycle control knob)
SSR connected to PB3 (OC2A) for heating element
LED connected to PB5 (optional)
For a bit more precision divide down Vcc and run the pots and AREF at 4V.
Bracket temp setpoint knob with resistors to taste.

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 3 or any later
version. This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
******************************************************************/

#include <avr/io.h>

void delay(uint16_t);
void blink (uint8_t);
void adc_init(void);
uint16_t adc_read(uint8_t);
void write_pwm(uint8_t);

uint8_t hyst = 2;    //about 1/10th deg. C per count at 12 bits 
uint16_t probe_ad;  
uint16_t set_ad;  

int main(){
    DDRB = 0xFF;
    DDRD = 0xFF;
    //8 bit Timer 0 is used by delay().
    TCCR0A = 0;                //stardard timer mode (page 103)
    TCCR0B = 2;                //fcpu / 1
    //16-bit Timer 1 used as output PWM on OC1B PB2 (Arduino pin 10) p.115
    //noninverting phase correct, OCR1A=TOP mode channel B p135 
    TCCR1A = (1<<COM1B1)|(1<<WGM11)|(1<<WGM10); TCCR1B = (1<<WGM13);
    TCCR1B = (1<<CS12);        //  clk/256
    OCR1A = 0x1000;            //set TOP=12 bits
    OCR1B = 0x800;
    adc_init();
    /****************************************
    *****main loop***************************
    ****************************************/
    blink(3); 
    for(;;){  
	probe_ad = adc_read(0); 
	blink(probe_ad>>10);
	if(probe_ad < 3500){            //read setpoint knob iff probe exists
	    set_ad = adc_read(1);                
	    if(set_ad > (probe_ad + hyst)){   //thermostat block
		write_pwm(1);
	    }else if(set_ad < (probe_ad-hyst)){
		write_pwm(0);
	    }
	}else{                //just apply duty cycle setting if no probe
	    write_pwm(1);
	}
	blink(1);
	PORTB=4;
	delay(1000);
	PORTB=0;
	delay(1000);
   } //infty
}//main
void adc_init(void){
    ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); //125Hz @ 1MHz clock, page 264
    ADMUX |= (1<<REFS0);           //Avcc (5v)
    //ADMUX |= (1<<ADLAR);           //left align for 8-bit operation
    ADCSRA |= (1<<ADEN); 
    ADCSRA |= (1<<ADSC);  
}
uint16_t adc_read(uint8_t me){
    uint16_t ad_bucket=0;
    ADMUX &= 0xF0;
    ADMUX |= me;
    for (int i=0; i<16; i++){
	ADCSRA |= (1<<ADSC); 
	while(ADCSRA & (1<<ADSC)); 
	ad_bucket += ADCW;
    }
    return (ad_bucket>>2); //12 bits oversampled
}
void write_pwm(uint8_t me){
    if(me){
	adc_read(2);
	OCR2A = (ADCW>>4);
    }else{
	OCR2A=0;
    }
}
void delay(uint16_t me){    //at 1MHz, each unit is 2.55us. 1ms is 4units. 
    while(me){
	while(TCNT0 < 128){}
	me--;
	while(TCNT0 > 128){}
    }
}
void blink (uint8_t me){
    for (int i=0; i<me; i++){
	PORTB |= (1<<5);
	delay(300);
	PORTB &= ~(1<<5);
	delay(300);
    }
    delay(600);
}
/*Set a bit
 bit_fld |= (1 << n)

Clear a bit
 bit_fld &= ~(1 << n)

Toggle a bit
 bit_fld ^= (1 << n)

Test a bit
 bit_fld & (1 << n)
*/ 
