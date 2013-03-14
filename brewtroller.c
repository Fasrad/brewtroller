/*-------|---------|---------|---------|---------|---------|---------|---------|
brewtroller.c	

a simple temperature/boil controller for brewing or sous vide

by chaz miller 2013

Controller has 2 knobs, temperature setpoint and duty cycle.
With no temp probe, it acts as an open-loop 0-100% boil controller.
With temp probe connected it acts as a simple heat-only thermostat.
The duty cycle adjustment is still applied even in temp control mode. 

HARDWARE:
for ATMEGAxx8 set at 1MHz running at 5V. 
LM335 diode temp sensor on PC0 (10mV/K absolute)
Pot on PC1 (temperature setpoint knob)
Pot on PC2 (duty cycle control knob)
SSR connected to PB3 (OC2A)
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

uint8_t hyst = 0;      //hysteresis value in ADC counts
uint16_t probe_ad;  
uint16_t set_ad;  

int main(){
    DDRB = 0xFF;
    DDRC = 0;
    PORTC = 0;
    PORTB = 0;
    //8 bit Timer 0 is used by delay().
    TCCR0A = 0;                //stardard timer mode (page 103)
    TCCR0B = 2;                //fcpu / 1
    //8 bit Timer 2 used as output PWM on OC2A PB3 (Arduino pin 11)
    TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20); //CPU/1024; 2Hz
    TCCR2A = (1<<WGM20); TCCR2B = (1<<WGM22); //phase correct TOP=OCR2A mode 
    TCCR2A = (1<<COM2A1); //turn on pin, noninverting, when slow pwm
    adc_init();
    /****************************************
    *****main loop***************************
    ****************************************/
    blink(3); 
    for(;;){  
	probe_ad = adc_read(0);        //read temp probe
	if(probe_ad < 1000){            //read setpoint knob iff probe exists
	    //probe_ad = probe_ad-612;  //map 3-3.75V to 0-5V 
	    //probe_ad = probe_ad<<3;   //to 'span' septoint knob
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
   } //infty
}//main
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
void adc_init(void){
    ADCSRA = (1<<ADEN)|(1<<ADPS2); //62kHz @ 1MHz clock
    ADMUX |= (1<<REFS0);           //Avcc (5v)
    //ADMUX |= (1<<ADLAR);           //left align for 8-bit operation
    ADCSRA |= (1<<ADEN); 
    ADCSRA |= (1<<ADSC);  
}
uint16_t adc_read(uint8_t me){
    uint16_t ad_bucket=0;
    ADMUX &= 0xF0;
    ADMUX |= me;
    for (int i=0; i<8; i++){
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

/*Set a bit
 bit_fld |= (1 << n)

Clear a bit
 bit_fld &= ~(1 << n)

Toggle a bit
 bit_fld ^= (1 << n)

Test a bit
 bit_fld & (1 << n)
*/ 
