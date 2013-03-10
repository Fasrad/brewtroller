/*-------|---------|---------|---------|---------|---------|---------|---------|
brewtroller.c	

a simple temperature/boil controller for brewing or sous vide

It has 2 knobs, one temperature setpoint and one duty cycle %
With no temp probe connected, it acts as an open-loop 0-100% boil controller
With temp probe connected it acts as a simple heat-only thermostat.
% duty cycle adjustment is still applied even in temp control mode. 

HARDWARE:
for ATMEGAxx8 set at 1MHz. 
LM335 on PC0 (10mV/K absolute)
3-4V pot on PC1 (temperature setpoint)
0-5V pot on PC2 (duty cycle control)
AREF=5V
SSR and LED connected to PBwhatever

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 3 or any later
version. This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
******************************************************************/

#include <avr/io.h>

void delay(uint16_t);
void die (uint8_t);
void blink (uint8_t);
void adc_init(void);
void adc_read(uint8_t);

uint8_t PWM_duty=0;
uint8_t hyst = 2;      //hysteresis value in ADC counts
uint16_t probe_ad;  

int main(){
    DDRB = 0xFF;
    DDRC = 0;
    PORTC = 0;
    PORTB = 0;
    //8 bit Timer 0 is used by delay().
    TCCR0A = 0;                //stardard timer mode (page 103)
    TCCR0B = 2;                //fcpu / 1
    for (int i=0; i<8; i++){   //startup blinkenled for user
	PORTB |= (1<<5);
	delay(80);            
	PORTB &= ~(1<<5);
	delay(80);
    }
    delay(800);
    /****************************************
    *****main loop***************************
    ****************************************/
    blink(3); //countdown for user convenience
    while(1){  
	/*
	adc_read(0);probe_ad = ADCW;      //read temp probe
	if(probe_ad < 900){               //read setpoint knob if probe exists
	    adc_read(1);                
	    if(ADCW > (probe_ad + hyst)){         //thermostat
		write_pwm(1);
	    }else if(ADCW < (probe_ad -hyst){
		write_pwm(0);
	    }
	}else{                  //just apply duty cycle setting if no probe
	    write_pwm(1);
	}
	*/
	PWM_duty++;
	PORTB = 0xFF;
	delay(PWM_duty);            //200ms
	PORTB = 0;
	delay(255-PWM_duty);
    } //infty
}//main

void delay(uint16_t me){    //at 1MHz, each unit is 2.55us. 1ms is 4units. 
    while(me){
	while(TCNT0 < 128){}
	me--;
	while(TCNT0 > 128){}
    }
}
void die (uint8_t me){
    while(1){
	blink(me);
	delay(2000);
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
    ADCSRA |= (1<<ADEN); 
    ADCSRA |= (1<<ADSC);  
}
void adc_read(uint8_t me){
    ADMUX &= 0xF0;
    ADMUX |= me;
    ADCSRA |= (1<<ADSC); while(ADCSRA & (1<<ADSC)){}; 
}
void write_pwm(uint8_t me){
    if(me){
	adc_read(2);
	PWM_duty=ADCW>>2;
    }else{
	PWM_duty=0;
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
