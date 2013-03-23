/*-------|---------|---------|---------|---------|---------|---------|---------|
brewtroller.c	

a simple temperature/boil controller for brewing or sous vide

by chaz miller 

With no temp probe, it acts as an open-loop 0-100% boil controller.
If installed, boil override switch allows fixed boil power to be set. 
Temp probe presence is auto-detected.
With temp probe connected it acts as a simple heat-only thermostat.
The duty cycle knob setting is still applied even in temp control mode. 

HARDWARE:
for ATMEGAxx8 set at 1MHz running at 5V. 
SSR connected to PB2 (OC1B) for heating element
2 SPST switch connected from PB4 and PB5 to ground (optional, simplest)
PB3 = boil
PB4 = OFF
PB5 = contactor
Pot on PC2 (duty cycle control knob, optional)
LED connected to PB5 (recommended)
NO contactor connected to PB6 (optional)
LM335 diode temp sensor + 10kish resistor on PC0 (10mV/K absolute) (optional)
Pot on PC3 (temp setpoint knob) (needed if temp sensor is used)

For a bit more real precision divide down Vcc and run the pots and AREF at 4V.
Bracket temp setpoint knob with resistors to taste.

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 3 or any later
version. This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
******************************************************************/
//todo: add GFCI?

#include <avr/io.h>

#define boil_setpoint 0x980    //adjust to your system
#define hyst 2    //about 1/10th deg. C per count at 12 bits; optional

void delay(uint16_t);
void blink (uint8_t);
void adc_init(void);
uint16_t adc_read(uint8_t);

uint16_t probe_ad;  
uint16_t set_ad;  

int main(){
    //8 bit Timer 0 is used by delay().
    TCCR0A = 0;                //standard timer mode (page 103)
    TCCR0B = 2;                //fcpu / 1
    //16-bit Timer 1 used as output PWM on OC1B PB2 (Arduino pin 10) p.115
    //noninverting phase correct, CTC-PWM hybrid mode p135 
    TCCR1A = (1<<COM1B1)|(1<<WGM11)|(1<<WGM10); 
    TCCR1B = (1<<WGM13)|(1<<CS12);                 //  clk/256
    //TCCR1B = (1<<WGM13)|(1<<CS11)|(1<<CS10);       //  clk/64
    //TCCR1B = (1<<WGM13)|(1<<CS10);                 //  clk/1
    OCR1A = 0x0FF0;             //sets pwm TOP 
    OCR1B = 0;
    PORTB = 0xFF;
    DDRB = 0b000100100;        //LED on PB5; OC1B is PB2
    adc_init();
    /****************************************
    *****main loop***************************
    ****************************************/
    blink(3);
    for(;;){  
	if(PINB&1<<3){                      //if switch is not 'WFO'
	    PINB|=1<<5;                     //turn on contactor
	    if(PINB&1<<4){                  //if switch is not 'boil'
		probe_ad = adc_read(0); 
		if(probe_ad < 0x0EFF){       //continue iff probe detected 
		    set_ad = adc_read(1);                
		    if(set_ad > (probe_ad + hyst)){   //thermostat block
			OCR1B = adc_read(2) & 0xFF00;
		    }else if(set_ad < (probe_ad - hyst)){
			OCR1B = 0x0000;
		    }
		}else{                       //no probe; no boil override
		    OCR1B = adc_read(2) & 0xFF00;
		}
	    }else{                           //'boil' switch is on
		OCR1B = boil_setpoint;
	    }
	}else{
	    OCR1B = OCR1A;                 //WFO switch is on
	}
   } //infty
}//main
void adc_init(void){
    ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); //125kHz @ 1MHz clock, page 264
    ADMUX |= (1<<REFS0);           //Avcc (5v)
    //ADMUX |= (1<<ADLAR);           //left align for 8-bit operation
    ADCSRA |= (1<<ADEN); 
    //ADCSRA |= (1<<ADSC);  
}
uint16_t adc_read(uint8_t me){    //expects register value, not port pin label
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
	delay(200);
	PORTB &= ~(1<<5);
	delay(200);
    }
    delay(500);
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
