/*-------|---------|---------|---------|---------|---------|---------|---------|
brewtroller.c	

a simple PWM generator/thermostat/boil controller for brewing or sous vide
refer to brewtroller.pdf for details

by chaz miller for ATMEGAxx8 set at 1MHz running at 5V. 

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

int main(){
    uint16_t probe_ad;  
    uint16_t set_ad;  
    uint16_t dly;         //compressor delay
    uint16_t dlycnt;      //comp. delay counter
    //16-bit Timer 1 used as output PWM on OC1B PB2 (Arduino pin 10) p.115
    //noninverting phase correct, CTC-PWM hybrid mode p135 
    TCCR1A = (1<<COM1B1)|(1<<WGM11)|(1<<WGM10); 
    OCR1A = 0x0FF0;             //sets pwm TOP 
    OCR1B = 0;
    switch ((PINB&0xC0)>>6){
	case 0:                                            //  PB6=PB7=gnd
	    TCCR1B = (1<<WGM13)|(1<<CS10);                 //  clk/1; 240kHz
	    CLKPR = 0x80; CLKPR = 0;                       //  clk*8; 2kHz
	    break;
	case 1:                                            //  PB6 = gnd
	    TCCR1B = (1<<WGM13)|(1<<CS10);                 //  clk/1; 240kHz
	    break;
	case 2:                                            //  PB7 = gnd
	    TCCR1B = (1<<WGM13)|(1<<CS11)|(1<<CS10);       //  clk/64; 4Hz
	    break;
	case 3:
	    TCCR1B = (1<<WGM13)|(1<<CS12);                 //  clk/256; 1Hz
    }
    //8 bit Timer 2 for compressor delay; 1 may work?
    TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);                //CPU/1024; 2Hz
    PORTB = 0xFF;
    PORTB|=1<<5;                     //turn on LED
    DDRB = 0b000100100;        //LED on PB5; OC1B is PB2
    adc_init();
    if (PINB&1<<0){dly=0x200;}   //enable compressor delay; 2 cnts=1sec
    /****************************************
    *****main loop***************************
    ****************************************/
    for(;;){  
	if(TIFR2&1<<TOV2){dlycnt++;TIFR2|=1;} //increment compressor delay
	if(PINB&1<<3){                      //if switch is not 'WFO'
	    if(PINB&1<<4){                  //if switch is not 'boil'
		probe_ad = adc_read(0);     //read probe
		if(probe_ad < 0x0EFF){       //continue iff probe detected 
		    set_ad = adc_read(1);    //be thermostat 
		    if((set_ad>(probe_ad+hyst))&&(dlycnt>dly)){  
			OCR1B = adc_read(2) & 0xFF00;
			dlycnt=0;
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
    //ADCSRA = (1<<ADEN)|(1<<ADPS1)|(1<<ADPS0); //125kHz @ 1MHz clock, page 264
    ADCSRA = (1<<ADEN)|(1<<ADPS2); //62kHz @ 1MHz clock, page 264
    ADMUX |= (1<<REFS0);           //AREF=Avcc (5v)
    //ADMUX |= (1<<ADLAR);           //left align for 8-bit operation
    ADCSRA |= (1<<ADEN); 
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
/*
    //8 bit Timer 0 is used by delay().
    TCCR0A = 0;                //standard timer mode (page 103)
    TCCR0B = 2;                //fcpu / 1
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
Set a bit
 bit_fld |= (1 << n)

Clear a bit
 bit_fld &= ~(1 << n)

Toggle a bit
 bit_fld ^= (1 << n)

Test a bit
 bit_fld & (1 << n)
*/ 
