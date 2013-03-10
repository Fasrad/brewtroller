/*-------|---------|---------|---------|---------|---------|---------|---------|
brewtroller.c	

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

int main(){
    DDRB = 0xFF;
    DDRC = 0;
    PORTC = 0xFF;
    PORTB = 0;
    //8 bit Timer 0 is used by delay().
    TCCR0A = 0;                //stardard timer mode (page 103)
    TCCR0B = 2;                //fcpu / 1
    TCCR1A = 0;                //16 bit Timer 1: main program timer 
    for (int i=0; i<8; i++){   //startup blinkenled for user
	PORTB |= (1<<5);
	delay(800);            //200ms
	PORTB &= ~(1<<5);
	delay(800);
    }
    delay(8000);
    //read in DIPswitch; set up TIMER1 per Allegro datasheet page 6
    /*switch (PINC & 0b00000111){  
	case 0:
	    die (9);
	    break;
	case 1:
	    die (1);
	    break;
	case 2:
	    die (2);
	    break;
	case 3:
	    die (3);
	    break;
	case 4:
	    die (4);
	    break;
	case 5:
	    die (5);
	    break;
	case 6:
	    die (6);
	    break;
	case 7:
	    break;
	    die (1);
	default:
	    die (1);
    } */
    /****************************************
    *****this is where the magic happens*****
    ****************************************/
    blink(3); //countdown for user convenience
    uint8_t duty=0;
    while(1){  
	duty++;
	PORTB |= (1<<5);
	delay(duty);            //200ms
	PORTB &= ~(1<<5);
	delay(255-duty);
    }
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
	delay(20000);
    }
}

void blink (uint8_t me){
    for (int i=0; i<me; i++){
	PORTB |= (1<<5);
	delay(3000);
	PORTB &= ~(1<<5);
	delay(3000);
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
