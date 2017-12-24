#ifndef __SPEAKER__H
#define __SPEAKER__H

#include "interrupt.h"
#include "max.h"

bool speaker_on;

void T0_ISR( void ) __interrupt ( 2 );

void initialize_speaker() 
{
    SetVector( 0x200B, (void *)T0_ISR );
	TMOD|=0b00000010; 
	ET0=1;      
	TH0=-250;
}
void enable_speaker()
{
    speaker_on=0;
	TL0=-250;
    TR0=1;   
}
void disable_speaker()
{
    TR0=0;    
	speaker_on=0;
	write_max(ENA, 0b0100000);
}

void T0_ISR( void ) __interrupt ( 2 ){
	if( speaker_on ){
		write_max(ENA, 0b0111100);
	}else{
		write_max(ENA, 0b0100000);
	}
	
	speaker_on = !speaker_on;
}

#endif