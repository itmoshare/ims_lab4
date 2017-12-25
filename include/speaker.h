#ifndef __SPEAKER__H
#define __SPEAKER__H

#include "interrupt.h"
#include "max.h"

unsigned char speaker_on;

void T1_ISR( void ) __interrupt ( 2 );

void initialize_speaker() 
{
    SetVector( 0x201B, (void *)T1_ISR );
	TMOD|=0b00000010; 
	ET1=1;      
	TH1=-250;
}
void enable_speaker()
{
    speaker_on=0;
	TL1=-250;
    TR1=1;   
}
void disable_speaker()
{
    TR1=0;    
	speaker_on=0;
	write_max(ENA, 0b0100000);
}

void T1_ISR( void ) __interrupt ( 2 ){
	if( speaker_on ){
		write_max(ENA, 0b0111100);
	}else{
		write_max(ENA, 0b0100000);
	}
	
	speaker_on = !speaker_on;
}

#endif