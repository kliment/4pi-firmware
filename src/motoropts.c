#include <board.h>
#include <pio/pio.h>
#include <stdio.h>
#include "init_configuration.h"
#include "parameters.h"



const Pin MOSI={1 <<  14, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin SCK={1 <<  15, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin CS={1 <<  16, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_PULLUP};

const Pin XMS1={1 <<  30, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin XMS2={1 <<  29, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin XEN={1 <<  31, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin XSTEP={1 <<  28, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin XDIR={1 <<  8, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin YMS1={1 <<  11, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin YMS2={1 <<  10, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin YEN={1 <<  22, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin YSTEP={1 <<  23, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin YDIR={1 <<  31, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin ZMS1={1 <<  6, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin ZMS2={1 <<  5, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin ZEN={1 <<  7, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin ZSTEP={1 <<  27, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin ZDIR={1 <<  27, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E0MS1={1 <<  12, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E0MS2={1 <<  11, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E0EN={1 <<  13, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E0STEP={1 <<  26, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E0DIR={1 <<  2, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E1MS1={1 <<  0, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E1MS2={1 <<  24, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E1EN={1 <<  13, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E1STEP={1 <<  1, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_PULLUP};
const Pin E1DIR={1 <<  25, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_PULLUP};


void AD5206_sendbit(unsigned char bit){
    volatile unsigned int uDummy;
    for (uDummy=0; uDummy<512; ++uDummy);
    PIO_Clear(&SCK);
    if(bit)
        PIO_Set(&MOSI);
    else
        PIO_Clear(&MOSI);
    for (uDummy=0; uDummy<512; ++uDummy);
    PIO_Set(&SCK);
}
    
void AD5206_setchan(unsigned char chan, unsigned char value){
    
    PIO_Set(&CS);
    volatile unsigned int uDummy;
    for (uDummy=0; uDummy<10000; ++uDummy);
    PIO_Clear(&SCK);
    PIO_Set(&MOSI);
    int i;
    PIO_Clear(&CS); //Enable chip
    for(i=0;i<3;i++){
        AD5206_sendbit((chan&(1<<(2-i)))>0);
    }
    for(i=0;i<8;i++){
        AD5206_sendbit((value&(1<<(7-i)))>0);
    }
    PIO_Clear(&SCK);
    PIO_Set(&CS);

}

void AD5206_setup(){
    Pin SPIPINS[]={MOSI,SCK,CS};

    PIO_Configure(SPIPINS,3);
    
    PIO_Set(&CS);
    
}


//convert digipot count to mA
unsigned int count_ma(unsigned char count)
{
	unsigned int ma = count*743/100;
	return ma;
}

//convert mA to digipot count
unsigned char ma_count(unsigned int ma)
{
	unsigned int count = ma*100/743;
	return (unsigned char)count;
}


unsigned char microstep_mode(unsigned char usteps){
	unsigned char mode;

	switch(usteps){
		case 1:
			mode=0;
			break;
		case 2:
			mode=1;
			break;
		case 4:
			mode=2;
			break;
		case 16:
			mode=3;
			break;
		default:
			mode=3;
			break;
	}
	return(mode);
}

//convert microstep mode to usteps
unsigned char microstep_usteps(unsigned char mode)
{
	const unsigned char usteps[] = { 1,2,4,16 };
	return usteps[mode];
}


	
void motor_setopts(unsigned char axis, unsigned char ustepbits, unsigned char current){
    Pin MS1;
    Pin MS2;
    
    unsigned char channel;
    switch(axis){
        case 0:
            MS1=XMS1;
            MS2=XMS2;
            channel=3;
            break;
        case 1:
            MS1=YMS1;
            MS2=YMS2;
            channel=1;
            break;
        case 2:
            MS1=ZMS1;
            MS2=ZMS2;
            channel=0;
            break;
        case 3:
            MS1=E0MS1;
            MS2=E0MS2;
            channel=2;
            break;
        case 4:
            MS1=E1MS1;
            MS2=E1MS2;
            channel=5;
            break;
        case 6:
            ustepbits=4;
            channel=4;
            break;
        default:
            return;
    }
    if(ustepbits<4){
        if(ustepbits&1)
            PIO_Set(&MS1);
        else
            PIO_Clear(&MS1);
        if(ustepbits&2)
            PIO_Set(&MS2);
        else
            PIO_Clear(&MS2);
    }
    AD5206_setchan(channel,current);

	//printf("Setting channel %u to current value %u and ustep value %u\r\n",channel, current, ustepbits);
	
}

void motor_setup(){
    Pin MOTPINS[]={XMS1,XMS2,XEN,XSTEP,XDIR,YMS1,YMS2,YEN,YSTEP,YDIR,ZMS1,ZMS2,ZEN,ZSTEP,ZDIR,E0MS1,E0MS2,E0EN,E0STEP,E0DIR,E1MS1,E1MS2,E1EN,E1STEP,E1DIR,};
    PIO_Configure(MOTPINS,25);
    PIO_Set(&XEN);
    PIO_Set(&YEN);
    PIO_Set(&ZEN);
    PIO_Set(&E0EN);
    PIO_Set(&E1EN);
    AD5206_setup();
    int i;
    for(i=0;i<5;i++)
        motor_setopts(i,pa.axis_ustep[i],pa.axis_current[i]);
    printf("done setting up motors\r\n\n");
}

void motor_enaxis(unsigned char axis, unsigned char en){
    switch(axis){
        case(0):
            if(!en)
                PIO_Set(&XEN);
            else
                PIO_Clear(&XEN);
            break;
        case(1):
            if(!en)
                PIO_Set(&YEN);
            else
                PIO_Clear(&YEN);
            break;
        case(2):
            if(!en)
                PIO_Set(&ZEN);
            else
                PIO_Clear(&ZEN);
            break;
        case(3):
            if(!en)
                PIO_Set(&E0EN);
            else
                PIO_Clear(&E0EN);
            break;
        case(4):
            if(!en)
                PIO_Set(&E1EN);
            else
                PIO_Clear(&E1EN);
            break;
    }
}

void motor_setdir(unsigned char axis, unsigned char dir){
    switch(axis){
        case(0):
            if(dir)
                PIO_Set(&XDIR);
            else
                PIO_Clear(&XDIR);
            break;
        case(1):
            if(dir)
                PIO_Set(&YDIR);
            else
                PIO_Clear(&YDIR);
            break;
        case(2):
            if(dir)
                PIO_Set(&ZDIR);
            else
                PIO_Clear(&ZDIR);
            break;
        case(3):
            if(dir)
                PIO_Set(&E0DIR);
            else
                PIO_Clear(&E0DIR);
            break;
        case(4):
            if(dir)
                PIO_Set(&E1DIR);
            else
                PIO_Clear(&E1DIR);
            break;
    }
}

__attribute__((always_inline)) void motor_step(unsigned char axis){
    switch(axis){
        case(0):
            PIO_Set(&XSTEP);
            break;
        case(1):
            PIO_Set(&YSTEP);
            break;
        case(2):
            PIO_Set(&ZSTEP);
            break;
        case(3):
            PIO_Set(&E0STEP);
            break;
        case(4):
            PIO_Set(&E1STEP);
            break;
    }
}
__attribute__((always_inline)) void motor_unstep(){
    PIO_Clear(&XSTEP);
    PIO_Clear(&YSTEP);
    PIO_Clear(&ZSTEP);
    PIO_Clear(&E0STEP);
    PIO_Clear(&E1STEP);
}
