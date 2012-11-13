

#include "util.h"


extern volatile unsigned long timestamp;

void delay_ms(unsigned long msec)
{
	unsigned long curms = timestamp+msec;
	
	//TODO: handle overflow
	while(timestamp < curms) { __asm volatile("nop"); }
	
}