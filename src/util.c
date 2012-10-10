

#include "util.h"


extern volatile unsigned long timestamp;

void delay_ms(unsigned long msec)
{
	unsigned long curms = timestamp;
	unsigned char overflow = curms+msec < curms ? 1 : 0;
	
	//TODO: handle overflow
	while(timestamp < curms) { __asm volatile("nop"); }
	
	
}