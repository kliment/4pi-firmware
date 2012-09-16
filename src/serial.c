
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "serial.h"
#include "usb.h"

static void (*callback)(unsigned char)=0;

void samserial_setcallback(void (*c)(unsigned char)){
	callback=c;
}

void samserial_datareceived(const void* pBuffer,int size)
{
	const char* pDataPtr = (const char*)pBuffer;

	if (callback)
	{
		while(size--)
			callback(*pDataPtr++);
	}
}

//volatile int busyflag=0;
//volatile char _samserial_buffer[128];
void samserial_print(const char* c)
{
	usb_printf(c);
}


void samserial_init()
{
}


