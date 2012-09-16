
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "serial.h"


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
    char _buf[128];
    strncpy(_buf,c,128);
    _buf[127]='\0';
/*    
    if(flag && isSerialConnected){
        //flag=0;
        printf(_buf);
        printf("BUFLEN:%d\r\n",strlen(_buf));
        if(CDCDSerialDriver_Write((void *)_buf,strlen(_buf), 0, 0)!= USBD_STATUS_SUCCESS)
            printf("FAIL\r\n");
        //CDCDSerialDriver_Write((void *)_buf,(strlen(_buf)%60), 0, 0);
    }
*/
}

void usb_printf (char * format, ...)
{
  char buffer[256];
  unsigned int str_len = 0;
  va_list args;
  va_start (args, format);
  str_len = vsprintf (buffer,format, args);
  
//  if(flag && isSerialConnected)
//		if(CDCDSerialDriver_Write((void *)buffer,str_len, 0, 0)!= USBD_STATUS_SUCCESS)
//			printf("USB FAIL\r\n");
  
  va_end (args);
}


void samserial_init()
{
}


