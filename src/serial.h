
#ifndef SERIAL_H_A1QR1TYL
#define SERIAL_H_A1QR1TYL


void samserial_datareceived(const void* pBuffer,int size);
void samserial_setcallback(void (*c)(unsigned char));
void samserial_print(const char* c);
void samserial_init();


#endif /* end of include guard: SERIAL_H_A1QR1TYL */


