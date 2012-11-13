#ifndef SERIAL_H_SZ1XK7A8
#define SERIAL_H_SZ1XK7A8

void samserial_setcallback(void (*c)(unsigned char));
void samserial_print(const char* c);
void samserial_init();
void samserial_datareceived(unsigned char c);

extern unsigned char isSerialConnected;

#endif /* end of include guard: SERIAL_H_SZ1XK7A8 */

