

#define MAX_CMD_SIZE 96
#define BUFSIZE 6 //8


unsigned char get_byte_from_UART(unsigned char *zeichen);

void usb_characterhandler(unsigned char c);
unsigned char get_byte_from_UART(unsigned char *zeichen);
void process_commands();
void get_command();


extern unsigned char buflen;
extern unsigned char bufindr;
extern unsigned char bufindw;

