#ifndef USB_H_YYMSUWJF
#define USB_H_YYMSUWJF

#define MSC_INACTIVE 0
#define MSC_ACTIVE 1


void usb_printf(const char * format, ...);
void usb_init();

void usb_set_msc_mode(unsigned char v);
unsigned char usb_get_msc_mode();
void usb_mount_msc();
void usb_unmount_msc();
void usb_statemachine();

#endif /* end of include guard: USB_H_YYMSUWJF */
