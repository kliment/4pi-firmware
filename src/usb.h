#ifndef USB_H_Y8TBR60V
#define USB_H_Y8TBR60V


void usb_init();
void usb_handle_state();
void usb_printf (const char * format, ...);

void usb_mount_sdcard();
void usb_unmount_sdcard();



#endif /* end of include guard: USB_H_Y8TBR60V */
