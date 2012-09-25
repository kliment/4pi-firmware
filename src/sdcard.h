#ifndef SDCARD_H_YIQ9IWI0
#define SDCARD_H_YIQ9IWI0



#define SD_MODE_CPU 0
#define SD_MODE_HOST 1

void sdcard_mount();
void sdcard_unmount();
void sdcard_handle_state();
void sdcard_set_mode(unsigned char mode);
unsigned char sdcard_ismounted();
unsigned char sdcard_carddetected();

#endif /* end of include guard: SDCARD_H_YIQ9IWI0 */
