#ifndef SDCARD_H_YIQ9IWI0
#define SDCARD_H_YIQ9IWI0


void sdcard_listfiles();
void sdcard_selectfile(const char* name);
void sdcard_capturestart();
void sdcard_capturestop();
unsigned char sdcard_iscapturing();
unsigned char sdcard_writeline(const char* line);
void sdcard_setposition(unsigned int filepos);
void sdcard_printstatus();
int sdcard_getchar(unsigned char* chr);
void sdcard_replaystart();
void sdcard_replaypause();
void sdcard_replaystop();
int sdcard_isreplaying();
int sdcard_isreplaypaused();
void sdcard_handle_state();
void sdcard_mount();
void sdcard_unmount();
unsigned char sdcard_ismounted();
unsigned char sdcard_carddetected();

#endif /* end of include guard: SDCARD_H_YIQ9IWI0 */
