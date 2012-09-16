
#include <board.h>
#include <memories/MEDSdcard.h>

#include "sdcard.h"
#include <fatfs/src/ff.h>

#define MAX_LUNS            1
#define DRV_DISK            0

Media medias[MAX_LUNS];

static unsigned char is_mounted = 0;
static FATFS fs;

void ISR_Media()
{
    MED_HandleAll(medias, 1);
}


void sdcard_mount()
{
	DIR dirs;
	
	MEDSdcard_Initialize(&medias[DRV_DISK],0);
	f_mount(&fs,0);
	if (f_opendir(&dirs,"0:") == FR_NO_FILESYSTEM)
		f_mkfs(0,0,512);
	
	is_mounted = 1;
}

void sdcard_unmount()
{
	
	is_mounted = 0;
}

unsigned char sdcard_ismounted()
{
	return is_mounted;
}

unsigned char sdcard_carddetected()
{
	return MEDSdcard_Detect(&medias[DRV_DISK],0);
}

