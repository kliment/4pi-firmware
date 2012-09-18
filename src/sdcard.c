
#include <board.h>
#include <memories/MEDSdcard.h>
#include <fatfs/src/ff.h>
#include <stdio.h>

#include "sdcard.h"

#define MAX_LUNS            1
#define DRV_DISK            0

Media medias[MAX_LUNS];

static unsigned char is_mounted = 0;
static FATFS fs;


#define _ERR(x) #x
static const char* errorStrings[] = {
	_ERR(FR_OK),			/* 0 */
	_ERR(FR_NOT_READY),		/* 1 */
	_ERR(FR_NO_FILE),			/* 2 */
	_ERR(FR_NO_PATH),			/* 3 */
	_ERR(FR_INVALID_NAME),	/* 4 */
	_ERR(FR_INVALID_DRIVE),	/* 5 */
	_ERR(FR_DENIED),			/* 6 */
	_ERR(FR_EXIST),			/* 7 */
	_ERR(FR_RW_ERROR),		/* 8 */
	_ERR(FR_WRITE_PROTECTED),	/* 9 */
	_ERR(FR_NOT_ENABLED),		/* 10 */
	_ERR(FR_NO_FILESYSTEM),	/* 11 */
	_ERR(FR_INVALID_OBJECT),	/* 12 */
	_ERR(FR_MKFS_ABORTED)		/* 13 */
};
#undef _ERR

static const char* getError(FRESULT r)
{
	return errorStrings[r];
}

DWORD get_fattime()
{
	return 0;
}

void ISR_Media()
{
    MED_HandleAll(medias, 1);
}



void sdcard_mount()
{
	DIR dirs;
	FRESULT res;

	if (is_mounted)
		return;
	
	if (!MEDSdcard_Initialize(&medias[DRV_DISK],0))
	{
		printf("\r\nsdcard: SD card initialization failed, no sd card inserted?\r\n");
		return;
	}
	
	f_mount(0,&fs);
	
	if (f_opendir(&dirs,"0:") == FR_NO_FILESYSTEM)
	{
		printf("sdcard: No filesystem found on SD card, formatting...");
		res = f_mkfs(0,0,512);
		if (res != FR_OK)
		{
			printf("failed: %s\r\n",getError(res));
			return;
		}
		else
			printf("ok\r\n");
	}
	is_mounted = 1;
}

void sdcard_unmount()
{
	if (!is_mounted)
		return;
		
	f_mount(NULL,0);
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

