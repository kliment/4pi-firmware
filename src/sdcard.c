
#include <board.h>
#include <memories/MEDSdcard.h>
#include <fatfs/src/ff.h>
#include <stdio.h>
#include <string.h>
#include "sdcard.h"
#include "serial.h"

#define MAX_LUNS            1
#define DRV_DISK            0

Media medias[MAX_LUNS];

static unsigned char is_mounted = 0;
static FATFS fs;

static unsigned char had_card = 0;

static unsigned char capture_mode = 0;
static char selectedfileBuffer[_MAX_LFN];
static const char* selectedFile = NULL;
static FIL captureFile;
static FIL replayFile;
static int fileSeekpos = 0;
static unsigned char replay_mode = 0;
static unsigned char replay_pause = 0;

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

void sdcard_selectfile(const char* name)
{
	strcpy(selectedfileBuffer,name);
	selectedFile = selectedfileBuffer;
	printf("sdcard_selectfile: selected file %s\n\r",selectedFile);
	usb_printf("File selected: %s\n\r",selectedFile);
}

unsigned char sdcard_iscapturing()
{
	return capture_mode;
}

int sdcard_getchar(unsigned char* chr)
{
	FRESULT res;
	UINT read;

	if (!replay_mode && !replay_pause)
		return 0;
	
	res = f_read(&replayFile,chr,1,&read);
	if (res != FR_OK)
	{
		printf("sdcard_getchar: error %s\n\r",getError(res));
		return 0;
	}

	if (read != 1)
	{
		printf("sdcard_getchar: end of file\n\r");
		return 0;
	}	

	return 1;
}

void sdcard_replaystart()
{
	if (replay_mode)
		return;

	if (capture_mode)
		sdcard_capturestop();

	if (!replay_pause)
	{
		if (!selectedFile)
		{
			usb_printf("error: file not selected\r\n");
			return;
		}

		printf("sdcard_replaystart: opening file %s for replay\n\r",selectedFile);
		FRESULT res = f_open(&replayFile,selectedFile,FA_OPEN_EXISTING|FA_READ);
		if (res != FR_OK)
		{
			printf("sdcard_replaystart: error %s\n\r",getError(res));
			usb_printf("error: file.open failed\n\r");
			return;
		}
		usb_printf("File opened: %s \n\rok\n\r",selectedFile);
	}
	replay_mode = 1;
	replay_pause = 0;
	if(fileSeekpos)
        sdcard_setposition(fileSeekpos);
	fileSeekpos = 0;
}

void sdcard_replaypause()
{
	if (!replay_mode)
		return;

	printf("sdcard_replaypause\n\r");
	replay_pause = 1;
}

void sdcard_replaystop()
{
	if (!replay_mode)
		return;
		
	f_close(&replayFile);
	replay_mode = 0;
}

int sdcard_isreplaying()
{
	return replay_mode;
}

int sdcard_isreplaypaused()
{
	return replay_pause;
}

void sdcard_capturestart(const char* name)
{
	strcpy(selectedfileBuffer,name);
	selectedFile = selectedfileBuffer;
	printf("sdcard_capturestart: selected file %s\n\r",selectedFile);
	usb_printf("Writing to file: %s\n\r",selectedFile);
	if (!selectedFile)
	{
		usb_printf("error: file not selected\r\n");
		return;
	}
	
	if (capture_mode)
		sdcard_capturestop();
		
	printf("sdcard_capturestart: opening file %s for capture\n\r",selectedFile);
	FRESULT res = f_open(&captureFile,selectedFile,FA_CREATE_ALWAYS|FA_WRITE|FA_READ);

	if (res != FR_OK)
	{
		printf("sdcard_capturestart: failed to open file, error: %s\n\r",getError(res));
		usb_printf("error: failed to open file\r\n");
		return;
	}

	capture_mode = 1;
}

void sdcard_setposition(unsigned int filepos)
{
	if (replay_mode)
		f_lseek(&replayFile,filepos);
	else
		fileSeekpos = filepos;
}


void sdcard_capturestop()
{
	printf("sdcard_capturestop\n\r");
	
	if (!capture_mode)
	{
		usb_printf("error: capture not started\r\n");
		return;
	}
	
	usb_printf("%d bytes written\n\r",f_tell(&captureFile));
	f_sync(&captureFile);
	f_close(&captureFile);
	capture_mode = 0;
}

unsigned char sdcard_writeline(const char* line)
{
	FRESULT res;
	UINT written;
	
	if (!capture_mode)
		return 0;
		
	
	res = f_write(&captureFile,line,strlen(line),&written);
	if (res != FR_OK)
	{
		printf("sdcard_writeline error %s\n\r",getError(res));
		return 0;
	}

	if (strlen(line) != written)
	{
		printf("sdcard_writeline error: disk full?\n\r");
		return 0;
	}
	f_write(&captureFile,"\n",1,&written);
	
	return 1;
}




void sdcard_handle_state()
{
	unsigned char has_card = sdcard_carddetected();

	if (has_card != had_card)
	{
		if (has_card)
		{
			printf("sdcard: card inserted\n\r");
			sdcard_mount();
		}
		else
		{
			printf("sdcard: card removed\n\r");
			sdcard_unmount();
			is_mounted = 0;
		}
	}
	had_card = has_card;
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
		
	f_mount(0,NULL);
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

void sdcard_printstatus()
{
	if (!replay_mode)
	{
		usb_printf("ok not printing\n\r");
	}
	else
	{
		usb_printf("ok %02d%% (%d/%d)\n\r",(int)(f_tell(&replayFile)/(double)f_size(&replayFile)),f_tell(&replayFile),f_size(&replayFile));
	}
}

void sdcard_listfiles()
{
	DIR rootDir;
	FILINFO fileInfo;
	FRESULT res;
	const char *filename;
	
	if (!is_mounted)
	{
		if (!sdcard_carddetected())
			usb_printf("error: sd card not inserted\r\n");
		else
			usb_printf("error: sd card not mounted\r\n");
		return;
	}
	
	res = f_opendir(&rootDir,"0:");
	
	if (res != FR_OK)
	{
		printf("sdcard_listfiles: error %s\r\n",getError(res));
		usb_printf("error: %s\r\n",getError(res));
		return;
	}
	
	printf("Begin file list\r\n");
	//usb_printf("ok Files: {");
	usb_printf("Begin file list\r\n");
	while(1)
	{
		res = f_readdir(&rootDir,&fileInfo);
		if (res != FR_OK)
		{
			printf("sdcard_listfiles: error %s\r\n",getError(res));
			break;
		}
		
		if (fileInfo.fname[0] == 0)
			break;
			
		if (!(fileInfo.fattrib & AM_DIR))
		{
			filename = fileInfo.fname;
			usb_printf("%s\r\n",filename);
			printf("\t%s\n\r",filename);
		}
	}
	//usb_printf("}\r\n");
	usb_printf("End file list\r\n");
	
}



