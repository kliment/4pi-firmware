/*
 G-Code Interpreter
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. 
*/

#include <inttypes.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>

#include <board.h>


#include "init_configuration.h"
#include "gcode_parser.h"
#include "serial.h"
#include "parameters.h"
#include "samadc.h"
#include "heaters.h"
#include "planner.h"
#include "stepper_control.h"
#include "motoropts.h"
#include "sdcard.h"

#define BUFFER_SIZE 256

#define DEBUG_PARSER


#ifdef DEBUG_PARSER
#define DEBUG(...) printf(__VA_ARGS__);
#else
#define DEBUG(...)
#endif

typedef struct 
{
	int readPos;
	int writePos;
	int numBytes;
	uint8_t buffer[BUFFER_SIZE];
} RingBuffer;


void ringbuffer_init(RingBuffer* pBuffer)
{
	memset(pBuffer,0,sizeof(RingBuffer));
}

int ringbuffer_numFree(const RingBuffer* pBuffer)
{
	return BUFFER_SIZE-pBuffer->numBytes;
}

int ringbuffer_numAvailable(const RingBuffer* pBuffer)
{
	return pBuffer->numBytes;
}

uint8_t ringbuffer_get(RingBuffer* pBuffer)
{
	uint8_t b = pBuffer->buffer[pBuffer->readPos++];
	pBuffer->readPos &= (BUFFER_SIZE-1);
	pBuffer->numBytes--;
	return b;
}

void ringbuffer_put(RingBuffer* pBuffer,uint8_t b)
{
	pBuffer->buffer[pBuffer->writePos++] = b;
	pBuffer->writePos &= (BUFFER_SIZE-1);
	pBuffer->numBytes++;
}


//----------------------------------------------------------------------------------------------
static RingBuffer uartBuffer;

static void gcode_datareceived(unsigned char chr)
{
	if (ringbuffer_numFree(&uartBuffer))
		ringbuffer_put(&uartBuffer,chr);
}


typedef struct 
{
	int comment_mode : 1;
	int commandLen;
	long last_N;
	long line_N;
	char commandBuffer[BUFFER_SIZE];
	char* parsePos;
	ReplyFunction replyFunc;
} ParserState;


static ParserState parserState;

static int32_t get_int(char chr)
{
	char* ptr = strchr(parserState.parsePos,chr);
	return ptr ? strtol(ptr+1,NULL,10) : 0;
}

static uint32_t get_uint(char chr)
{
	char* ptr = strchr(parserState.parsePos,chr);
	return ptr ? strtoul(ptr+1,NULL,10) : 0;
}

static float get_float(char chr)
{
	char* ptr = strchr(parserState.parsePos,chr);
	return ptr ? strtod(ptr+1,NULL) : 0;
}

static const char* get_str(char chr)
{
	char *ptr = strchr(parserState.parsePos,chr);
	return ptr ? ptr+1 : NULL;
}

static uint8_t get_command()
{
	return parserState.parsePos[0];
}

static char* trim_line(const char* line)
{
	int i;
	char command_chars[] = {'G','M','T'};
	
	while(*line)
	{
		for (i=0;i<sizeof(command_chars);i++)
		{
			if (*line == command_chars[i])
				return line;
		}
		line++;
	}
	return line;
}

static uint8_t calculate_checksum(const char* pLine)
{
	uint8_t checksum = 0;
	while(*pLine != '*')
		checksum ^= *pLine++;
		
	return checksum;
}

#define sendReply(...) if (parserState.replyFunc) { parserState.replyFunc(__VA_ARGS__); } 


enum ProcessReply {
	NO_REPLY,
	SEND_REPLY,
};

//process the actual gcode command
static int gcode_process_command()
{
	switch(get_command())
	{
		case 'G':
		{
			switch(get_int('G'))
			{
				
				default:
					sendReply("Unknown G%d\n\r",get_int('G'));
					return NO_REPLY;
			}
			break;
		}
		case 'M':
		{
			switch(get_int('M'))
			{
				case 44:
					if (strcmp(get_str(' '),"IKnowWhatIAmDoing") == 0)
					{
						FLASH_BootFromROM();
						sendReply("bootloader enabled\n\r")
					}
					else
					{
            			FLASH_BootFromFLASH();
					}
					break;
				case 105: // M105
//				  	if(tmp_extruder < MAX_EXTRUDER)
//						sendReply("ok T:%u @%u B:%u ",heaters[tmp_extruder].akt_temp,heaters[tmp_extruder].pwm,bed_heater.akt_temp);
//					else
						sendReply("ok T:%u @%u B:%u ",heaters[0].akt_temp,heaters[0].pwm,bed_heater.akt_temp);
					return NO_REPLY;
				case 110:
					parserState.last_N = get_int('N');
					break;
				case 115: // M115
					usb_printf("ok FIRMWARE_NAME: Sprinter 4pi PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa EXTRUDER_COUNT:%d\r\n",MAX_EXTRUDER);
					return NO_REPLY;
				 case 500: // M500 - stores paramters in EEPROM
					FLASH_StoreSettings();
				 	break;
				 case 501: // M501 - reads parameters from EEPROM (if you need to reset them after you changed them temporarily).
					FLASH_LoadSettings();
				 	break;
				 case 502:	// M502 - reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
					init_parameters();
				 	break;
				 case 503:	//M503 show settings
					FLASH_PrintSettings();
					break;
				default:
					sendReply("Unknown M%d\n\r",get_int('M'));
					return NO_REPLY;
			}
			break;
		}
		default:
			sendReply("Unknown command %c\n\r",get_command());
			return NO_REPLY;
	}
	return SEND_REPLY;
}


//full line has been received, process it for line number, checksum, etc. before processing the actual command
static void gcode_line_received()
{
	if (parserState.commandLen)
	{
		if (parserState.commandBuffer[0] == 'N')
		{
			parserState.line_N = get_uint('N');
			
			if (parserState.line_N != parserState.last_N+1)
			{
				sendReply("rs %u line number incorrect\r\n",parserState.last_N+1);
				return;
			}

			char* ptr;
			if ((ptr = strchr(parserState.parsePos,'*')) != NULL)
			{
				if (get_uint('*') != calculate_checksum(parserState.commandBuffer))
				{
					sendReply("rs %u incorrect checksum\r\n",parserState.last_N+1);
					return;
				}
				*ptr = 0;
			}
			else
			{
				sendReply("No checksum with line number\n\r");
				return;
			}
			
		}
		else if (strchr(parserState.commandBuffer,'*') != NULL)
		{
			sendReply("No line number with checksum\n\r");
			return;
		}

		parserState.parsePos = trim_line(parserState.commandBuffer);

		DEBUG("original line: '%s'\n\r",parserState.commandBuffer);
		DEBUG("gcode line: '%s'\n\r",parserState.parsePos);
		if (gcode_process_command() == SEND_REPLY)
		{
			sendReply("ok\r\n");
		}
		parserState.last_N = parserState.line_N;
	}
	
}

void gcode_init(ReplyFunction replyFunc)
{
	ringbuffer_init(&uartBuffer);
	memset(&parserState,0,sizeof(ParserState));
	parserState.replyFunc = replyFunc;
	
	samserial_setcallback(gcode_datareceived);
}

void gcode_update()
{
	while (ringbuffer_numAvailable(&uartBuffer) > 0)
	{
		uint8_t chr = ringbuffer_get(&uartBuffer);
		
		switch(chr)
		{
			case ';':
				parserState.comment_mode = true;
				break;
			case '\n':
			case '\r':
				parserState.commandBuffer[parserState.commandLen] = 0;
				parserState.parsePos = parserState.commandBuffer;
				gcode_line_received();
				parserState.comment_mode = false;
				parserState.commandLen = 0;
				
				break;
			default:
				if (parserState.commandLen >= BUFFER_SIZE)
				{
					printf("error: command buffer full!\n\r");
				}
				else
				{
					if (!parserState.comment_mode)
						parserState.commandBuffer[parserState.commandLen++] = chr;
				}
				break;
		}
		
	}
	
}


