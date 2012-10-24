/*
 G-Code Interpreter 
 Mikko Sivulainen <sivu@paeae.com>

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
#include "globals.h"

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
	uint32_t last_N;
	uint32_t line_N;
	char commandBuffer[BUFFER_SIZE];
	char* parsePos;
	ReplyFunction replyFunc;
} ParserState;


static ParserState parserState;

int32_t get_int(char chr)
{
	char* ptr = strchr(parserState.parsePos,chr);
	return ptr ? strtol(ptr+1,NULL,10) : 0;
}

uint32_t get_uint(char chr)
{
	char* ptr = strchr(parserState.parsePos,chr);
	return ptr ? strtoul(ptr+1,NULL,10) : 0;
}

float get_float(char chr)
{
	char* ptr = strchr(parserState.parsePos,chr);
	return ptr ? strtod(ptr+1,NULL) : 0;
}

const char* get_str(char chr)
{
	char *ptr = strchr(parserState.parsePos,chr);
	return ptr ? ptr+1 : NULL;
}

int has_code(char chr)
{
	return strchr(parserState.parsePos,chr) != NULL;
}

static uint8_t get_command()
{
	return parserState.parsePos[0];
}

static char* trim_line(char* line)
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

#define sendReply(...) { if (parserState.replyFunc) { parserState.replyFunc(__VA_ARGS__); } }

enum ProcessReply {
	NO_REPLY,
	SEND_REPLY,
};


#define GET_AXES(var,type,count) { int cnt_c; for(cnt_c = 0;cnt_c < count;cnt_c++) { if (has_code(axis_codes[cnt_c])) var[cnt_c] = get_##type(axis_codes[cnt_c]); } }
#define GET_ALL_AXES(var,type) GET_AXES(var,type,NUM_AXIS)

//process the actual gcode command
static int gcode_process_command()
{
	switch(get_command())
	{
		case 'G':
		{
			switch(get_int('G'))
			{
				case 0:
				case 1:
					get_coordinates();
					prepare_move();
					break;
				case 2:
					get_arc_coordinates();
					prepare_arc_move(1);
					break;
				case 3:
					get_arc_coordinates();
					prepare_arc_move(0);
					break;
				case 4:
				{
					uint32_t wait_until = 0;
					if(has_code('P')) 
						wait_until = get_uint('P'); // milliseconds to wait
					if(has_code('S')) 
						wait_until = get_uint('S') * 1000; // seconds to wait
					
					wait_until += timestamp;  // keep track of when we started waiting
					st_synchronize();  // wait for all movements to finish

					while(timestamp	 < wait_until )
					{
					}
					break;
				}
				case 21:
					break;
				case 28: //G28 Home all Axis one at a time
					saved_feedrate = feedrate;
					saved_feedmultiply = feedmultiply;
					previous_millis_cmd = timestamp;

					feedmultiply = 100;	   

					enable_endstops(1);

					feedrate = 0;
					is_homing = 1;

					home_all_axis = !((has_code(axis_codes[0])) || (has_code(axis_codes[1])) || (has_code(axis_codes[2])));

					if((home_all_axis) || (has_code(axis_codes[X_AXIS]))) 
						homing_routine(X_AXIS);

					if((home_all_axis) || (has_code(axis_codes[Y_AXIS]))) 
						homing_routine(Y_AXIS);

					if((home_all_axis) || (has_code(axis_codes[Z_AXIS]))) 
						homing_routine(Z_AXIS);

				#ifdef ENDSTOPS_ONLY_FOR_HOMING
					enable_endstops(0);
				#endif

					is_homing = 0;
					feedrate = saved_feedrate;
					feedmultiply = saved_feedmultiply;

					previous_millis_cmd = timestamp;
					break;
				case 90: // G90
					relative_mode = 0;
					break;
				case 91: // G91
					relative_mode = 1;
					break;
				case 92: // G92
				{
					if(!has_code(axis_codes[E_AXIS])) 
						st_synchronize();

					GET_ALL_AXES(current_position,float);
					plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

					break;
				}
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
				case 20: //list sd files
					sdcard_listfiles();
					return NO_REPLY;
				case 21: //init sd card
					sdcard_mount();
					break;
				case 22: //release sd card
					sdcard_unmount();
					break;
				case 23: //select sd file
					sdcard_selectfile(get_str(' '));
					break;
				case 24: //start/resume sd print
					sdcard_replaystart();
					break;
				case 25: //pause sd print
					sdcard_replaypause();
					break;
				case 26: //set sd position
					if (has_code('S'))
						sdcard_setposition(get_uint('S'));
					break;
				case 27: //sd print status
					sdcard_printstatus();
					return NO_REPLY;
				case 28: //begin write to sd file
					sdcard_selectfile(get_str(' '));
					sdcard_capturestart();
					break;
				case 29: //stop writing sd file
					sdcard_capturestop();
					break;
				
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
				case 82:
					axis_relative_modes[3] = 0;
					break;
				case 83:
					axis_relative_modes[3] = 1;
					break;
				case 84:
					st_synchronize(); // wait for all movements to finish
					if(has_code('S'))
					{
					  stepper_inactive_time = get_uint('S') * 1000; 
					}
					else if(has_code('T'))
					{
					  enable_x(); 
					  enable_y(); 
					  enable_z(); 
					  enable_e(); 
					}
					else
					{ 
					  disable_x(); 
					  disable_y(); 
					  disable_z(); 
					  disable_e(); 
					}
					break;
				case 85: // M85
					if (has_code('S'))
						max_inactive_time = get_uint('S') * 1000; 
					break;
				case 92: // M92
				{
					int cnt_c;
					for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
					{
						if(has_code(axis_codes[cnt_c])) 
						{
							pa.axis_steps_per_unit[cnt_c] = get_float(axis_codes[cnt_c]);
							axis_steps_per_sqr_second[cnt_c] = pa.max_acceleration_units_per_sq_second[cnt_c] * pa.axis_steps_per_unit[cnt_c];
				  		}
					}
					break;
				}
				case 93: // M93 show current axis steps.
					sendReply("X:%d Y:%d Z:%d E:%d",(int)pa.axis_steps_per_unit[0],(int)pa.axis_steps_per_unit[1],(int)pa.axis_steps_per_unit[2],(int)pa.axis_steps_per_unit[3]);
					break;
			  
				case 104: // M104
					if (has_code('S'))
					{
						if(active_extruder < MAX_EXTRUDER)
							heaters[active_extruder].target_temp = get_uint('S');
					}
			        break;
				case 105: // M105
					if(active_extruder < MAX_EXTRUDER)
					{
						sendReply("ok T:%u @%u B:%u ",heaters[active_extruder].akt_temp,heaters[active_extruder].pwm,bed_heater.akt_temp);
					}
					else
					{
						sendReply("ok T:%u @%u B:%u ",heaters[0].akt_temp,heaters[0].pwm,bed_heater.akt_temp);
					}
					return NO_REPLY;
				case 106: //M106 Fan On
					break;

				case 107: //M107 Fan Off
					break;
		      
				case 109: // M109 - Wait for extruder heater to reach target.
					if(active_extruder < MAX_EXTRUDER)
					{
						heater_struct* heater = &heaters[active_extruder];
	
						if (has_code('S')) 
							heater->target_temp = get_uint('S');

						uint32_t codenum = timestamp; 
						
						/* See if we are heating up or cooling down */
						 // true if heating, false if cooling
						unsigned char target_direction = (heater->akt_temp < heater->target_temp); 

					#ifdef TEMP_RESIDENCY_TIME
						long residencyStart;
						residencyStart = -1;
						/* continue to loop until we have reached the target temp   
						_and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
						while( (target_direction ? (heater->akt_temp < heater->target_temp) : (heater->akt_temp > heater->target_temp))
						|| (residencyStart > -1 && (timestamp - residencyStart) < TEMP_RESIDENCY_TIME*1000) )
						{
					#else
						while ( target_direction ? (heater->akt_temp < heater->target_temp) : (heater->akt_temp > heater->target_temp) ) 
						{
					#endif
							if( (timestamp - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up/cooling down
							{
								sendReply("ok T:%u\n\r",heater->akt_temp);
								codenum = timestamp;
							}
							#ifdef TEMP_RESIDENCY_TIME
							/* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
							or when current temp falls outside the hysteresis after target temp was reached */
							if (   (residencyStart == -1 &&  target_direction && heater->akt_temp >= heater->target_temp)
							|| (residencyStart == -1 && !target_direction && heater->akt_temp <= heater->target_temp)
							|| (residencyStart > -1 && labs(heater->akt_temp) - heater->target_temp > TEMP_HYSTERESIS) )
							{
								residencyStart = timestamp;
							}
							#endif
						}
					}
					break;
				case 110:
					break;
				case 114: // M114 Display current position
					sendReply("X:%f Y:%f Z:%f E:%f",current_position[0],current_position[1],current_position[2],current_position[3]);
		        	break;			  
				case 115: // M115
					sendReply("ok FIRMWARE_NAME: Sprinter 4pi PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa EXTRUDER_COUNT:%d\r\n",MAX_EXTRUDER);
					return NO_REPLY;
				case 119: // M119 show endstop state
				{
					char read_endstops[6] = {'X','X','X','X','X','X'};
				  
					if(pa.x_min_endstop_aktiv > -1)
						read_endstops[0] = (PIO_Get(&X_MIN_PIN) ^ pa.x_endstop_invert) + 48;

					if(pa.y_min_endstop_aktiv > -1)
						read_endstops[1] = (PIO_Get(&Y_MIN_PIN) ^ pa.y_endstop_invert) + 48;

					if(pa.z_min_endstop_aktiv > -1)
						read_endstops[2] = (PIO_Get(&Z_MIN_PIN) ^ pa.z_endstop_invert) + 48;

					if(pa.x_max_endstop_aktiv > -1)
						read_endstops[3] = (PIO_Get(&X_MAX_PIN) ^ pa.x_endstop_invert) + 48;

					if(pa.y_max_endstop_aktiv > -1)
						read_endstops[4] = (PIO_Get(&Y_MAX_PIN) ^ pa.y_endstop_invert) + 48;

					if(pa.z_max_endstop_aktiv > -1)
						read_endstops[5] = (PIO_Get(&Z_MAX_PIN) ^ pa.z_endstop_invert) + 48; 


					sendReply("Xmin:%c Ymin:%c Zmin:%c / Xmax:%c Ymax:%c Zmax:%c",read_endstops[0],read_endstops[1],read_endstops[2],read_endstops[3],read_endstops[4],read_endstops[5]);
					break;
  				}
				case 140: // M140 set bed temp
					if (has_code('S')) 
						bed_heater.target_temp = get_uint('S');

					break;
				case 190: // M190 - Wait for bed heater to reach target temperature.
				{
					if (has_code('S'))
						bed_heater.target_temp = get_float('S');

					uint32_t codenum = timestamp; 
					while(bed_heater.akt_temp < bed_heater.target_temp) 
					{
						if( (timestamp - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
						{
							if(active_extruder < MAX_EXTRUDER)
							{
								sendReply("T:%u B:%u",heaters[active_extruder].akt_temp,bed_heater.akt_temp);
							}
							else
							{
								sendReply("T:%u B:%u",heaters[0].akt_temp,bed_heater.akt_temp);
							}
							codenum = timestamp; 
						}
					}
					break;
				}
				case 201: // M201	 Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)
				{
					int cnt_c;
					for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
					{
						if(has_code(axis_codes[cnt_c]))
						{
							float acc = get_float(axis_codes[cnt_c]);
							pa.max_acceleration_units_per_sq_second[cnt_c] = acc;
							axis_steps_per_sqr_second[cnt_c] = acc * pa.axis_steps_per_unit[cnt_c];
						}
					}
					break;
				}
				case 202: // M202 max feedrate mm/sec
				{
					GET_ALL_AXES(pa.max_feedrate,float);
					break;
				}
				case 203: // M203 Temperature monitor
				 	//if(code_seen('S')) manage_monitor = code_value();
				 	//if(manage_monitor==100) manage_monitor=1; // Set 100 to heated bed
					break;
				case 204: // M204 acceleration S normal moves T filmanent only moves
					if(has_code('S')) 
						pa.move_acceleration = get_float('S');
					if(has_code('T'))
						pa.retract_acceleration = get_float('T');
					break;
				case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,	 B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E= max E jerk
					if(has_code('S')) 
						pa.minimumfeedrate = get_float('S');
					if(has_code('T')) 
						pa.mintravelfeedrate = get_float('T');
					//if(code_seen('B')) minsegmenttime = code_value() ;
					if(has_code('X')) 
						pa.max_xy_jerk = get_float('X');
					if(has_code('Z'))
						pa.max_z_jerk = get_float('Z');
					if(has_code('E'))
						pa.max_e_jerk = get_float('E');
					break;
				case 206: // M206 additional homing offset
				{
					if(has_code('D'))
					{
				 		sendReply("Addhome X:%g Y:%g Z:%g",add_homing[0],add_homing[1],add_homing[2]);
					}

					GET_AXES(add_homing,float,3);
					break;
				}	
				case 207: //M207 Homing Feedrate mm/min Xnnn Ynnn Znnn
				{
					GET_AXES(pa.homing_feedrate,float,3);
					break;
				}
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
		case 'T':
		{
			int new_extruder = get_uint('T');
			if (new_extruder >= MAX_EXTRUDER)
			{
				sendReply("Invalid extruder\n\r");
			}
			else
				active_extruder = new_extruder;
			
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
			int32_t line = get_int('N');
			
			if (line != parserState.last_N+1 && (!has_code('M') || get_uint('M') != 110))
			{
				sendReply("rs %u line number incorrect\r\n",parserState.last_N+1);
				return;
			}
			parserState.line_N = line;

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
			parserState.last_N = parserState.line_N;			
		}
		else if (strchr(parserState.commandBuffer,'*') != NULL)
		{
			sendReply("No line number with checksum\n\r");
			return;
		}

		parserState.parsePos = trim_line(parserState.commandBuffer);

//		DEBUG("gcode line: '%s'\n\r",parserState.parsePos);
		if (gcode_process_command() == SEND_REPLY)
		{
			sendReply("ok\r\n");
			previous_millis_cmd = timestamp;
		}
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
			case '(':
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


