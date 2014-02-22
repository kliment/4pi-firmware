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

// look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
/*
Implemented Codes
-------------------
 G0	 -> G1
 G1	 - Coordinated Movement X Y Z E
 G2	 - CW ARC
 G3	 - CCW ARC
 G4	 - Dwell S<seconds> or P<milliseconds>
 G28 - Home all Axis
 G90 - Use Absolute Coordinates
 G91 - Use Relative Coordinates
 G92 - Set current position to cordinates given

RepRap M Codes
 M104 - Set extruder target temp
 M105 - Read current temp
 M106 - Fan 1 on
 M107 - Fan 1 off
 M109 - Wait for extruder current temp to reach target temp.
 M114 - Display current position

Custom M Codes
 M20  - List SD card
 M21  - Init SD card
 M22  - Release SD card
 M23  - Select SD file (M23 filename.g)
 M24  - Start/resume SD print
 M25  - Pause SD print
 M26  - Set SD position in bytes (M26 S12345)
 M27  - Report SD print status
 M28  - Start SD write (M28 filename.g)
 M29  - Stop SD write
   -  <filename> - Delete file on sd card
 M42  - Set output on free pins (not implemented)
 M44  - Boot From ROM (load bootloader for uploading firmware)
 M80  - Turn on Power Supply (not implemented)
 M81  - Turn off Power Supply (not implemented)
 M82  - Set E codes absolute (default)
 M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 M84  - Disable steppers until next move, 
		or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.	S0 to disable the timeout.
 M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 M92  - Set axis_steps_per_unit - same syntax as G92
 M93  - Send axis_steps_per_unit
 M115	- Capabilities string
 M119 - Show Endstop State 
 M140 - Set bed target temp
 M176 - Fan 2 on
 M177 - Fan 2 off
 M190 - Wait for bed current temp to reach target temp.
 M201 - Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 M202 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 M203 - Set temperture monitor to Sx
 M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2
 M205 - advanced settings:	minimum travel speed S=while printing T=travel only,  X=maximum xy jerk, Z=maximum Z jerk
 M206 - set additional homing offset
 M207 - set homing feedrate mm/min (M207 X1500 Y1500 Z120)

 M220 - set speed factor override percentage S=factor in percent 
 M221 - set extruder multiply factor S100 --> original Extrude Speed 

Note: M301, M303, M304 applies to currently selected extruder.	Use T0 or T1 to select.
 M301 - Set Heater parameters P, I, D, S (slope), B (y-intercept), W (maximum pwm)
 M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
 M304 - Calculate slope and y-intercept for HEATER_DUTY_FOR_SETPOINT formula.
		 Caution - this can take 30 minutes to complete and will heat the hotend 
				   to 200 degrees.

 M400 - Finish all moves

 M510 - Invert axis, 0=false, 1=true (M510 X0 Y0 Z0 E1)
 M520 - Set maximum print area (M520 X200 Y200 Z150)
 M521 - Disable axis when unused (M520 X0 Y0 Z1 E0)
 M522 - Use software endstops I=min, A=max, 0=false, 1=true (M522 I0 A1)
 M523 - Enable min endstop input 1=true, -1=false (M523 X1 Y1 Z1)
 M524 - Enable max endstop input 1=true, -1=false (M524 X-1 Y-1 Z-1)
 M525 - Set homing direction 1=+, -1=- (M525 X-1 Y-1 Z-1)
 M526 - Invert endstop inputs 0=false, 1=true (M526 X0 Y0 Z0)
 
Note: M530, M531 applies to currently selected extruder.  Use T0 or T1 to select.
 M530 - Set heater sensor (thermocouple) type B (bed) E (extruder) (M530 E11 B11)
 M531 - Set heater PWM mode 0=false, 1=true (M531 E1)
 
 M350 - Set microstepping steps (M350 X16 Y16 Z16 E16 B16)
 M906 - Set motor current (mA) (M906 X1000 Y1000 Z1000 E1000 B1000) or set all (M906 S1000)
 M907 - Set motor current (raw) (M907 X128 Y128 Z128 E128 B128) or set all (M907 S128)

 M500 - stores paramters in EEPROM
 M501 - reads parameters from EEPROM (if you need to reset them after you changed them temporarily).
 M502 - reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
 M503 - Print settings
 M505 - Save Parameters to SD-Card

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

uint32_t get_bool(char chr)
{
	return get_int(chr) ? 1 : 0;
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

heater_struct* get_heater(int idx)
{
	if (idx < MAX_EXTRUDER)
		return &heaters[idx];
		
	return NULL;
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
#define GET(code,default_value) has_code(code) ? get_int(code) : default_value


//process the actual gcode command
static int gcode_process_command()
{
	switch(get_command())
	{
		case 'G':
		{
			if (sdcard_iscapturing()) {
				sdcard_writeline(parserState.parsePos);
				break;
			}
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
			int mcode = get_int('M');
			if (sdcard_iscapturing() && (mcode < 20 || mcode > 29)) {
				sdcard_writeline(parserState.parsePos);
				break;
			}
			switch(mcode)
			{
				case 20: //list sd files
					sdcard_listfiles();
//					break;
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
					//sdcard_selectfile();
					sdcard_capturestart(get_str(' '));
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
						heater_struct* heater = get_heater(GET('T',GET('P',active_extruder)));
						if (heater)
							heater->target_temp = get_uint('S');
					}
					break;
				case 105: // M105
				{
					heater_struct* heater = get_heater(GET('T',GET('P',active_extruder)));
                    
					if (heater)
					{
                        const char* ok = (sdcard_isreplaying()) ? "" : "ok ";
                        sendReply("%sT:%u @%u B:%u \r\n",ok,heater->akt_temp,heater->pwm,bed_heater.akt_temp);
					}
					return NO_REPLY;
				}
				case 106: //M106 Fan 1 On
					  if (has_code('S'))
					  {
						g_pwm_value[2] = constrain(get_uint('S'),0,255);		  
					  }
					  else 
					  {
						g_pwm_value[2] = 255;
					  }
					  g_pwm_aktiv[2] = 1;
					  break;
				case 107: //M107 Fan 1 Off
					  g_pwm_value[2] = 0;
					  break;
				case 109: // M109 - Wait for extruder heater to reach target.
				{
					heater_struct* heater = get_heater(GET('T',GET('P',active_extruder)));
				
					if (heater)
					{
	
						int min_target,max_target;

						if (has_code('S'))
						{
							heater->target_temp = get_uint('S');
							min_target = heater->target_temp;
						}
						else
						{
							min_target = heater->target_temp - TEMP_HYSTERESIS;
						}
						
						max_target = has_code('R') ? get_uint('R') : heater->target_temp + TEMP_HYSTERESIS;
					
						uint32_t codenum = timestamp; 

						//loops separated for cleanliness
					#ifdef TEMP_RESIDENCY_TIME
						long residencyStart;
						residencyStart = -1;

						while(1)
						{
							if (heater->akt_temp < min_target || heater->akt_temp > max_target)
							{
								residencyStart = -1;
								if( (timestamp - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up/cooling down
								{
									sendReply("T:%u \r\n",heater->akt_temp);
									codenum = timestamp;
								}
							}
							else
							{
								if ((residencyStart > (-1)) && ((timestamp - residencyStart) > TEMP_RESIDENCY_TIME*1000))
								{
									break; //done
								}
								else
								{
									residencyStart = timestamp;
								}
							}
						}
					#else
						while(1)
						{
							if (heater->akt_temp < min_target || heater->akt_temp > max_target)
							{
								if( (timestamp - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up/cooling down
								{
									sendReply("T:%u \r\n",heater->akt_temp);
									codenum = timestamp;
								}
							}
							else
							{
								break; //done
							}
						}
					}
					#endif
					break;
				}
				case 110:
					break;
				case 114: // M114 Display current position
					sendReply("X:%f Y:%f Z:%f E:%f ",current_position[0],current_position[1],current_position[2],current_position[3]);
					break;			  
				case 115: // M115
                {
                    const char* ok = (sdcard_isreplaying()) ? "" : "ok ";
					sendReply("%sFIRMWARE_NAME: Sprinter 4pi PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa EXTRUDER_COUNT:%d\r\n", ok, MAX_EXTRUDER);
					return NO_REPLY;
                }
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


					sendReply("Xmin:%c Ymin:%c Zmin:%c / Xmax:%c Ymax:%c Zmax:%c ",read_endstops[0],read_endstops[1],read_endstops[2],read_endstops[3],read_endstops[4],read_endstops[5]);
					break;
				}
				case 140: // M140 set bed temp
					if (has_code('S')) 
						bed_heater.target_temp = get_uint('S');

					break;
				case 176: //M176 Fan 2 On
					  if (has_code('S'))
					  {
						g_pwm_value[3] = constrain(get_uint('S'),0,255);		  
					  }
					  else 
					  {
						g_pwm_value[3] = 255;
					  }
					  g_pwm_aktiv[3] = 1;
					  break;
				case 177: //M177 Fan 2 Off
					  g_pwm_value[3] = 0;
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
							heater_struct* heater = get_heater(GET('T',GET('P',active_extruder)));

							if (heater)
							{
								sendReply("T:%u B:%u\r\n",heater->akt_temp,bed_heater.akt_temp);
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
				case 205: //M205 advanced settings:	 minimum travel speed S=while printing T=travel only,	 B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E= max E jerk
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
					GET_AXES(pa.add_homing,float,3);
					break;
				}	
				case 207: //M207 Homing Feedrate mm/min Xnnn Ynnn Znnn
				{
					GET_AXES(pa.homing_feedrate,float,3);
					break;
				}
				case 220: // M220 S<factor in percent>- set speed factor override percentage
				{
					if(has_code('S')) 
					{
						feedmultiply = constrain(get_uint('S'), 20, 200);
						feedmultiplychanged=1;
					}
				  break;
				}
				case 221: // M221 S<factor in percent>- set extrude factor override percentage
				{
					if(has_code('S')) 
					{
						extrudemultiply = constrain(get_uint('S'), 40, 200);
					}
					break;
				}
				case 301: // M301
				{
					int extruder = GET('T',active_extruder);
					heater_struct* heater = get_heater(extruder);
		
					if (heater)
					{
						if(has_code('P'))
							heater->PID_Kp = pa.heater_pTerm[extruder] = get_uint('P');

						if(has_code('I'))
							heater->PID_I	 = pa.heater_iTerm[extruder] = get_uint('I');

						if(has_code('D'))
							heater->PID_Kd = pa.heater_dTerm[extruder] = get_uint('D');

						if(has_code('S'))
							heater->slope = pa.heater_slope[extruder] = get_uint('S');

						if(has_code('B'))
							heater->intercept = pa.heater_intercept[extruder] = get_uint('B');

						if(has_code('W'))
							heater->max_pwm = pa.heater_max_pwm[extruder] = get_uint('W');

						heater->temp_iState_max = (256L * PID_INTEGRAL_DRIVE_MAX) / (int)heater->PID_I;
						heater->temp_iState_min = heater->temp_iState_max * (-1);
					}
					break;
				}
				case 303: // M303 PID autotune
				{
					heater_struct* heater = get_heater(GET('T',GET('P',active_extruder)));

					if (heater)
					{

						float help_temp = 150.0;

						if (has_code('S')) 
							help_temp=get_float('S');

						PID_autotune(heater, help_temp);
					}
					return NO_REPLY;
				}
				case 304: // M304 Evaluate heater performance
				{
					heater_struct* heater = get_heater(GET('T',GET('P',active_extruder)));

					if (heater)
					{
						unsigned int step = 10;

						if (has_code('S')) 
							step=get_uint('S');

						Heater_Eval(heater, step);
					}
					return NO_REPLY;
				}
				case 400: // M400 finish all moves
				{
					st_synchronize();	
					break;

				}
				case 350: // Set microstepping mode (1=full step, 2=1/2 step, 4=1/4 step, 16=1/16 step).
				//Warning: Steps per unit remains unchanged. 
				// M350 X[value] Y[value] Z[value] E[value] B[value] 
				// M350 S[value] set all motors
				{
					int cnt_c;
					for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
					{
						if(has_code(axis_codes[cnt_c])) 
						{
							pa.axis_ustep[cnt_c] = microstep_mode(get_uint(axis_codes[cnt_c]));
							motor_setopts(cnt_c,pa.axis_ustep[cnt_c],pa.axis_current[cnt_c]);
						}
					}
					
					if(has_code('B'))
					{
						pa.axis_ustep[4] = microstep_mode(get_uint('B'));
						motor_setopts(4,pa.axis_ustep[4],pa.axis_current[4]);
					}
					 
					if(has_code('S'))
					{
						for(cnt_c=0; cnt_c<5; cnt_c++)
						{
							pa.axis_ustep[cnt_c] = microstep_mode(get_uint('S'));
							motor_setopts(cnt_c,pa.axis_ustep[cnt_c],pa.axis_current[cnt_c]);
						}
					}
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
				case 505:
					FLASH_Store_to_SD();
					break;
				case 510: // M510 Axis invert
					if(has_code('X'))
						pa.invert_x_dir = get_bool('X');

					if(has_code('Y'))
						pa.invert_y_dir = get_bool('Y');

					if(has_code('Z'))
						pa.invert_z_dir = get_bool('Z');

					if(has_code('E'))
						pa.invert_e_dir = get_bool('E');

					break;
				case 520: // M520 Maximum Area unit
					if(has_code('X'))
						pa.x_max_length = get_int('X');

					if(has_code('Y'))
						pa.y_max_length = get_int('Y');

					if(has_code('Z'))
						pa.z_max_length = get_int('Z');

					break;
				case 521: // M521 Disable axis when unused
					if(has_code('X'))
						pa.disable_x_en = get_bool('X');

					if(has_code('Y'))
						pa.disable_y_en = get_bool('Y');

					if(has_code('Z'))
						pa.disable_z_en = get_bool('Z');

					if(has_code('E'))
						pa.disable_e_en = get_bool('E');

					break;
				case 522: // M522 Software Endstop
					if(has_code('I'))
						pa.min_software_endstops = get_bool('I');

					if(has_code('A'))
						pa.max_software_endstops = get_bool('A');

					break;
				case 523: // M523 MIN Endstop
					if(has_code('X'))
						pa.x_min_endstop_aktiv = get_int('X')==1 ? 1 : -1;

					if(has_code('Y'))
						pa.y_min_endstop_aktiv = get_int('Y')==1 ? 1 : -1;

					if(has_code('Z'))
						pa.z_min_endstop_aktiv = get_int('Z')==1 ? 1 : -1;

					break;
				case 524: // M524 MAX Endstop
					if(has_code('X'))
						pa.x_max_endstop_aktiv = get_int('X')==1 ? 1 : -1;

					if(has_code('Y'))
						pa.y_max_endstop_aktiv = get_int('Y')==1 ? 1 : -1;

					if(has_code('Z'))
						pa.z_max_endstop_aktiv = get_int('Z')==1 ? 1 : -1;

					break;
				case 525: // M525 Homing Direction
					if(has_code('X'))
						pa.x_home_dir = get_int('X')==1 ? 1 : -1;

					if(has_code('Y'))
						pa.y_home_dir = get_int('Y')==1 ? 1 : -1;

					if(has_code('Z'))
						pa.z_home_dir = get_int('Z')==1 ? 1 : -1;

					break;
				case 526: // M526 Endstop Invert
					if(has_code('X'))
						pa.x_endstop_invert = get_bool('X');

					if(has_code('Y'))
						pa.y_endstop_invert = get_bool('Y');

					if(has_code('Z'))
						pa.z_endstop_invert = get_bool('Z');

					break;
				case 530: // M530 Heater Sensor
				{
					int extruder = GET('T',GET('P',active_extruder));
					heater_struct* heater = get_heater(extruder);

					if (heater)
					{
						if(has_code('E')) 
							heater->thermistor_type = pa.heater_thermistor_type[extruder] = get_uint('E');
					}
					
					if(has_code('B')) 
						bed_heater.thermistor_type = pa.bed_thermistor_type = get_uint('B');
					
					break;
				}
	
				case 531: // M531 Heater PWM
				{
					int extruder = GET('T',GET('P',active_extruder));
					heater_struct* heater = get_heater(extruder);

					if (heater)
					{
						if(has_code('E')) 
							heater->pwm = pa.heater_pwm_en[extruder] = get_bool('E');
					}
					
					break;
				}
				case 906: // set motor current value in mA using axis codes
				// M906 X[mA] Y[mA] Z[mA] E[mA] B[mA] 
				// M906 S[mA] set all motors current 
				{
					int cnt_c;
					unsigned int ma;
		
					for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
					{
						if(has_code(axis_codes[cnt_c])) 
						{
							ma = constrain(get_uint(axis_codes[cnt_c]),0,1900);
							pa.axis_current[cnt_c] = ma_count(ma);
							motor_setopts(cnt_c,pa.axis_ustep[cnt_c],pa.axis_current[cnt_c]);
						}
					}
					
					if(has_code('B'))
					{
						ma = constrain(get_uint('B'),0,1900);
						pa.axis_current[4] = ma_count(ma);
						motor_setopts(4,pa.axis_ustep[4],pa.axis_current[4]);
					}
					  
					if(has_code('S'))
					{
						for(cnt_c=0; cnt_c<5; cnt_c++)
						{
							ma = constrain(get_uint('S'),0,1900);
							pa.axis_current[cnt_c] = ma_count(ma);
							motor_setopts(cnt_c,pa.axis_ustep[cnt_c],pa.axis_current[cnt_c]);
						}
					}
					break;
				}
				case 907: // set motor current value (0-255) using axis codes
				// M907 X[value] Y[value] Z[value] E[value] B[value] 
				// M907 S[value] set all motors current 
				{
					int cnt_c;
					for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
					{
						if(has_code(axis_codes[cnt_c])) 
						{
							pa.axis_current[cnt_c] = constrain(get_uint(axis_codes[cnt_c]),0,255);
							motor_setopts(cnt_c,pa.axis_ustep[cnt_c],pa.axis_current[cnt_c]);
						}
					}
					  
					if(has_code('B'))
					{
						pa.axis_current[4] = constrain(get_uint('B'),0,255);
						motor_setopts(4,pa.axis_ustep[4],pa.axis_current[4]);
					}
					  
					if(has_code('S'))
					{
						for(cnt_c=0; cnt_c<5; cnt_c++)
						{
							pa.axis_current[cnt_c] = constrain(get_uint('S'),0,255);
							motor_setopts(cnt_c,pa.axis_ustep[cnt_c],pa.axis_current[cnt_c]);
						}
					}
					break;	  
				}
			  default:
					sendReply("Unknown M%d\n\r",get_int('M'));
					return NO_REPLY;
			}
			break;
		}
		case 'T':
		{
			if (sdcard_iscapturing()) {
				sdcard_writeline(parserState.parsePos);
				break;
			}
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
            if (sdcard_isreplaying() == false)
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
	uint8_t chr='\0';
	while (ringbuffer_numAvailable(&uartBuffer) > 0)
	{
		chr = ringbuffer_get(&uartBuffer);
		
		switch(chr)
		{
			case '\0':
				break;
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
	if(parserState.commandLen == 0 && sdcard_isreplaying() && !sdcard_isreplaypaused()){
		int newline=0;
        unsigned char nchar=0;
		while(!newline){
			int x=sdcard_getchar(&nchar);
			if(!x){
				sendReply("Done printing file\n\r");
				sdcard_replaystop();
				newline=1;
				break;
			}
//			printf("%c\n\r",nchar);
		switch(nchar)
		{
			case '\0':
				newline=1;
				break;
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
				newline=1;
				break;
			default:
				if (parserState.commandLen >= BUFFER_SIZE)
				{
					printf("error: command buffer full!\n\r");
				}
				else
				{
					if (!parserState.comment_mode)
						parserState.commandBuffer[parserState.commandLen++] = nchar;
				}
				break;
		}
	
		}
	}
	
}


