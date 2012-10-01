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
 
 // look here for descriptions of gcodes: http://linuxcnc.org/handbook/gcode/g-code.html
// http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M20  - List SD card
// M21  - Init SD card
// M22  - Release SD card
// M23  - Select SD file (M23 filename.g)
// M24  - Start/resume SD print
// M25  - Pause SD print
// M26  - Set SD position in bytes (M26 S12345)
// M27  - Report SD print status
// M28  - Start SD write (M28 filename.g)
// M29  - Stop SD write
//   -  <filename> - Delete file on sd card
// M42  - Set output on free pins, on a non pwm pin (over pin 13 on an arduino mega) use S255 to turn it on and S0 to turn it off. Use P to decide the pin (M42 P23 S255) would turn pin 23 on
// M80  - Turn on Power Supply
// M81  - Turn off Power Supply
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move, 
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M93  - Send axis_steps_per_unit
// M115	- Capabilities string
// M119 - Show Endstopper State 
// M140 - Set bed target temp
// M190 - Wait for bed current temp to reach target temp.
// M201 - Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M203 - Set temperture monitor to Sx
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2
// M205 - advanced settings:  minimum travel speed S=while printing T=travel only,  X=maximum xy jerk, Z=maximum Z jerk
// M206 - set additional homing offset

// M220 - set speed factor override percentage S=factor in percent 
// M221 - set extruder multiply factor S100 --> original Extrude Speed 

// M301 - Set PID parameters P I and D
// M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)

// M400 - Finish all moves

// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need to reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
// M503 - Print settings


#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <systick/systick.h>
#include <utility/trace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#include "serial.h"
#include "samadc.h"
#include "com_interpreter.h"
#include "heaters.h"
#include "planner.h"
#include "stepper_control.h"


extern void motor_enaxis(unsigned char axis, unsigned char en);


volatile char uart_in_buffer[256];
volatile unsigned char uart_wr_pointer = 0;
volatile unsigned char uart_rd_pointer = 0;


char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
unsigned char fromsd[BUFSIZE];


volatile unsigned char bufindr = 0;
volatile unsigned char bufindw = 0;
volatile unsigned char buflen = 0;
unsigned char serial_char;
volatile int serial_count = 0;
unsigned char comment_mode = 0;
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
long gcode_N, gcode_LastN;

//Inactivity shutdown variables
unsigned long previous_millis_cmd = 0;
unsigned long max_inactive_time = 0;
unsigned long stepper_inactive_time = 0;

unsigned char relative_mode = 0;
volatile signed short feedmultiply=100; //100->original / 200 -> Factor 2 / 50 -> Factor 0.5
signed short saved_feedmultiply = 0;
volatile char feedmultiplychanged=0;
volatile signed short extrudemultiply=100; //100->1 200->2

unsigned char active_extruder = 0;		//0 --> Exteruder 1 / 1 --> Extruder 2
unsigned char tmp_extruder = 0;


extern volatile unsigned long timestamp;

//extern int bed_temp_celsius;


void usb_characterhandler(unsigned char c){ 
    //every time the USB receives a new character, this function is called
	uart_in_buffer[uart_wr_pointer++] = c;
	if(uart_wr_pointer >= UART_BUFFER_SIZE)
		uart_wr_pointer = 0;
}


unsigned char get_byte_from_UART(unsigned char *zeichen)
{
	if(uart_rd_pointer == uart_wr_pointer)
		return(0);
		
	*zeichen = uart_in_buffer[uart_rd_pointer++];
	if(uart_rd_pointer >= UART_BUFFER_SIZE)
		uart_rd_pointer = 0;
		
	return(1);	
}


void ClearToSend()
{
	previous_millis_cmd = timestamp;
	usb_printf("ok\r\n");
}

void FlushSerialRequestResend()
{
	uart_rd_pointer = uart_wr_pointer;
	usb_printf("Resend:%u ok\r\n",gcode_LastN + 1);
}

void get_command() 
{ 
  while( get_byte_from_UART(&serial_char) != 0 && buflen < BUFSIZE)
  {
    if(serial_char == '\n' || serial_char == '\r' || (serial_char == ':' && comment_mode == 0) || serial_count >= (MAX_CMD_SIZE - 1) ) 
    {
      if(!serial_count) { //if empty line
        comment_mode = 0; // for new command
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string

        fromsd[bufindw] = 0;
        if(strstr(cmdbuffer[bufindw], "N") != NULL)
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) )
          {
            usb_printf("Serial Error: Line Number is not Last Line Number+1, Last Line:%u",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
    
          if(strstr(cmdbuffer[bufindw], "*") != NULL)
          {
            unsigned char checksum = 0;
            unsigned char count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');
  
            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum)
            {
              usb_printf("Error: checksum mismatch, Last Line:%u",gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              return;
            }
            //if no errors, continue parsing
          }
          else 
          {
            usb_printf("Error: No Checksum with line number, Last Line:%u",gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            return;
          }
    
          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        }
        else  // if we don't receive 'N' but still see '*'
        {
          if((strstr(cmdbuffer[bufindw], "*") != NULL))
          {
            usb_printf("Error: No Line Number with checksum, Last Line:%u",gcode_LastN);
            serial_count = 0;
            return;
          }
        }
        
				if((strstr(cmdbuffer[bufindw], "G") != NULL))
        {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL))))
          {
            case 0:
            case 1:
            #ifdef USE_ARC_FUNCTION
            case 2:  //G2
            case 3:  //G3 arc func
            #endif
              #ifdef SDSUPPORT
              if(savetosd)
                break;
              #endif
              usb_printf("ok\r\n");
            break;
            
            default:
            break;
          }
        }
        //Removed modulo (%) operator, which uses an expensive divide and multiplication
        //bufindw = (bufindw + 1) % BUFSIZE;
        bufindw++;
        if(bufindw == BUFSIZE) bufindw = 0;
        buflen += 1;

      comment_mode = 0; //for new command
      serial_count = 0; //clear buffer
    }
    else
    {
      if(serial_char == ';') comment_mode = 1;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }

}

float code_value() { return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); }
long code_value_long() { return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); }
unsigned char code_seen_str(char code_string[]) { return (strstr(cmdbuffer[bufindr], code_string) != NULL); }  //Return True if the string was found

unsigned char code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}


//------------------------------------------------
// CHECK COMMAND AND CONVERT VALUES
//------------------------------------------------
void process_commands()
{
  unsigned long codenum; //throw away variable
  unsigned char read_endstops[6] = {0,0,0,0,0,0};
  //char *starpos = NULL;
  unsigned char cnt_c = 0;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        get_coordinates(); // For X Y Z E F
        prepare_move();
        previous_millis_cmd = timestamp;
        return;
        //break;
      case 2: // G2  - CW ARC
        get_arc_coordinates();
        prepare_arc_move(1);
        previous_millis_cmd = timestamp;
        return;
      case 3: // G3  - CCW ARC
        get_arc_coordinates();
        prepare_arc_move(0);
        previous_millis_cmd = timestamp;
        return;  
      case 4: // G4 dwell
		codenum = 0;
		if(code_seen('P')) codenum = code_value(); // milliseconds to wait
		if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
		codenum += timestamp;  // keep track of when we started waiting
		st_synchronize();  // wait for all movements to finish

		while(timestamp  < codenum )
		{

		}
		break;
      case 28: //G28 Home all Axis one at a time
	    saved_feedrate = feedrate;
        saved_feedmultiply = feedmultiply;
        previous_millis_cmd = timestamp;
        
        feedmultiply = 100;    
      
        enable_endstops(1);
      
        /*
		for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
        {
          destination[cnt_c] = current_position[cnt_c];
        }
		*/
        feedrate = 0;
        is_homing = 1;

        home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));

        if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) 
          homing_routine(X_AXIS);

        if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) 
          homing_routine(Y_AXIS);

        if((home_all_axis) || (code_seen(axis_codes[Z_AXIS]))) 
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
		if(!code_seen(axis_codes[E_AXIS])) 
			st_synchronize();
          
        for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++)
        {
			if(code_seen(axis_codes[cnt_c])) current_position[cnt_c] = code_value();  
        }
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
        
		break;
      default:
        usb_printf("Unknown G-COM: %s \r\n",cmdbuffer[bufindr]);
      break;
    }
  }

  else if(code_seen('M'))
  {
    
    switch( (int)code_value() ) 
    {

      case 42: //M42 -Change pin status via gcode
        if (code_seen('S'))
        {

        }
        break;
      case 104: // M104
		if (code_seen('S'))
		{
			if(tmp_extruder < MAX_EXTRUDER)
				heaters[tmp_extruder].target_temp = code_value();
		}
        break;
      case 140: // M140 set bed temp
		if (code_seen('S')) bed_heater.target_temp = code_value();
        break;
      case 105: // M105
		  	if(tmp_extruder < MAX_EXTRUDER)
				usb_printf("ok T:%u @%u B:%u",heaters[tmp_extruder].akt_temp,heaters[tmp_extruder].pwm,bed_heater.akt_temp);
			else
				usb_printf("ok T:%u @%u B:%u",heaters[0].akt_temp,heaters[0].pwm,bed_heater.akt_temp);
        return;
        //break;
      case 109: // M109 - Wait for extruder heater to reach target.
		if(tmp_extruder < MAX_EXTRUDER)
		{
			if (code_seen('S')) heaters[tmp_extruder].target_temp = code_value();

			codenum = timestamp; 

			/* See if we are heating up or cooling down */
			 // true if heating, false if cooling
			unsigned char target_direction = (heaters[tmp_extruder].akt_temp < heaters[tmp_extruder].target_temp); 

		#ifdef TEMP_RESIDENCY_TIME
			long residencyStart;
			residencyStart = -1;
			/* continue to loop until we have reached the target temp   
			_and_ until TEMP_RESIDENCY_TIME hasn't passed since we reached it */
			while( (target_direction ? (heaters[tmp_extruder].akt_temp < heaters[tmp_extruder].target_temp) : (heaters[tmp_extruder].akt_temp > heaters[tmp_extruder].target_temp))
			|| (residencyStart > -1 && (timestamp - residencyStart) < TEMP_RESIDENCY_TIME*1000) )
			{
		#else
			while ( target_direction ? (heaters[tmp_extruder].akt_temp < heaters[tmp_extruder].target_temp) : (heaters[tmp_extruder].akt_temp > heaters[tmp_extruder].target_temp) ) 
			{
		#endif
				if( (timestamp - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up/cooling down
				{
					usb_printf("ok T:%u",heaters[tmp_extruder].akt_temp);
					codenum = timestamp;
				}
				#ifdef TEMP_RESIDENCY_TIME
				/* start/restart the TEMP_RESIDENCY_TIME timer whenever we reach target temp for the first time
				or when current temp falls outside the hysteresis after target temp was reached */
				if (   (residencyStart == -1 &&  target_direction && heaters[tmp_extruder].akt_temp >= heaters[tmp_extruder].target_temp)
				|| (residencyStart == -1 && !target_direction && heaters[tmp_extruder].akt_temp <= heaters[tmp_extruder].target_temp)
				|| (residencyStart > -1 && labs(heaters[tmp_extruder].akt_temp) - heaters[tmp_extruder].target_temp > TEMP_HYSTERESIS) )
				{
					residencyStart = timestamp;
				}
				#endif
			}
		}
		break;
		
      case 190: // M190 - Wait for bed heater to reach target temperature.
		
		if (code_seen('S')) bed_heater.target_temp = code_value();
		codenum = timestamp; 
		while(bed_heater.akt_temp < bed_heater.target_temp) 
		{
			if( (timestamp - codenum) > 1000 ) //Print Temp Reading every 1 second while heating up.
			{
				if(tmp_extruder < MAX_EXTRUDER)
					usb_printf("T:%u B:%u",heaters[tmp_extruder].akt_temp,bed_heater.akt_temp);
				else
					usb_printf("T:%u B:%u",heaters[0].akt_temp,bed_heater.akt_temp);
				
				codenum = timestamp; 
			}
		}
		break;
      case 106: //M106 Fan On

        break;
		
      case 107: //M107 Fan Off

        break;
      case 82:
        axis_relative_modes[3] = 0;
        break;
      case 83:
        axis_relative_modes[3] = 1;
        break;
      case 84:
        st_synchronize(); // wait for all movements to finish
        if(code_seen('S'))
        {
          stepper_inactive_time = code_value() * 1000; 
        }
        else if(code_seen('T'))
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
        code_seen('S');
        max_inactive_time = code_value() * 1000; 
        break;
	  case 92: // M92
        for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
        {
          if(code_seen(axis_codes[cnt_c])) 
          {
            axis_steps_per_unit[cnt_c] = code_value();
            axis_steps_per_sqr_second[cnt_c] = max_acceleration_units_per_sq_second[cnt_c] * axis_steps_per_unit[cnt_c];
          }
        }
        break;
      case 93: // M93 show current axis steps.
		usb_printf("ok X:%d Y:%d Z:%d E:%d",(int)axis_steps_per_unit[0],(int)axis_steps_per_unit[1],(int)axis_steps_per_unit[2],(int)axis_steps_per_unit[3]);
		//printf("ok X:%d Y:%d Z:%d E:%d",(int)axis_steps_per_unit[0],(int)axis_steps_per_unit[1],(int)axis_steps_per_unit[2],(int)axis_steps_per_unit[3]);
        break;
	  case 114: // M114 Display current position
		usb_printf("X:%d Y:%d Z:%d E:%d",(int)current_position[0],(int)current_position[1],(int)current_position[2],(int)current_position[3]);
        break;
      case 115: // M115
        usb_printf("FIRMWARE_NAME: Sprinter 4pi PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa EXTRUDER_COUNT:1\r\n");
        break;
	  case 119: // M119 show endstop state
		#if (X_MIN_ACTIV > -1)
			read_endstops[0] = PIO_Get(&X_MIN_PIN);
      	#endif
      	#if (X_MAX_ACTIV > -1)
			read_endstops[1] = PIO_Get(&Y_MIN_PIN);
      	#endif
      	#if (Y_MIN_ACTIV > -1)
			read_endstops[2] = PIO_Get(&Z_MIN_PIN);
      	#endif
      	#if (Y_MAX_ACTIV > -1)
			read_endstops[3] = PIO_Get(&X_MAX_PIN);
      	#endif
      	#if (Z_MIN_ACTIV > -1)
			read_endstops[4] = PIO_Get(&Y_MAX_PIN);
      	#endif
      	#if (Z_MAX_ACTIV > -1)
			read_endstops[5] = PIO_Get(&Z_MAX_PIN);
      	#endif
      
        usb_printf("Xmin:%d Ymin:%d Zmin:%d / Xmax:%d Ymax:%d Zmax:%d",read_endstops[0],read_endstops[1],read_endstops[2],read_endstops[3],read_endstops[4],read_endstops[5]);
		break;
	  case 201: // M201  Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)

        for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
        {
          if(code_seen(axis_codes[cnt_c]))
          {
            max_acceleration_units_per_sq_second[cnt_c] = code_value();
            axis_steps_per_sqr_second[cnt_c] = code_value() * axis_steps_per_unit[cnt_c];
          }
        }
        break;
      case 202: // M202 max feedrate mm/sec
        for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
        {
          if(code_seen(axis_codes[cnt_c])) max_feedrate[cnt_c] = code_value();
        }
      break;
      case 203: // M203 Temperature monitor
          //if(code_seen('S')) manage_monitor = code_value();
          //if(manage_monitor==100) manage_monitor=1; // Set 100 to heated bed
      break;
      case 204: // M204 acceleration S normal moves T filmanent only moves
          if(code_seen('S')) move_acceleration = code_value() ;
          if(code_seen('T')) retract_acceleration = code_value() ;
      break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E= max E jerk
        if(code_seen('S')) minimumfeedrate = code_value();
        if(code_seen('T')) mintravelfeedrate = code_value();
      //if(code_seen('B')) minsegmenttime = code_value() ;
        if(code_seen('X')) max_xy_jerk = code_value() ;
        if(code_seen('Z')) max_z_jerk = code_value() ;
        if(code_seen('E')) max_e_jerk = code_value() ;
      break;
      case 206: // M206 additional homing offset
        if(code_seen('D'))
        {
          usb_printf("Addhome X:%g Y:%g Z:%g",add_homing[0],add_homing[1],add_homing[2]);
        }

        for(cnt_c=0; cnt_c < 3; cnt_c++) 
        {
          if(code_seen(axis_codes[cnt_c])) add_homing[cnt_c] = code_value();
        }
      break;  	
	  case 220: // M220 S<factor in percent>- set speed factor override percentage
      {
        if(code_seen('S')) 
        {
          feedmultiply = code_value() ;
          feedmultiply = constrain(feedmultiply, 20, 200);
          feedmultiplychanged=1;
        }
      }
      break;
      case 221: // M221 S<factor in percent>- set extrude factor override percentage
      {
        if(code_seen('S')) 
        {
          extrudemultiply = code_value() ;
          extrudemultiply = constrain(extrudemultiply, 40, 200);
        }
      }
      break;
      case 301: // M301
      {
        if(tmp_extruder < MAX_EXTRUDER)
		{
			if(code_seen('P')) heaters[tmp_extruder].PID_Kp = code_value();
			if(code_seen('I')) heaters[tmp_extruder].PID_I = code_value();
			if(code_seen('D')) heaters[tmp_extruder].PID_Kd = code_value();
			heaters[tmp_extruder].temp_iState_max = (256L * PID_INTEGRAL_DRIVE_MAX) / (int)heaters[tmp_extruder].PID_I;
			heaters[tmp_extruder].temp_iState_min = heaters[tmp_extruder].temp_iState_max * (-1);
		}
      }
      break;
      case 303: // M303 PID autotune
      {
        //float help_temp = 150.0;
        //if (code_seen('S')) help_temp=code_value();
        //PID_autotune(help_temp);
      }
      break;
      case 400: // M400 finish all moves
      {
      	st_synchronize();	
      }
      break;	  

      default:
		usb_printf("Unknown M-COM: %s \r\n",cmdbuffer[bufindr]);
      break;

    }
    
  }
  else if(code_seen('T')) 
  {
    tmp_extruder = code_value();
    if(tmp_extruder >= MAX_EXTRUDER) 
	{
		//No more extruder
		usb_printf("Only 2 Extruder possible\r\n");
    }
    else 
	{
		active_extruder = tmp_extruder;
    }
  }
  else
  {
       usb_printf("Unknown command: %s \r\n",cmdbuffer[bufindr]);
  }
  
  ClearToSend();
      
}