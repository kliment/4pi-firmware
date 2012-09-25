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
#include "usb.h"




volatile char uart_in_buffer[256];
volatile unsigned char uart_wr_pointer = 0;
volatile unsigned char uart_rd_pointer = 0;


char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
unsigned char fromsd[BUFSIZE];


unsigned char bufindr = 0;
unsigned char bufindw = 0;
unsigned char buflen = 0;
unsigned char serial_char;
int serial_count = 0;
unsigned char comment_mode = 0;
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc
long gcode_N, gcode_LastN;

//extern int bed_temp_celsius;


void usb_characterhandler(unsigned char c){ 
    //every time the USB receives a new character, this function is called
	uart_in_buffer[uart_wr_pointer++] = c;
    //printf("%c, %u\n",c,uart_wr_pointer);
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
  //previous_millis_cmd = millis();
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
  char *starpos = NULL;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        //ClearToSend();
        return;
        //break;
      #ifdef USE_ARC_FUNCTION
      case 2: // G2  - CW ARC
        //break;
        return;
      case 3: // G3  - CCW ARC
        //break;
        return;  
      #endif  
      case 4: // G4 dwell
        break;
      case 28: //G28 Home all Axis one at a time
        break;
      case 90: // G90
        break;
      case 91: // G91
        break;
      case 92: // G92
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
		if (code_seen('S')) heaters[0].target_temp = code_value();
		heaters[1].target_temp = heaters[0].target_temp;
        break;
      case 140: // M140 set bed temp
		if (code_seen('S')) bed_heater.target_temp = code_value();
	  
        break;
      case 105: // M105

		  	usb_printf("ok T:%u @%u B:%u",heaters[0].akt_temp,heaters[0].pwm,bed_heater.akt_temp);

        return;
        //break;
      case 109: // M109 - Wait for extruder heater to reach target.

		break;
		
      case 190: // M190 - Wait for bed heater to reach target temperature.
		break;

      case 106: //M106 Fan On

        break;
		
      case 107: //M107 Fan Off

        break;
      case 82:
		break;
		
      case 83:
		break;
		
      case 84:
        break;
		
      case 85: // M85
        break;
		
      case 115: // M115
        usb_printf("FIRMWARE_NAME: Sprinter 4pi PROTOCOL_VERSION:1.0 MACHINE_TYPE:Prusa EXTRUDER_COUNT:1\r\n");
        break;

      default:
		usb_printf("Unknown M-COM: %s \r\n",cmdbuffer[bufindr]);
      break;

    }
    
  }
  else{
  
      usb_printf("Unknown command: %s \r\n",cmdbuffer[bufindr]);
  }
  
  ClearToSend();
      
}