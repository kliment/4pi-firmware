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
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */


 //Buffersize for USB UART receiving
 #define UART_BUFFER_SIZE	256
 
 //Command Buffer, can read BUFSIZE while doing the last command
 #define MAX_CMD_SIZE 96
 #define BUFSIZE 6

 unsigned char get_byte_from_UART(unsigned char *zeichen);

 void usb_characterhandler(unsigned char c);
 unsigned char get_byte_from_UART(unsigned char *zeichen);
 void process_commands();
 void get_command();

 float code_value();
 long code_value_long();
 unsigned char code_seen_str(char code_string[]);
 unsigned char code_seen(char code);
 

 extern volatile unsigned char buflen;
 extern volatile unsigned char bufindr;
 extern volatile unsigned char bufindw;
 extern unsigned char relative_mode;
 extern volatile signed short feedmultiply;
 extern signed short saved_feedmultiply;
 extern volatile char feedmultiplychanged;
 extern volatile signed short extrudemultiply;
 extern unsigned long previous_millis_cmd;
 extern unsigned long max_inactive_time;
 extern unsigned long stepper_inactive_time;
 extern unsigned char active_extruder;
 extern unsigned char tmp_extruder;



