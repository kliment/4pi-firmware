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


 extern unsigned char buflen;
 extern unsigned char bufindr;
 extern unsigned char bufindw;

