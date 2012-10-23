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

#ifndef GCODE_PARSER_H_4G4XUC96
#define GCODE_PARSER_H_4G4XUC96

#include <inttypes.h>

typedef void (*ReplyFunction)(const char* format,...);

void gcode_init(ReplyFunction replyFunc);
void gcode_update();

int32_t get_int(char chr);
uint32_t get_uint(char chr);
float get_float(char chr);
const char* get_str(char chr);
int has_code(char chr);


#endif /* end of include guard: GCODE_PARSER_H_4G4XUC96 */
