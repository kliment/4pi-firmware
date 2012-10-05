/*
 Stepper Control
 Load Data from Plannerbuffer 
 
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
 
 
 
 #define MAX_STEP_FREQUENCY 30000
 
 
 
 extern const Pin X_MIN_PIN;
 extern const Pin Y_MIN_PIN;
 extern const Pin Z_MIN_PIN;
 extern const Pin X_MAX_PIN;
 extern const Pin Y_MAX_PIN;
 extern const Pin Z_MAX_PIN;
 
 extern unsigned char X_ENDSTOP_INVERT;
 extern unsigned char Y_ENDSTOP_INVERT;
 extern unsigned char Z_ENDSTOP_INVERT;
 
 extern unsigned char INVERT_X_DIR;
 extern unsigned char INVERT_Y_DIR;
 extern unsigned char INVERT_Z_DIR;
 extern unsigned char INVERT_E_DIR;

 
 
 void ConfigureTc0_Stepper(void);
 void stepper_setup(void);
 void enable_endstops(unsigned char check);
 
 
 