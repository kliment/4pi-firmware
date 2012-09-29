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
 
 //-----------------------------------------------------------------------
//// ENDSTOP SETTINGS:
//-----------------------------------------------------------------------
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1
 
#define X_MIN_ACTIV          1
#define X_MAX_ACTIV          -1

#define Y_MIN_ACTIV          1
#define Y_MAX_ACTIV          -1

#define Z_MIN_ACTIV          1
#define Z_MAX_ACTIV          -1
 
 
 
 void ConfigureTc0_Stepper(void);
 void stepper_setup(void);
 void enable_endstops(unsigned char check);
 
 
 