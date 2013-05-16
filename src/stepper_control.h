/*
 Stepper Control
 Load Data from Plannerbuffer 

 Originally from Grbl (http://github.com/grbl/grbl)
 
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
#ifndef STEPPER_CONTROL_H_3FACLIDQ
#define STEPPER_CONTROL_H_3FACLIDQ

#include <pio/pio.h>

#define MAX_STEP_FREQUENCY 30000

extern const Pin X_MIN_PIN;
extern const Pin Y_MIN_PIN;
extern const Pin Z_MIN_PIN;
extern const Pin X_MAX_PIN;
extern const Pin Y_MAX_PIN;
extern const Pin Z_MAX_PIN;

void ConfigureTc0_Stepper(void);
void stepper_setup(void);
void enable_endstops(unsigned char check);
 
  
#endif /* end of include guard: STEPPER_CONTROL_H_3FACLIDQ */
