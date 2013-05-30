/*
 
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
 
 
 #define NUM_AXIS 4
 #define MAX_EXTRUDER 2
 
 #define FLASH_VERSION "F02" 
  
 
 typedef struct {
	//Version and Chksum, dont change this place of this parameters
	unsigned short chk_sum;		//16 bit CRC checksum
	char version[4];			//Settings Version like F001
	
	//Planner Parameter
	float max_feedrate[NUM_AXIS];
	float homing_feedrate[3];
	float add_homing[3];
	float axis_steps_per_unit[NUM_AXIS];
	unsigned long max_acceleration_units_per_sq_second[NUM_AXIS];
	float minimumfeedrate;
	float retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
	float max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
	float max_z_jerk;
	float max_e_jerk;
	float mintravelfeedrate;
	float move_acceleration;       
	
	//Software Endstops YES / NO
	unsigned char min_software_endstops;
	unsigned char max_software_endstops;
	
	//Homing direction
	signed short x_home_dir;
	signed short y_home_dir;
	signed short z_home_dir;
	
	//Invert endstop signal
	volatile unsigned char x_endstop_invert;
	volatile unsigned char y_endstop_invert;
	volatile unsigned char z_endstop_invert;
	
	//Digital Input for Endstop switch aktiv
	volatile signed short x_min_endstop_aktiv;
	volatile signed short x_max_endstop_aktiv;
	volatile signed short y_min_endstop_aktiv;
	volatile signed short y_max_endstop_aktiv;
	volatile signed short z_min_endstop_aktiv;
	volatile signed short z_max_endstop_aktiv;
	
	//Invert axis direction 
	volatile unsigned char invert_x_dir;
	volatile unsigned char invert_y_dir;
	volatile unsigned char invert_z_dir;
	volatile unsigned char invert_e_dir;
	
	//maximum Printing area
	signed short x_max_length;
	signed short y_max_length;
	signed short z_max_length;
	
	//Disable axis when not used
	unsigned char disable_x_en;
	unsigned char disable_y_en;
	unsigned char disable_z_en;
	unsigned char disable_e_en;
	
	//Motor Settings
	unsigned char axis_current[5];
	unsigned char axis_ustep[5];
	
	//Heater Sensor Settings
	unsigned char heater_thermistor_type[MAX_EXTRUDER];
	unsigned char bed_thermistor_type;
	
	//Softpwm enable
	unsigned char heater_pwm_en[MAX_EXTRUDER];
	
	//PID Gain values
	signed short heater_pTerm[MAX_EXTRUDER];
	signed short heater_iTerm[MAX_EXTRUDER];
	signed short heater_dTerm[MAX_EXTRUDER];
	
	signed short heater_slope[MAX_EXTRUDER]; 
	signed short heater_intercept[MAX_EXTRUDER];
	signed short heater_max_pwm[MAX_EXTRUDER];
 
} parameter_struct;


extern parameter_struct pa;

void init_parameters(void);
void FLASH_StoreSettings(void);
void FLASH_PrintSettings(void);
void FLASH_LoadSettings(void);
void FLASH_BootFromROM(void);
void FLASH_BootFromFLASH(void);
void FLASH_Store_to_SD(void);



