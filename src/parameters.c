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
 
 
#include <board.h>
#include <stdio.h>
#include <stdlib.h>

#include "init_configuration.h"
#include "parameters.h"

unsigned short calc_crc16(void);

parameter_struct pa;


void init_parameters(void)
{
	unsigned char cnt_c = 0;
	
	pa.chk_sum = 0;
	pa.version[0] = 'F';
	pa.version[1] = '0';
	pa.version[2] = '0';
	pa.version[3] = '1';
	
	float f_tmp1[NUM_AXIS] = _MAX_FEEDRATE;
	float f_tmp2[NUM_AXIS] = _AXIS_STEP_PER_UNIT;
	float f_tmp3[3] = _HOMING_FEEDRATE;
	unsigned long ul_tmp1[NUM_AXIS] = _MAX_ACCELERATION_UNITS_PER_SQ_SECOND;
	
	for(cnt_c = 0;cnt_c < 4;cnt_c++)
	{
		pa.max_feedrate[cnt_c] = f_tmp1[cnt_c];
		pa.axis_steps_per_unit[cnt_c] = f_tmp2[cnt_c];
		pa.max_acceleration_units_per_sq_second[cnt_c] = ul_tmp1[cnt_c];
		if(cnt_c < 3)
			pa.homing_feedrate[cnt_c] = f_tmp3[cnt_c];
	}
	
	pa.minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;
	pa.retract_acceleration = _RETRACT_ACCELERATION; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
	pa.max_xy_jerk = _MAX_XY_JERK; //speed than can be stopped at once, if i understand correctly.
	pa.max_z_jerk = _MAX_Z_JERK;
	pa.max_e_jerk = _MAX_E_JERK;
	pa.mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
	pa.move_acceleration = _ACCELERATION;       
	
	pa.min_software_endstops = _MIN_SOFTWARE_ENDSTOPS;
	pa.max_software_endstops = _MAX_SOFTWARE_ENDSTOPS;

	pa.x_max_length = _X_MAX_LENGTH;
	pa.y_max_length = _Y_MAX_LENGTH;
	pa.z_max_length = _Z_MAX_LENGTH;
	
	pa.disable_x_en = _DISABLE_X_EN;
	pa.disable_y_en = _DISABLE_Y_EN;
	pa.disable_z_en = _DISABLE_Z_EN;
	pa.disable_e_en = _DISABLE_E_EN;

	
	unsigned char uc_temp1[5] = _AXIS_CURRENT;
	unsigned char uc_temp2[5] = _AXIS_USTEP;
	
	for(cnt_c = 0;cnt_c < 5;cnt_c++)
	{
		pa.axis_current[cnt_c] = uc_temp1[cnt_c];
		pa.axis_ustep[cnt_c] = uc_temp2[cnt_c];
	}
	
	pa.heater_thermistor_type[0] = THERMISTORHEATER;
	pa.heater_thermistor_type[1] = THERMISTORHEATER;
	pa.bed_thermistor_type = THERMISTORBED;
	
	pa.heater_pwm_en[0] = HEATER_1_PWM;
	pa.heater_pwm_en[1] = HEATER_2_PWM;
	
	pa.heater_pTerm[0] = PID_PGAIN;
	pa.heater_iTerm[0] = PID_IGAIN;
	pa.heater_dTerm[0] = PID_DGAIN;
	
	pa.heater_pTerm[1] = PID_PGAIN;
	pa.heater_iTerm[1] = PID_IGAIN;
	pa.heater_dTerm[1] = PID_DGAIN;

	pa.chk_sum = calc_crc16();
	
	
}




#define CRC16POLY 0x8408
/*
//                                      16   12   5
// this is the CCITT CRC 16 polynomial X  + X  + X  + 1.
// This works out to be 0x1021, but the way the algorithm works
// lets us use 0x8408 (the reverse of the bit pattern).  The high
// bit is always assumed to be set, thus we only use 16 bits to
// represent the 17 bit value.
*/
unsigned short calc_crc16(void)
{
	unsigned short pa_size = 0;
	unsigned char  *pa_ptr;

	unsigned char  cnt_c;
	unsigned short data;
	unsigned short crc = 0xffff;
	
	pa_size = sizeof(pa) - 2;	//skip 2 byte chksum + 131 byte parameter
	
	printf("sizeof pa struct:%d \n\r", pa_size);	//result 131 --> ??? 134
	
	pa_ptr = (unsigned char*)&pa;
	
	pa_ptr+=2; //skip checksum space

	if (pa_size == 0)
		return (~crc);

	do
	{
		for (cnt_c=0, data=(unsigned int)0xff & *pa_ptr++;cnt_c < 8;cnt_c++, data >>= 1)
		{
			if ((crc & 0x0001) ^ (data & 0x0001))
				crc = (crc >> 1) ^ CRC16POLY;
			else  
				crc >>= 1;
		}	
	} while (--pa_size);

	crc = ~crc;
	data = crc;
	crc = (crc << 8) | (data >> 8 & 0xff);
	
	printf("CRC16: 0x%04X \n\r", crc);
	
	return (crc);
	
	
	
}






