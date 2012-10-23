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
#include <string.h>

#include <memories/flash/flashd.h>
#include "init_configuration.h"
#include "parameters.h"
#include "serial.h"

unsigned short calc_crc16(void);

parameter_struct pa;


void init_parameters(void)
{
	unsigned char cnt_c = 0;
	
	// Initialize flash driver
    FLASHD_Initialize(BOARD_MCK);
		
	pa.chk_sum = 0;
	
	char ver[4] = FLASH_VERSION;
	
	for(cnt_c = 0;cnt_c < 3;cnt_c++)
	{
		pa.version[cnt_c] = ver[cnt_c];
	}
	
	pa.version[3] = 0;
	
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
	
	//-------------
	pa.min_software_endstops = _MIN_SOFTWARE_ENDSTOPS;
	pa.max_software_endstops = _MAX_SOFTWARE_ENDSTOPS;

	pa.x_max_length = _X_MAX_LENGTH;
	pa.y_max_length = _Y_MAX_LENGTH;
	pa.z_max_length = _Z_MAX_LENGTH;
	
	pa.disable_x_en = _DISABLE_X_EN;
	pa.disable_y_en = _DISABLE_Y_EN;
	pa.disable_z_en = _DISABLE_Z_EN;
	pa.disable_e_en = _DISABLE_E_EN;

	pa.x_home_dir = X_HOME_DIR;
	pa.y_home_dir = Y_HOME_DIR;
	pa.z_home_dir = Z_HOME_DIR;
	
	//stepper_control.c
	pa.x_endstop_invert = _X_ENDSTOP_INVERT;
	pa.y_endstop_invert = _Y_ENDSTOP_INVERT;
	pa.z_endstop_invert = _Z_ENDSTOP_INVERT;
	
	pa.x_min_endstop_aktiv = X_MIN_ACTIV;
	pa.x_max_endstop_aktiv = X_MAX_ACTIV;
	pa.y_min_endstop_aktiv = Y_MIN_ACTIV;
	pa.y_max_endstop_aktiv = Y_MAX_ACTIV;
	pa.z_min_endstop_aktiv = Z_MIN_ACTIV;
	pa.z_max_endstop_aktiv = Z_MAX_ACTIV;
	
	pa.invert_x_dir = _INVERT_X_DIR;
	pa.invert_y_dir = _INVERT_Y_DIR;
	pa.invert_z_dir = _INVERT_Z_DIR;
	pa.invert_e_dir = _INVERT_E_DIR;
	
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
	
	pa.heater_slope[0] = HEATER_0_SLOPE; 
	pa.heater_intercept[0] = HEATER_0_INTERCEPT;
	pa.heater_max_pwm[0] = HEATER_0_MAX_PWM;
	
	pa.heater_slope[1] = HEATER_1_SLOPE; 
	pa.heater_intercept[1] = HEATER_1_INTERCEPT;
	pa.heater_max_pwm[1] = HEATER_1_MAX_PWM;
	
}

void FLASH_StoreSettings(void) 
{
	unsigned int lastPageAddress;
    volatile unsigned char *pLastPageData;
	unsigned char error;
	unsigned short pa_size = 0;
	unsigned char  *pa_ptr;
	unsigned short cnt_s=0;
	unsigned short w_err = 0;
	unsigned char pageLocked;


	pa_size = sizeof(pa);
	pa_ptr = (unsigned char*)&pa;
	
	//Make CRC16 Checksum before Save
	pa.chk_sum = calc_crc16();
	
	// Performs on last page (to avoid overriding existing program).
    lastPageAddress = AT91C_IFLASH1 + AT91C_IFLASH1_SIZE - AT91C_IFLASH1_PAGE_SIZE;
    pLastPageData = (volatile unsigned char *) lastPageAddress;
		
	// Check that associated region is locked
    pageLocked = FLASHD_IsLocked(lastPageAddress, lastPageAddress + pa_size);
	
    if(pageLocked)
	{
		printf("Page Locked %d\n\r",pageLocked);
		
		error = FLASHD_Unlock(lastPageAddress, lastPageAddress + pa_size, 0, 0);
		if(error)
		{
			printf("Unlock error\n\r");
			usb_printf("Flash Error\r\n");
			return;
		}
		else
		{
			printf("Unlocking page\n\r");
		}
	}
		
		
	//Write Data to flash
	error = FLASHD_Write(lastPageAddress, pa_ptr, pa_size);
	if(error)
	{
		printf("Flash Write error %d\n\r",error);
	}
	
	//Check Data in Flash
	for(cnt_s=0;cnt_s < pa_size;cnt_s++)
	{
		if((*pLastPageData++) != (*pa_ptr++))
		{
			printf("Write err %d\n\r",cnt_s);
			w_err++;
		}
	}
	
	
	if(w_err > 0)
	{
		//Verify Error 
		printf("Flash Error\n\r");
		usb_printf("Flash Error\r\n");
	}
	else
	{
		//Flash content is good, lock the page
		printf("Locking page\n\r");
		error = FLASHD_Lock(lastPageAddress, lastPageAddress + pa_size, 0, 0);
		if(error)
		{
			printf("Lock Error %d\n\r",error);
		}
		else
		{
			printf("Flash write OK\n\r");
			usb_printf("Flash write OK\r\n");
		}
	}
	
	

}

void FLASH_PrintSettings(void) 
{
	usb_printf("Version: %s \r\n",&pa.version[0]);
	usb_printf("Homing Feedrate mm/min \r\n  M207 X%d Y%d Z%d\r\n",(int)pa.homing_feedrate[0],(int)pa.homing_feedrate[1],(int)pa.homing_feedrate[2]);
	usb_printf("Steps per unit:\r\n  M92 X%d Y%d Z%d E%d\r\n",(int)pa.axis_steps_per_unit[0],(int)pa.axis_steps_per_unit[1],(int)pa.axis_steps_per_unit[2],(int)pa.axis_steps_per_unit[3]);
	usb_printf("Maximum feedrates (mm/s):\r\n  M202 X%d Y%d Z%d E%d \r\n",(int)pa.max_feedrate[0],(int)pa.max_feedrate[1],(int)pa.max_feedrate[2],(int)pa.max_feedrate[3]);
	usb_printf("Maximum Acceleration (mm/s2):\r\n  M201 X%d Y%d Z%d E%d\r\n",(int)pa.max_acceleration_units_per_sq_second[0],(int)pa.max_acceleration_units_per_sq_second[1],(int)pa.max_acceleration_units_per_sq_second[2],(int)pa.max_acceleration_units_per_sq_second[3]);
	usb_printf("Acceleration: S=acceleration, T=retract acceleration\r\n  M204 S%d T%d\r\n",(int)pa.move_acceleration,(int)pa.retract_acceleration);
	//max 100 chars ??
	usb_printf("Advanced variables (mm/s): S=Min feedrate, T=Min travel feedrate, X=max xY jerk,  Z=max Z jerk,");
	usb_printf(" E=max E jerk\r\n  M205 S%d T%d X%d Z%d E%d\r\n",(int)pa.minimumfeedrate,(int)pa.mintravelfeedrate,(int)pa.max_xy_jerk,(int)pa.max_z_jerk,(int)pa.max_e_jerk);

	usb_printf("Maximum Area unit:\r\n  M520 X%d Y%d Z%d\r\n",(int)pa.x_max_length,(int)pa.y_max_length,(int)pa.z_max_length);
	usb_printf("Disable axis when unused:\r\n  M521 X%d Y%d Z%d E%d\r\n",pa.disable_x_en,pa.disable_y_en,pa.disable_z_en,pa.disable_e_en);

	usb_printf("Use Software Endstop I=mIn, A=mAx:\r\n  M522 I%d A%d\r\n",pa.min_software_endstops,pa.max_software_endstops);
	
	usb_printf("Minimum Endstop Input:\r\n  M523 X%d Y%d Z%d\r\n",pa.x_min_endstop_aktiv,pa.y_min_endstop_aktiv,pa.z_min_endstop_aktiv);
	usb_printf("Maximum Endstop Input:\r\n  M524 X%d Y%d Z%d\r\n",pa.x_max_endstop_aktiv,pa.y_max_endstop_aktiv,pa.z_max_endstop_aktiv);
	
	usb_printf("Homing Direction:\r\n  M525 X%d Y%d Z%d\r\n",pa.x_home_dir,pa.y_home_dir,pa.z_home_dir);
	usb_printf("Endstop invert:\r\n  M526 X%d Y%d Z%d\r\n",pa.x_endstop_invert,pa.y_endstop_invert,pa.z_endstop_invert);
	usb_printf("Axis invert:\r\n  M510 X%d Y%d Z%d E%d\r\n",pa.invert_x_dir,pa.invert_y_dir,pa.invert_z_dir,pa.invert_e_dir);
	
	usb_printf("Heater 1 PID:\r\n  M301 P%d I%d D%d\r\n",(int)pa.heater_pTerm[0],(int)pa.heater_iTerm[0],(int)pa.heater_dTerm[0]);
	usb_printf("Heater 2 PID:\r\n  M301 P%d I%d D%d\r\n",(int)pa.heater_pTerm[1],(int)pa.heater_iTerm[1],(int)pa.heater_dTerm[1]);
	
	usb_printf("Heater 1 Sensor:\r\n  M530 E%d\r\n",pa.heater_thermistor_type[0]);
	usb_printf("Heater 2 Sensor:\r\n  M530 E%d\r\n",pa.heater_thermistor_type[1]);
	usb_printf("Heatbed Sensor:\r\n  M530 B%d\r\n",pa.bed_thermistor_type);

	usb_printf("Heater 1 PWM: \r\n  M531 E%d\r\n",pa.heater_pwm_en[0]);
	usb_printf("Heater 2 PWM: \r\n  M531 E%d\r\n",pa.heater_pwm_en[1]);
	
	usb_printf("Heater 1 Max pwm: \r\n  M301 W%d\r\n",pa.heater_max_pwm[0]);
	usb_printf("Heater 2 Max pwm: \r\n  M301 W%d\r\n",pa.heater_max_pwm[1]);
	
	usb_printf("Heater 1 (S)lope, (I)ntercept \r\n  M301 S%d I%d\r\n",pa.heater_slope[0],pa.heater_intercept[0]);
	usb_printf("Heater 2 (S)lope, (I)ntercept \r\n  M301 S%d I%d\r\n",pa.heater_slope[0],pa.heater_intercept[0]);
	
	usb_printf("Motor Current \r\n  M907 X%d Y%d Z%d E%d B%d \r\n",pa.axis_current[0],pa.axis_current[1],pa.axis_current[2],pa.axis_current[3],pa.axis_current[4]);
	usb_printf("Motor Microstepping \r\n  M350 X%d Y%d Z%d E%d B%d \r\n",pa.axis_ustep[0],pa.axis_ustep[1],pa.axis_ustep[2],pa.axis_ustep[3],pa.axis_ustep[4]);
	
}

void FLASH_LoadSettings(void) 
{
	
	char ver[4] = FLASH_VERSION;
	unsigned short cnt_s = 0;
	unsigned int lastPageAddress;
    volatile unsigned char *pLastPageData;
	
	unsigned short pa_size = 0;
	volatile unsigned char  *pa_ptr;
	unsigned short help_crc = 0;


	pa_size = sizeof(pa);
	pa_ptr = (volatile unsigned char*)&pa;
	
	// Performs tests on last page (to avoid overriding existing program).
    lastPageAddress = AT91C_IFLASH1 + AT91C_IFLASH1_SIZE - AT91C_IFLASH1_PAGE_SIZE;
    pLastPageData = (volatile unsigned char *) lastPageAddress;
	
	//Read Data from Flash
	for(cnt_s = 0;cnt_s < pa_size;cnt_s++)
	{
		*pa_ptr++ = *pLastPageData++;
		
		/*
		printf("0x%02X ",*pLastPageData);
		if((cnt_s+1)%16 == 0)
			printf("\n\r");
		*/
	}
	
	printf("Read Parameters from flash\n\r");
	
	
	//Check the CRC16
	help_crc = calc_crc16();
	if(pa.chk_sum != help_crc)
	{
		printf("CRC NOT OK\n\r");
		usb_printf("Flash Error CRC  Load Initvalues\r\n");
		//Error --> load init Parameters
		init_parameters();
		return;
		
	}
	
	//Check the Parameter Version
	if(strncmp(ver,pa.version,3)==0)
	{
		printf("VER OK\n\r");
		usb_printf("Load Paramaters from Flash OK\r\n");
	}
	else
	{
		printf("VER NOT OK\n\r");
		usb_printf("Flash Error Version  Load Initvalues\r\n");
		//Error --> load init Parameters
		init_parameters();
	}

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

void FLASH_BootFromROM(void){
    FLASHD_ClearGPNVM(1);
}




