/*
 heater funtions for Hotend and Heatbed
 
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
#include <stdio.h>
#include <stdlib.h>
#include "samadc.h"
#include "heaters.h"
#include "thermistortables.h"

#define HEATER_BED			0
#define HEATER_HOTEND_1		1
#define HEATER_HOTEND_2		2



const Pin BEDHEAT={1 <<  20, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin HOTEND1={1 <<  21, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin HOTEND2={1 <<  23, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin AUX1={1 <<  25, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin AUX2={1 <<  24, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};


//Global struct for Heatercontrol
heater_struct heaters[2];
heater_bed_struct bed_heater;

//-------------------------
// SETUP HEATERS IO
//-------------------------
void heaters_setup(){
    Pin FETPINS[]={BEDHEAT,HOTEND1,HOTEND2,AUX1,AUX2};
    PIO_Configure(FETPINS,5);
    int i;
    for(i=0;i<5;++i)
        PIO_Clear(&(FETPINS[i]));
		
	init_heaters_values();	
		
}

//-------------------------
// IO Function
//-------------------------
void heater_switch(unsigned char heater, unsigned char en){
    Pin FETPINS[]={BEDHEAT,HOTEND1,HOTEND2,AUX1,AUX2};
    if(heater<0||heater>5)
        return;
    if(en)
        PIO_Set(&(FETPINS[heater]));
    else
        PIO_Clear(&(FETPINS[heater]));
}


//-------------------------
// Convert °C to mV
//-------------------------
#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int temp2analog_thermistor(int celsius, const short table[][2], int numtemps) 
{
    int raw = 0;
    unsigned char i;
    
    for (i=1; i<numtemps; i++)
    {
      if (table[i][1] < celsius)
      {
        raw = table[i-1][0] + 
          (celsius - table[i-1][1]) * 
          (table[i][0] - table[i-1][0]) /
          (table[i][1] - table[i-1][1]);
      
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == numtemps) raw = table[i-1][0];

    return /*3300 -*/ raw;
}
#endif

//-------------------------
// Convert mV to °C
//-------------------------
#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int analog2temp_thermistor(int raw,const short table[][2], int numtemps) {
    int celsius = 0;
    unsigned char i;
    
    //raw = 3300 - raw;

    for (i=1; i<numtemps; i++)
    {
      if (table[i][0] > raw)
      {
        celsius  = table[i-1][1] + 
          (raw - table[i-1][0]) * 
          (table[i][1] - table[i-1][1]) /
          (table[i][0] - table[i-1][0]);

        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == numtemps) celsius = table[i-1][1];

    return celsius;
}
#endif

//-------------------------
// Init heater Values
//-------------------------
void init_heaters_values(void)
{
	
	heaters[0].io_adr = HEATER_HOTEND_1;
	heaters[0].ad_cannel = 3;
	heaters[0].pwm = 0;
	heaters[0].soft_pwm_aktiv = 1;
	heaters[0].PID_Kp = PID_PGAIN;
	heaters[0].PID_I = PID_IGAIN;
	heaters[0].PID_Kd = PID_DGAIN;
	heaters[0].temp_iState = 0;
	heaters[0].prev_temp = 0;
	heaters[0].temp_iState_max = (256L * PID_INTEGRAL_DRIVE_MAX) / (int)heaters[0].PID_I;
	heaters[0].temp_iState_min = heaters[0].temp_iState_max * (-1);
	

	heaters[1].io_adr = HEATER_HOTEND_2;
	heaters[1].ad_cannel = 1;
	heaters[1].pwm = 0;
	heaters[1].soft_pwm_aktiv = 0;
	heaters[1].PID_Kp = PID_PGAIN;
	heaters[1].PID_I = PID_IGAIN;
	heaters[1].PID_Kd = PID_DGAIN;
	heaters[1].temp_iState = 0;
	heaters[1].prev_temp = 0;
	heaters[1].temp_iState_max = (256L * PID_INTEGRAL_DRIVE_MAX) / (int)heaters[1].PID_I;
	heaters[1].temp_iState_min = heaters[1].temp_iState_max * (-1);
	
	
	
}

//--------------------------------------------------
// Simple Hotend Tempcontrol with ON/OFF switching
//--------------------------------------------------
void onoff_control_hotend(heater_struct *hotend)
{
	hotend->akt_temp = analog2temp(adc_read(hotend->ad_cannel));
	
	if(hotend->akt_temp < 4)
	{
		hotend->target_temp = 0;
		heater_switch(hotend->io_adr, 0);
	}
		
	if(hotend->akt_temp  > (hotend->target_temp+1))
	{
		heater_switch(hotend->io_adr, 0);
		hotend->pwm = 0;
	}
	else if((hotend->akt_temp  < (hotend->target_temp-1)) && (hotend->target_temp > 0))
	{
		heater_switch(hotend->io_adr, 1);
		hotend->pwm = 255;
	
	}
	
}

void heater_soft_pwm(void)
{
	static unsigned char pwm_cnt = 0;
	
	pwm_cnt++;
	
	if(heaters[0].soft_pwm_aktiv == 1)
	{
		if(heaters[0].pwm == 0)
			heater_switch(heaters[0].io_adr, 0);
		else if(heaters[0].pwm == 255)
			heater_switch(heaters[0].io_adr, 1);
		else
		{
			if(pwm_cnt == 0)
				heater_switch(heaters[0].io_adr, 1);
			else if(pwm_cnt >= heaters[0].pwm)
				heater_switch(heaters[0].io_adr, 0);
		}
	}

}

//--------------------------------------------------
// Tempcontrol with PID for Hotend
//--------------------------------------------------
void heater_PID_control(heater_struct *hotend)
{
	int error;
	int delta_temp;
	int heater_duty;
  
	hotend->akt_temp = analog2temp(adc_read(hotend->ad_cannel));
  
	//MIN save to display the jitter of Heaterbarrel
	#ifdef MINTEMP
	if(hotend->akt_temp < 4)
	{
		hotend->target_temp = 0;
		heater_switch(hotend->io_adr, 0);
	}
	#endif
	
	//MAX save to display the jitter of Heaterbarrel
	#ifdef MAXTEMP
	if(hotend->akt_temp > 300)
	{
		hotend->target_temp = 0;
		heater_switch(hotend->io_adr, 0);
	}
	#endif

	error = hotend->target_temp - hotend->akt_temp;
	printf("ERR: %d ", error);
	delta_temp = hotend->akt_temp - hotend->prev_temp;

	hotend->prev_temp = hotend->akt_temp;
	hotend->pTerm = (int)(((long)hotend->PID_Kp * error) / 256);
	
	const int H0 = min(HEATER_DUTY_FOR_SETPOINT(hotend->target_temp),HEATER_CURRENT);
	heater_duty = H0 + hotend->pTerm;

	if(abs(error) < 30)
	{
		hotend->temp_iState += error;
		printf("I1: %d ", hotend->temp_iState);
		hotend->temp_iState = constrain(hotend->temp_iState, hotend->temp_iState_min, hotend->temp_iState_max);
		printf("I2: %d ", hotend->temp_iState);
		hotend->iTerm = (int)(((long)hotend->PID_I * hotend->temp_iState) / 256);
		heater_duty += hotend->iTerm;
	}
	
	printf("I: %d ", hotend->iTerm);

	int prev_error = abs(hotend->target_temp - hotend->prev_temp);
	int log3 = 1; // discrete logarithm base 3, plus 1

	if(prev_error > 81){ prev_error /= 81; log3 += 4; }
	if(prev_error >  9){ prev_error /=  9; log3 += 2; }
	if(prev_error >  3){ prev_error /=  3; log3 ++;   }

	hotend->dTerm = (int)(((long)hotend->PID_Kd * delta_temp) / (256*log3));
	
	printf("D: %d ", hotend->dTerm);
	
	heater_duty += hotend->dTerm;
	printf("PWM: %d \n", heater_duty);
	
	heater_duty = constrain(heater_duty, 0, HEATER_CURRENT);
	


	if(hotend->target_temp == 0)
		hotend->pwm = 0;
	else
		hotend->pwm = (unsigned char)heater_duty;


}

//--------------------------------------------------
// Simple Heatbed Tempcontrol with ON/OFF switching
//--------------------------------------------------
void onoff_control_bed(void)
{
	
	bed_heater.akt_temp = analog2tempBed(adc_read(5));
	
	if(bed_heater.akt_temp < 4)
	{
		bed_heater.target_temp = 0;
		heater_switch(HEATER_BED, 0);
	}
			
	if(bed_heater.akt_temp  > bed_heater.target_temp)
	{
		heater_switch(HEATER_BED, 0);
	}
	else if((bed_heater.akt_temp  < bed_heater.target_temp) && (bed_heater.target_temp > 0))
	{
		heater_switch(HEATER_BED, 1);
	}
}

//--------------------------------------------------
// Cycle Function for Tempcontrol
//--------------------------------------------------
void manage_heaters(void)
{
	static unsigned char bed_timer = 0;
	static unsigned char hotend_timer = 0;
	
	bed_timer++;
	
	if(bed_timer >= 20)
	{
		//Call every 2 sec
		onoff_control_bed();
		bed_timer = 0;
	}
	
	if(hotend_timer == 0)
	{
		//Call every 200 ms
		//onoff_control_hotend(&heaters[0]);
		heater_PID_control(&heaters[0]);
		hotend_timer = 1;
	}
	else if(hotend_timer == 1)
	{
		onoff_control_hotend(&heaters[1]);
		hotend_timer = 0;
	}
}
