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
#include <irq/irq.h>
#include <tc/tc.h>
#include <math.h>

#include "init_configuration.h"
#include "parameters.h"
#include "samadc.h"
#include "heaters.h"
#include "thermistortables.h"
#include "serial.h"

#define HEATER_BED			0
#define HEATER_HOTEND_1		1
#define HEATER_HOTEND_2		2

const Pin BEDHEAT={1 <<  20, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin HOTEND1={1 <<  21, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin HOTEND2={1 <<  23, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin AUX1={1 <<  25, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};
const Pin AUX2={1 <<  24, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_PULLUP};

/// LED pin definition.
const Pin PIN_LED1 = {1 << 22, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin PIN_LED2 = {1 << 29, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin PIN_LED3 = {1 << 28, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin PIN_LED4 = {1 << 2 , AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin PIN_LED5 = {1 << 1 , AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin PIN_LED6 = {1 << 0 , AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin PIN_LED7 = {1 << 26, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin PIN_LED8 = {1 << 20, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT};
const Pin PIN_LED9 = {1 << 0 , AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT};


extern const Pin time_check2;
extern volatile unsigned long timestamp;

unsigned char autotune_active = false;

//Global struct for Heatercontrol
heater_struct heaters[2];	//MAX_EXTRUDERS ?
heater_bed_struct bed_heater;

//-----------------------------------------------------
/// SOFT Pwm for Heater 1 & 2 and Ext Pwm 1 & 2 like Fan
//-----------------------------------------------------
volatile unsigned char g_pwm_value[4] = {0,0,0,0};
volatile unsigned char g_pwm_io_adr[4] = {1,2,3,4};
volatile unsigned char g_pwm_aktiv[4] = {0,0,0,0};


//-------------------------
// SETUP HEATERS IO
//-------------------------
void heaters_setup()
{
	Pin FETPINS[]={BEDHEAT,HOTEND1,HOTEND2,AUX1,AUX2};
	Pin LEDPINS[]={PIN_LED1,PIN_LED2,PIN_LED3,PIN_LED4,PIN_LED5,PIN_LED6,PIN_LED7,PIN_LED8,PIN_LED9};

	PIO_Configure(FETPINS,5);
	PIO_Configure(LEDPINS,9);

	unsigned short i;
	
	for(i=0;i<5;++i)
		PIO_Clear(&(FETPINS[i]));

	for(i=0;i<9;++i)
		PIO_Clear(&(LEDPINS[i]));
		
	
	init_heaters_values();	

}

//-------------------------
// IO Function for FET's
//-------------------------
void heater_switch(unsigned char heater, unsigned char en)
{
	Pin FETPINS[]={BEDHEAT,HOTEND1,HOTEND2,AUX1,AUX2};

	if(heater<0||heater>5)
		return;
	
	if(en)
		PIO_Set(&(FETPINS[heater]));
	else
		PIO_Clear(&(FETPINS[heater]));
}

//-------------------------
// IO Function for LED's
//-------------------------
void LED_switch(unsigned char led, unsigned char en)
{
	Pin LEDPINS[]={PIN_LED1,PIN_LED2,PIN_LED3,PIN_LED4,PIN_LED5,PIN_LED6,PIN_LED7,PIN_LED8,PIN_LED9};

	if(led<0||led>9)
		return;
	
	if(en)
		PIO_Set(&(LEDPINS[led]));
	else
		PIO_Clear(&(LEDPINS[led]));
}


//--------------------------------------------
// Convert �C to mV with Compute function
//--------------------------------------------
signed short temp2analog_thermistor_compute(signed short celsius, const float beta, const float rs, const float r_inf)
{
	float r = r_inf*exp(beta/(celsius - ABS_ZERO));
	return (signed short)(0.5 + ADC_VREF*r/(r + rs));
}

//--------------------------------------------------
// Convert temperatur to Analog with Tablefunction
//--------------------------------------------------
signed short temp2analog_thermistor_table(signed short celsius, const short table[][2], signed short numtemps)
{
	signed short raw = 0;
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

	return raw;
}


//--------------------------------------------
// Convert mV to �C with Compute function
//---------------------------------------------
signed short analog2temp_thermistor_compute(signed short raw, const float beta, const float rs, const float r_inf)
{
	//Support to compute temperature  from themistor Beta
	//Thanks to bilsef
	signed short celsius = 0; 
	
	if ((raw < 0) || (raw >= ADC_VREF)) 
		return (0);    // return if value is out of range 

	float r = rs/((ADC_VREF/(float)(raw))-1); 

	celsius = 0.5 + ABS_ZERO + beta/log( r/r_inf ); 

	if (celsius < 0) 
		celsius = 0; 

	return celsius; 

	
}

//--------------------------------------------------
// Convert Analog to temperatur with Tablefunction
//--------------------------------------------------
signed short analog2temp_thermistor_table(signed short raw,const short table[][2], signed short numtemps)
{
	signed short celsius = 0;
	unsigned char i;

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


//--------------------------------------------------
// Convert fron Analog to Temperatur , Sensortype can select online
//--------------------------------------------------
signed short analog2temp_convert(signed short raw, unsigned char sensortype)
{
	signed short temperature = 0;

	switch(sensortype)
	{
		case THERMISTORTYP_TABLE_1:
			temperature = analog2temp_thermistor_table(raw,temptable_1,NUMTEMPS_1);
		break;
		
		case THERMISTORTYP_TABLE_2:
			temperature = analog2temp_thermistor_table(raw,temptable_2,NUMTEMPS_2);
		break;
		
		case THERMISTORTYP_TABLE_3:
			temperature = analog2temp_thermistor_table(raw,temptable_3,NUMTEMPS_3);
		break;
		
		case THERMISTORTYP_TABLE_4:
			temperature = analog2temp_thermistor_table(raw,temptable_4,NUMTEMPS_4);
		break;
		
		case THERMISTORTYP_TABLE_5:
			temperature = analog2temp_thermistor_table(raw,temptable_5,NUMTEMPS_5);
		break;
		
		case THERMISTORTYP_TABLE_6:
			temperature = analog2temp_thermistor_table(raw,temptable_6,NUMTEMPS_6);
		break;
		
		case THERMISTORTYP_TABLE_7:
			temperature = analog2temp_thermistor_table(raw,temptable_7,NUMTEMPS_7);
		break;
		
		//Calclate Temperatur with formular
		case THERMISTORTYP_COMPUTE_11:
			temperature = analog2temp_thermistor_compute(raw, E_BETA_11, E_RS, E_R_INF_11);
		break;

		case THERMISTORTYP_COMPUTE_12:
			temperature = 0;
		break;

		case THERMISTORTYP_COMPUTE_13:
			temperature = analog2temp_thermistor_compute(raw, E_BETA_13, E_RS, E_R_INF_13);
		break;

		case THERMISTORTYP_COMPUTE_14:
			temperature = analog2temp_thermistor_compute(raw, E_BETA_14, E_RS, E_R_INF_14);
		break;

		case THERMISTORTYP_COMPUTE_15:
			temperature = analog2temp_thermistor_compute(raw, E_BETA_15, E_RS, E_R_INF_15);
		break;

		case THERMISTORTYP_COMPUTE_16:
			temperature = analog2temp_thermistor_compute(raw, E_BETA_16, E_RS, E_R_INF_16);
		break;

		case THERMISTORTYP_COMPUTE_17:
			temperature = analog2temp_thermistor_compute(raw, E_BETA_17, E_RS, E_R_INF_17);
		break;

	
		case AD595_TYP_50:
			temperature = (signed short)((int)raw * 500 / ADC_VREF);
		break;
	
		default:
			temperature = 0;
		break;
	}
	
	return(temperature);

}


//-------------------------
// Init heater Values
//-------------------------
void init_heaters_values(void)
{
	
	heaters[0].io_adr = HEATER_HOTEND_1;
	g_pwm_io_adr[0] = HEATER_HOTEND_1;
	heaters[0].ad_cannel = 3;
	heaters[0].pwm = 0;
	heaters[0].soft_pwm_aktiv = pa.heater_pwm_en[0];
	heaters[0].PID_Kp = pa.heater_pTerm[0];
	heaters[0].PID_I = pa.heater_iTerm[0];
	heaters[0].PID_Kd = pa.heater_dTerm[0];
	heaters[0].temp_iState = 0;
	heaters[0].prev_temp = 0;
	heaters[0].temp_iState_max = (256L * PID_INTEGRAL_DRIVE_MAX) / (signed short)heaters[0].PID_I;
	heaters[0].temp_iState_min = heaters[0].temp_iState_max * (-1);
	heaters[0].thermistor_type = pa.heater_thermistor_type[0];
	heaters[0].slope = pa.heater_slope[0];
	heaters[0].intercept = pa.heater_intercept[0];
	heaters[0].max_pwm = pa.heater_max_pwm[0];

	heaters[1].io_adr = HEATER_HOTEND_2;
	g_pwm_io_adr[1] = HEATER_HOTEND_2;
	heaters[1].ad_cannel = 1;
	heaters[1].pwm = 0;
	heaters[1].soft_pwm_aktiv = pa.heater_pwm_en[1];
	heaters[1].PID_Kp = pa.heater_pTerm[1];
	heaters[1].PID_I = pa.heater_iTerm[1];
	heaters[1].PID_Kd = pa.heater_dTerm[1];
	heaters[1].temp_iState = 0;
	heaters[1].prev_temp = 0;
	heaters[1].temp_iState_max = (256L * PID_INTEGRAL_DRIVE_MAX) / (signed short)heaters[1].PID_I;
	heaters[1].temp_iState_min = heaters[1].temp_iState_max * (-1);
	heaters[1].thermistor_type = pa.heater_thermistor_type[1];
	heaters[1].slope = pa.heater_slope[1];
	heaters[1].intercept = pa.heater_intercept[1];
	heaters[1].max_pwm = pa.heater_max_pwm[1];
	
	bed_heater.target_temp = 0;
	bed_heater.akt_temp = 0;
	bed_heater.thermistor_type = pa.bed_thermistor_type;
	
	
}



//--------------------------------------------------
// Soft PWM, runs every 100 us
// need 2,12 us  + 0,3 us per PWM channel
//--------------------------------------------------
volatile unsigned char g_TC1_pwm_cnt = 0;
volatile unsigned char pwm_io_is_off[4] = {0,0,0,0};
void TC1_IrqHandler(void)
{

	volatile unsigned int dummy;
	unsigned char cnt_pwm_ch = 0;
	
    // Clear status bit to acknowledge interrupt !!
	// Dont forget --> other interupts are blocked until the bit is cleared
    dummy = AT91C_BASE_TC1->TC_SR;
	
	if(dummy & AT91C_TC_CPCS)
	{
	}
	
	PIO_Set(&time_check2);
	
	g_TC1_pwm_cnt+=2;
	
	//Check the 4 PWM channels
	for(cnt_pwm_ch = 0;cnt_pwm_ch < 4;cnt_pwm_ch++)
	{
		if(g_pwm_aktiv[cnt_pwm_ch] == 1)
		{
			if(g_TC1_pwm_cnt == 0)
			{
				if(g_pwm_value[cnt_pwm_ch] == 0)
				{
					heater_switch(g_pwm_io_adr[cnt_pwm_ch], 0);
					pwm_io_is_off[cnt_pwm_ch] = 1;
				}
				else
				{
					heater_switch(g_pwm_io_adr[cnt_pwm_ch], 1);
					pwm_io_is_off[cnt_pwm_ch] = 0;
				}
			}
			else
			{
				if((g_TC1_pwm_cnt >= g_pwm_value[cnt_pwm_ch]) && (pwm_io_is_off[cnt_pwm_ch] == 0))
				{
					heater_switch(g_pwm_io_adr[cnt_pwm_ch], 0);
					pwm_io_is_off[cnt_pwm_ch] = 1;
				}
			}
		}
	}
	
	PIO_Clear(&time_check2);
}

//--------------------------------------------------
// Simple Hotend Tempcontrol with ON/OFF switching
//--------------------------------------------------
void heater_on_off_control(heater_struct *hotend)
{
	hotend->akt_temp = analog2temp_convert(adc_read(hotend->ad_cannel),hotend->thermistor_type);
	
	#ifdef MINTEMP
	if(hotend->akt_temp < MINTEMP)
	{
		hotend->target_temp = 0;
		heater_switch(hotend->io_adr, 0);
		hotend->pwm = 0;
	}
	#endif
		
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


//--------------------------------------------------
// Tempcontrol with PID for Hotend
//--------------------------------------------------
void heater_PID_control(heater_struct *hotend)
{
	signed short error;
	signed short delta_temp;
	signed short heater_duty;
  
	hotend->akt_temp = analog2temp_convert(adc_read(hotend->ad_cannel),hotend->thermistor_type);
  
	#ifdef MINTEMP
	if(hotend->akt_temp < MINTEMP)
	{
		hotend->target_temp = 0;
		heater_switch(hotend->io_adr, 0);
	}
	#endif
	
	#ifdef MAXTEMP
	if(hotend->akt_temp > MAXTEMP)
	{
		hotend->target_temp = 0;
		heater_switch(hotend->io_adr, 0);
	}
	#endif

	error = hotend->target_temp - hotend->akt_temp;
	//printf("ERR: %d ", error);
	delta_temp = hotend->akt_temp - hotend->prev_temp;

	hotend->prev_temp = hotend->akt_temp;
	hotend->pTerm = (signed short)(((long)hotend->PID_Kp * error) / 256);
	
  const signed short H0 = min(((((long)hotend->slope*(long)hotend->target_temp)>>8)+hotend->intercept),hotend->max_pwm);
	heater_duty = H0 + hotend->pTerm;

	//printf("P: %d ", hotend->pTerm);

	if(abs(error) < 30)
	{
		hotend->temp_iState += error;
		//printf("I1: %d ", hotend->temp_iState);
		hotend->temp_iState = constrain(hotend->temp_iState, hotend->temp_iState_min, hotend->temp_iState_max);
		//printf("I2: %d ", hotend->temp_iState);
		hotend->iTerm = (signed short)(((long)hotend->PID_I * hotend->temp_iState) / 256);
		heater_duty += hotend->iTerm;
	}
	
	//printf("I: %d ", hotend->iTerm);

	signed short prev_error = abs(hotend->target_temp - hotend->prev_temp);
	signed short log3 = 1; // discrete logarithm base 3, plus 1

	if(prev_error > 81){ prev_error /= 81; log3 += 4; }
	if(prev_error >  9){ prev_error /=  9; log3 += 2; }
	if(prev_error >  3){ prev_error /=  3; log3 ++;   }

	hotend->dTerm = (signed short)(((long)hotend->PID_Kd * delta_temp) / (256*log3));
	
	//printf("D: %d ", hotend->dTerm);
	
	heater_duty += hotend->dTerm;
	//printf("PWM: %d \r\n", heater_duty);
	
	heater_duty = constrain(heater_duty, 0, hotend->max_pwm);
	


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
	
	bed_heater.akt_temp = analog2temp_convert(adc_read(5),bed_heater.thermistor_type);
	
	#ifdef MINTEMP
	if(bed_heater.akt_temp < MINTEMP)
	{
		bed_heater.target_temp = 0;
		heater_switch(HEATER_BED, 0);
		LED_switch(3,0);
	}
	#endif
			
	if(bed_heater.akt_temp  > bed_heater.target_temp)
	{
		heater_switch(HEATER_BED, 0);
		LED_switch(3,0);
	}
	else if((bed_heater.akt_temp  < bed_heater.target_temp) && (bed_heater.target_temp > 0))
	{
		heater_switch(HEATER_BED, 1);
		LED_switch(3,1);
	}
}

//--------------------------------------------------
// Init Timer 1 for Soft PWM (Hotend 1 & 2)
//--------------------------------------------------
void ConfigureTc_1(void)
{

	// Enable peripheral clock
	AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC1;
	
	// Configure TC for a 10 kHz frequency and trigger on RC compare
	unsigned int freq=10000; 

	TC_Configure(AT91C_BASE_TC1, 3 | AT91C_TC_CPCTRG);
	//AT91C_BASE_TC1->TC_RB = 3;
	AT91C_BASE_TC1->TC_RC = (BOARD_MCK / 128) / freq; // timerFreq / desiredFreq

	// Configure and enable interrupt on RC compare
	IRQ_ConfigureIT(AT91C_ID_TC1, 2, TC1_IrqHandler);
	AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS;
	IRQ_EnableIT(AT91C_ID_TC1);

	// Start the counter if LED is enabled.
	TC_Start(AT91C_BASE_TC1);

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
		//Call every 500 ms
		//heater_on_off_control(&heaters[0]);
    if(!autotune_active)
		  heater_PID_control(&heaters[0]);
		g_pwm_value[0] = heaters[0].pwm;
		g_pwm_io_adr[0] = heaters[0].io_adr;
		g_pwm_aktiv[0] = heaters[0].soft_pwm_aktiv;
		
		if(g_pwm_value[0] > 0)
			LED_switch(4,1);
		else
			LED_switch(4,0);
			
		hotend_timer = 1;
	}
	else if(hotend_timer == 1)
	{
		heater_on_off_control(&heaters[1]);
    //if(!autotune_active)
		//  heater_PID_control(&heaters[1]);
		g_pwm_value[1] = heaters[1].pwm;
		g_pwm_io_adr[1] = heaters[1].io_adr;
		g_pwm_aktiv[1] = heaters[1].soft_pwm_aktiv;
		
		if(g_pwm_value[1] > 0)
			LED_switch(5,1);
		else
			LED_switch(5,0);
		
		hotend_timer = 0;
	}
}

//-------------------- START PID AUTOTUNE ---------------------------
// Based on PID relay test 
// Thanks to Erik van der Zalm for this idea to use it for Marlin
// Some information see:
// http://brettbeauregard.com/blog/2012/01/arduino-pid-autotune-library/
//------------------------------------------------------------------

void PID_autotune(heater_struct *hotend, float PIDAT_test_temp)
{
  float PIDAT_input = (float)hotend->akt_temp;
  float PIDAT_input_ave = 0;
  unsigned char PIDAT_count_input = 0;

  float PIDAT_max = 0.0;
  float PIDAT_min = 250.0;
 
  unsigned char PIDAT_PWM_val = hotend->max_pwm;
  
  unsigned char PIDAT_cycles = 0;
  unsigned char PIDAT_heating = true;

  unsigned long PIDAT_temp_millis = timestamp;
  unsigned long PIDAT_t1 = PIDAT_temp_millis;
  unsigned long PIDAT_t2 = PIDAT_temp_millis;
  unsigned long PIDAT_T_check_AI_val = PIDAT_temp_millis;

  
  long PIDAT_t_high = 0;
  long PIDAT_t_low = 0;

  long PIDAT_bias = hotend->max_pwm/2;  
  long PIDAT_d  =  hotend->max_pwm/2;
  
  float PIDAT_Ku = 0, PIDAT_Tu = 0;
  float PIDAT_Kp = 0, PIDAT_Ki = 0, PIDAT_Kd = 0;
  
  #define PIDAT_TIME_FACTOR ((HEATER_CHECK_INTERVAL * 256) / 1000)
  
  usb_printf("PID Autotune start\r\n");
  printf("PID Autotune channel %u\r\n",hotend->ad_cannel);

  autotune_active = true;  // disable PID while tuning

  PIDAT_min = PIDAT_test_temp;

  hotend->target_temp = (signed short)PIDAT_test_temp;
  hotend->pwm = (unsigned char)PIDAT_PWM_val;
  
  #ifdef BED_USES_THERMISTOR
    bed_heater.target_temp = 0;
    heater_switch(HEATER_BED, 0);
    LED_switch(3,0);
  #endif
  
  for(;;) 
  {
     // Average 10 readings
    if((timestamp - PIDAT_T_check_AI_val) > 100 )
    {
      PIDAT_T_check_AI_val = timestamp;
      
      PIDAT_input_ave += analog2temp_convert(adc_read(hotend->ad_cannel),hotend->thermistor_type);

      PIDAT_count_input++;
    }
    
    if(PIDAT_count_input >= 10 )
    {
      PIDAT_input = (float)PIDAT_input_ave / (float)PIDAT_count_input;
      PIDAT_input_ave = 0;
      PIDAT_count_input = 0;
      
      PIDAT_max=max(PIDAT_max,PIDAT_input);
      PIDAT_min=min(PIDAT_min,PIDAT_input);

      if(PIDAT_heating == true && PIDAT_input > PIDAT_test_temp) 
      {
        if(timestamp - PIDAT_t2 > 5000) 
        { 
          PIDAT_heating = false;
          PIDAT_PWM_val = (PIDAT_bias - PIDAT_d);
          PIDAT_t1 = timestamp;
          PIDAT_t_high = PIDAT_t1 - PIDAT_t2;
          PIDAT_max = PIDAT_test_temp;
        }
      }
      
      if((PIDAT_heating == false) && (PIDAT_input < PIDAT_test_temp)) 
      {
        if(timestamp - PIDAT_t1 > 5000) 
        {
          PIDAT_heating = true;
          PIDAT_t2 = timestamp;
          PIDAT_t_low = PIDAT_t2 - PIDAT_t1;
          
          if(PIDAT_cycles > 0) 
          {
            PIDAT_bias += (PIDAT_d*(PIDAT_t_high - PIDAT_t_low))/(PIDAT_t_low + PIDAT_t_high);
            PIDAT_bias = constrain(PIDAT_bias, 20 ,hotend->max_pwm - 20);
            if(PIDAT_bias > (hotend->max_pwm/2))
            {
              PIDAT_d = (hotend->max_pwm - 1) - PIDAT_bias;
            } else {
              PIDAT_d = PIDAT_bias;
            }
            usb_printf(" bias: %d  d: %d  min: %d  max: %d \r\n",(int)PIDAT_bias,(int)PIDAT_d,(int)PIDAT_min,(int)PIDAT_max);
            
            if(PIDAT_cycles > 2) 
            {
              PIDAT_Ku = (4.0*PIDAT_d)/(3.14159*(PIDAT_max-PIDAT_min));
              PIDAT_Tu = ((float)(PIDAT_t_low + PIDAT_t_high)/1000.0);
              
              usb_printf(" Ku: %d  Tu: %d \r\n",(int)PIDAT_Ku,(int)PIDAT_Tu);

            // reference http://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method

              PIDAT_Kp = 0.60*PIDAT_Ku;
              PIDAT_Ki = 2*PIDAT_Kp/PIDAT_Tu;
              PIDAT_Kd = PIDAT_Kp*PIDAT_Tu/8;

              printf(" P:%u, I:%u, D:%u\r\n",(unsigned)(PIDAT_Kp*1000),(unsigned)(PIDAT_Ki*1000),(unsigned)(PIDAT_Kd*1000));
              usb_printf(" Clasic PID \r\n  CFG Kp: %u \r\n  CFG Ki: %u \r\n  CFG Kd: %u \r\n", (unsigned int)(PIDAT_Kp*256),(unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR),(unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));
              usb_printf("  Set with M301 P%u I%u D%u\r\n", (unsigned int)(PIDAT_Kp*256),(unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR),(unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));

              PIDAT_Kp = 0.33*PIDAT_Ku;
              PIDAT_Ki = 2*PIDAT_Kp/PIDAT_Tu;
              PIDAT_Kd = PIDAT_Kp*PIDAT_Tu/3;

              usb_printf(" Some overshoot \r\n  CFG Kp: %u \r\n  CFG Ki: %u \r\n  CFG Kd: %u \r\n",(unsigned int)(PIDAT_Kp*256),(unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR),(unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));
              usb_printf("  Set with M301 P%u I%u D%u\r\n", (unsigned int)(PIDAT_Kp*256),(unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR),(unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));

              PIDAT_Kp = 0.20*PIDAT_Ku;
              PIDAT_Ki = 2*PIDAT_Kp/PIDAT_Tu;
              PIDAT_Kd = PIDAT_Kp*PIDAT_Tu/3;

              usb_printf(" No overshoot \r\n  CFG Kp: %u \r\n  CFG Ki: %u \r\n  CFG Kd: %u \r\n",(unsigned int)(PIDAT_Kp*256),(unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR),(unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));
              usb_printf("  Set with M301 P%u I%u D%u\r\n", (unsigned int)(PIDAT_Kp*256),(unsigned int)(PIDAT_Ki*PIDAT_TIME_FACTOR),(unsigned int)(PIDAT_Kd*PIDAT_TIME_FACTOR));

            }
          }
          PIDAT_PWM_val = (PIDAT_bias + PIDAT_d);
          PIDAT_cycles++;
          PIDAT_min = PIDAT_test_temp;
        }
      } 
      
    PIDAT_PWM_val = constrain(PIDAT_PWM_val, 0, hotend->max_pwm);
    hotend->pwm = (unsigned char)PIDAT_PWM_val;
    }

    if((PIDAT_input < (10)) || (PIDAT_input > 255))
    {
      usb_printf("PID Autotune failed! Double check thermistor connection \r\n");
      hotend->target_temp = 0;
      hotend->pwm = 0;
      autotune_active = false;
      return;
    }

    if((PIDAT_input > (PIDAT_test_temp + 55)) || (PIDAT_input > 255))
    {
      usb_printf("PID Autotune failed! Temperature too high \r\n");
      hotend->target_temp = 0;
      hotend->pwm = 0;
      autotune_active = false;
      return;
    }
    
    if(timestamp - PIDAT_temp_millis > 2000) 
    {
      PIDAT_temp_millis = timestamp;
      usb_printf("ok T: %u  @:%u \r\n",(unsigned char)PIDAT_input,(unsigned char)PIDAT_PWM_val);       
    }
    
    if(((timestamp - PIDAT_t1) + (timestamp - PIDAT_t2)) > (10L*60L*1000L*2L)) 
    {
      usb_printf("PID Autotune failed! timeout \r\n");
      hotend->target_temp = 0;
      hotend->pwm = 0;
      autotune_active = false;
      return;
    }
    
    if(PIDAT_cycles > 5) 
    {
      usb_printf("PID Autotune finished! Set new values with M301 \r\n");
      hotend->target_temp = 0;
      hotend->pwm = 0;
	    autotune_active = false;
      return;
    }
  }
}
//---------------- END AUTOTUNE PID ------------------------------


//-------------------- EVALUATE HEATER -----------------------------
//
// Calculate slope and y-intercept for setpoint pwm formula.
// Setting these constants correctly will greatly improve PID performance.
//
//------------------------------------------------------------------

void Heater_Eval(heater_struct *hotend, unsigned int step)
{
  unsigned long temp_millis = timestamp;
  unsigned long T_check;
  unsigned int pwm;
  float input = 25;
  float input_ave;
  unsigned int points[2][20];
  unsigned int count = 0;

#define X 0
#define Y 1

  usb_printf("Find equation of temperature to heater pwm \r\n\n");

  autotune_active = true;  // disable PID while running

  #ifdef BED_USES_THERMISTOR
    bed_heater.target_temp = 0;
    heater_switch(HEATER_BED, 0);
    LED_switch(3,0);
  #endif

  for(pwm = step; pwm < hotend->max_pwm; pwm+=step) 
  {
    hotend->pwm = pwm;
    input_ave = 0;
    T_check = timestamp;
    while((unsigned int)input != (unsigned int)input_ave)
    {
      if((timestamp - T_check) > 500 )
      {
        T_check = timestamp;
        input = analog2temp_convert(adc_read(hotend->ad_cannel),hotend->thermistor_type);
        input_ave += (input - input_ave)/25;
      }
      if( input > 195 ) break;
      if(timestamp - temp_millis > 2000) 
      {
        temp_millis = timestamp;
        usb_printf("ok T:%u  A:%u  @:%u\r\n",(unsigned int)input,(unsigned int)input_ave,pwm);       
      }
    }
    if( input > 195) break;
    points[X][count] = (unsigned char)input;
    points[Y][count] = pwm;
    printf("{%u,%u} ",points[X][count],points[Y][count]);
    if( count++ > 20 ) break;
  }
  printf("\r\n\n");
  hotend->target_temp = 0;
  hotend->pwm = 0;
  autotune_active = false;
  
  unsigned int x_sum = 0;
  unsigned int y_sum = 0;
  unsigned long xy_sum = 0;
  unsigned long x2_sum = 0;
  signed short slope;
  signed short intercept;
  unsigned char i;
  unsigned char max_pwm;
  
  for(i=0; i<count; i++)
  {
    x_sum += points[X][i];
    y_sum += points[Y][i];
    xy_sum += points[X][i] * points[Y][i];
    x2_sum += points[X][i] * points[X][i];
  }
  printf("count = %d, x_sum = %u, y_sum = %u, xy_sum = %u, x2_sum = %u \r\n",count,x_sum,y_sum,(unsigned int)xy_sum,(unsigned int)x2_sum);

  slope = (int)((float)(((count * xy_sum) - (x_sum * y_sum)) * 256) / ((count * x2_sum) - (x_sum * x_sum)) + 0.5);
  intercept = ((y_sum - (((float)(slope * x_sum)/256.0)) + 0.5) / (count));
  
  max_pwm = (unsigned char)min((((long)slope*(long)200>>8)+intercept)*4,255);

  usb_printf("HEATER_SLOPE = %d, HEATER_INTERCEPT = %d \r\n", slope, intercept);  
  usb_printf("Heater evaluation finished \r\n");
  usb_printf("Recommended HEATER_MAX_PWM: %u \r\n",max_pwm);
  usb_printf("Use M301 S%u B%d W%u to set values\r\n", slope, intercept, max_pwm);
  return;
}
//---------------- END EVALUATE HEATER ------------------------------
