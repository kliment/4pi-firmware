/*
 Reprap heater funtions based on Sprinter
 
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

/*
 This softwarepart for Heatercontrol is based on Sprinter
 big thanks to kliment (https://github.com/kliment/Sprinter)
 
*/

#define THERMISTORTYP_TABLE_1 	1
#define THERMISTORTYP_TABLE_2 	2
#define THERMISTORTYP_TABLE_3 	3
#define THERMISTORTYP_TABLE_4 	4
#define THERMISTORTYP_TABLE_5 	5
#define THERMISTORTYP_TABLE_6 	6
#define THERMISTORTYP_TABLE_7 	7

#define THERMISTORTYP_COMPUTE_11 	11
#define THERMISTORTYP_COMPUTE_12 	12
#define THERMISTORTYP_COMPUTE_13 	13
#define THERMISTORTYP_COMPUTE_14 	14
#define THERMISTORTYP_COMPUTE_15 	15
#define THERMISTORTYP_COMPUTE_16 	16
#define THERMISTORTYP_COMPUTE_17 	17

#define AD595_TYP_50 	50


//#include "init_configuration.h"
//#include "thermistortables.h"

/*
#if defined HEATER_USES_THERMISTOR
#if defined COMPUTE_THERMISTORS
#define temp2analogh( c ) temp2analog_thermistor(c,E_BETA, E_RS, E_R_INF)
#define analog2temp( c ) analog2temp_thermistor(c,E_BETA, E_RS, E_R_INF)
#else
#define temp2analogh( c ) temp2analog_thermistor(c,temptable,NUMTEMPS)
#define analog2temp( c ) analog2temp_thermistor(c,temptable,NUMTEMPS)
#endif
#elif defined HEATER_USES_AD595
#define temp2analogh( c ) temp2analog_ad595(c)
#define analog2temp( c ) analog2temp_ad595(c)
#elif defined HEATER_USES_MAX6675
#define temp2analogh( c ) temp2analog_max6675(c)
#define analog2temp( c ) analog2temp_max6675(c)
#endif

#if defined BED_USES_THERMISTOR
#if defined COMPUTE_THERMISTORS
#define temp2analogBed( c ) temp2analog_thermistor((c),BED_BETA, BED_RS, BED_R_INF)
#define analog2tempBed( c ) analog2temp_thermistor((c),BED_BETA, BED_RS, BED_R_INF)
#else
#define temp2analogBed( c ) temp2analog_thermistor((c),bedtemptable,BNUMTEMPS)
#define analog2tempBed( c ) analog2temp_thermistor((c),bedtemptable,BNUMTEMPS)
#endif
#elif defined BED_USES_AD595
#define temp2analogBed( c ) temp2analog_ad595(c)
#define analog2tempBed( c ) analog2temp_ad595(c)
#elif defined BED_USES_MAX6675
#define temp2analogBed( c ) temp2analog_max6675(c)
#define analog2tempBed( c ) analog2temp_max6675(c)
#endif
*/


signed short temp2analog_thermistor_compute(signed short celsius, const float beta, const float rs, const float r_inf);
signed short analog2temp_thermistor_compute(signed short raw, const float beta, const float rs, const float r_inf);

signed short temp2analog_thermistor_table(signed short celsius, const short table[][2], signed short numtemps);
signed short analog2temp_thermistor_table(signed short raw,const short table[][2], signed short numtemps);


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))


void manage_heaters(void);
void init_heaters_values(void);
void heater_switch(unsigned char heater, unsigned char en);
void LED_switch(unsigned char led, unsigned char en);
//void heater_soft_pwm(void);


typedef struct {
	signed short target_temp;
	signed short akt_temp;
	unsigned char pwm;
	unsigned char soft_pwm_aktiv;
	unsigned short PID_Kp;
	unsigned short PID_I;
	unsigned short PID_Kd;
	
	unsigned char io_adr;
	unsigned char ad_cannel;
	
	signed short temp_iState;
	signed short prev_temp;
	signed short pTerm;
	signed short iTerm;
	signed short dTerm;
	signed short temp_iState_min;
	signed short temp_iState_max;
	
	signed short thermistor_type;
	
	signed short slope;
	signed short intercept;
	signed short max_pwm;

} heater_struct;

typedef struct {
	signed short target_temp;
	signed short akt_temp;
	signed short thermistor_type;

} heater_bed_struct;



extern signed short bed_temp_celsius;
extern signed short hotend1_temp_celsius;
extern signed short hotend2_temp_celsius;
extern signed short target_hotend1;

extern heater_struct heaters[];
extern heater_bed_struct bed_heater;

extern volatile unsigned char g_pwm_value[];
extern volatile unsigned char g_pwm_aktiv[];

void PID_autotune(heater_struct *hotend, float PIDAT_test_temp);
void Heater_Eval(heater_struct *hotend, unsigned int step);

