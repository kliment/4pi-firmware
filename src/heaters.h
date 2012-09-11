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

#define THERMISTORHEATER	1
#define THERMISTORBED		1

#define BED_USES_THERMISTOR
#define HEATER_USES_THERMISTOR

#define MINTEMP
#define MAXTEMP

//PID Controler Settings
#define PID_INTEGRAL_DRIVE_MAX 80 // too big, and heater will lag after changing temperature, too small and it might not compensate enough for long-term errors
#define PID_PGAIN 1024 //256 is 1.0  // value of X means that error of 1 degree is changing PWM duty by X, probably no need to go over 25
#define PID_IGAIN 22 //256 is 1.0  // value of X (e.g 0.25) means that each degree error over 1 sec (2 measurements) changes duty cycle by 2X (=0.5) units (verify?)
#define PID_DGAIN 2048 //256 is 1.0  // value of X means that around reached setpoint, each degree change over one measurement (half second) adjusts PWM by X units to compensate

#define HEATER_DUTY_FOR_SETPOINT(setpoint) ((int)((187L*(long)setpoint)>>8)-27)  
// Change this value (range 30-255) to limit the current to the nozzle
#define HEATER_CURRENT 255

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

//#include "Configuration.h"
//#include "thermistortables.h"

#if defined HEATER_USES_THERMISTOR
#define temp2analogh( c ) temp2analog_thermistor(c,temptable,NUMTEMPS)
#define analog2temp( c ) analog2temp_thermistor(c,temptable,NUMTEMPS)
#elif defined HEATER_USES_AD595
#define temp2analogh( c ) temp2analog_ad595(c)
#define analog2temp( c ) analog2temp_ad595(c)
#elif defined HEATER_USES_MAX6675
#define temp2analogh( c ) temp2analog_max6675(c)
#define analog2temp( c ) analog2temp_max6675(c)
#endif

#if defined BED_USES_THERMISTOR
#define temp2analogBed( c ) temp2analog_thermistor((c),bedtemptable,BNUMTEMPS)
#define analog2tempBed( c ) analog2temp_thermistor((c),bedtemptable,BNUMTEMPS)
#elif defined BED_USES_AD595
#define temp2analogBed( c ) temp2analog_ad595(c)
#define analog2tempBed( c ) analog2temp_ad595(c)
#elif defined BED_USES_MAX6675
#define temp2analogBed( c ) temp2analog_max6675(c)
#define analog2tempBed( c ) analog2temp_max6675(c)
#endif

#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
int temp2analog_thermistor(int celsius, const short table[][2], int numtemps);
int analog2temp_thermistor(int raw,const short table[][2], int numtemps);
#endif

#if defined (HEATER_USES_AD595) || defined (BED_USES_AD595)
int temp2analog_ad595(int celsius);
int analog2temp_ad595(int raw);
#endif

#if defined (HEATER_USES_MAX6675) || defined (BED_USES_MAX6675)
int temp2analog_max6675(int celsius);
int analog2temp_max6675(int raw);
#endif


void manage_heaters(void);
void init_heaters_values(void);
void heater_soft_pwm(void);



typedef struct {
	int target_temp;
	int akt_temp;
	unsigned char pwm;
	unsigned char soft_pwm_aktiv;
	unsigned int PID_Kp;
	unsigned int PID_I;
	unsigned int PID_Kd;
	
	unsigned char io_adr;
	unsigned char ad_cannel;
	
	int temp_iState;
	int prev_temp;
	int pTerm;
	int iTerm;
	int dTerm;
	int temp_iState_min;
	int temp_iState_max;

} heater_struct;

typedef struct {
	int target_temp;
	int akt_temp;

} heater_bed_struct;



extern int bed_temp_celsius;
extern int hotend1_temp_celsius;
extern int hotend2_temp_celsius;
extern int target_hotend1;

extern heater_struct heaters[];
extern heater_bed_struct bed_heater;
