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

// Support to compute temperature  from themistor Beta instead of using lookup tables
// See http://en.wikipedia.org/wiki/Thermistor#B_or_.CE.B2_parameter_equation
//#define COMPUTE_THERMISTORS
#define ABS_ZERO -273.15
#define ADC_VREF       3300  // 3.3 * 1000

// extruder thermistor
#define E_BETA 4066	// EPCOS 100K Thermistor (B57540G0104F000)
#define E_R_INF ( 100000.0*exp(-E_BETA/298.15) ) // 100K @ 25C
#define E_RS 4700 // 4pi uses 4.7K series resistor to ADC_VREF

// heated bed thermistor
#define BED_BETA 4066	// EPCOS 100K Thermistor (B57540G0104F000)
#define BED_R_INF ( 100000.0*exp(-E_BETA/298.15) ) // 100K @ 25C
#define BED_RS 4700 // 4pi uses 4.7K series resistor to ADC_VREF


//#define BETA 4092	// EPCOS 100K Thermistor (B57560G1104F)
//#define R_INF ( 100000.0*exp(-E_BETA/298.15) ) // 100K @ 25C
//#define RS 4700

//#define BETA 3974	// Honeywell 100K Thermistor (135-104LAG-J01)
//#define R_INF ( 100000.0*exp(-E_BETA/298.15) ) // 100K @ 25C
//#define RS 4700

//#define BETA 3960	// RRRF 100K Thermistor; RS 198-961
//#define R_INF ( 100000.0*exp(-E_BETA/298.15) ) // 100K @ 25C
//#define RS 4700

//#define BETA 3964	// RRRF 10K Thermistor
//#define R_INF ( 10000.0*exp(-E_BETA/298.15) ) // 10K @ 25C
//#define RS 4700

//#define BETA 3480	// EPCOS 10K Thermistor (B57550G103J); RS 484-0149
//#define R_INF ( 10000.0*exp(-E_BETA/298.15) ) // 10K @ 25C
//#define RS 4700


//PID Controler Settings
#define PID_INTEGRAL_DRIVE_MAX 80 // too big, and heater will lag after changing temperature, too small and it might not compensate enough for long-term errors
#define PID_PGAIN 1024 //256 is 1.0  // value of X means that error of 1 degree is changing PWM duty by X, probably no need to go over 25
#define PID_IGAIN 22 //256 is 1.0  // value of X (e.g 0.25) means that each degree error over 1 sec (2 measurements) changes duty cycle by 2X (=0.5) units (verify?)
#define PID_DGAIN 2048 //256 is 1.0  // value of X means that around reached setpoint, each degree change over one measurement (half second) adjusts PWM by X units to compensate

#define HEATER_DUTY_FOR_SETPOINT(setpoint) ((signed short)((187L*(long)setpoint)>>8)-27)  
// Change this value (range 30-255) to limit the current to the nozzle
#define HEATER_CURRENT 255

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

//#include "Configuration.h"
//#include "thermistortables.h"

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

#if defined (HEATER_USES_THERMISTOR) || defined (BED_USES_THERMISTOR)
#if defined COMPUTE_THERMISTORS
signed short temp2analog_thermistor(signed short celsius, const float beta, const float rs, const float r_inf);
signed short analog2temp_thermistor(signed short raw, const float beta, const float rs, const float r_inf);
#else
signed short temp2analog_thermistor(signed short celsius, const short table[][2], signed short numtemps);
signed short analog2temp_thermistor(signed short raw,const short table[][2], signed short numtemps);
#endif
#endif

#if defined (HEATER_USES_AD595) || defined (BED_USES_AD595)
signed short temp2analog_ad595(signed short celsius);
signed short analog2temp_ad595(signed short raw);
#endif

#if defined (HEATER_USES_MAX6675) || defined (BED_USES_MAX6675)
signed short temp2analog_max6675(signed short celsius);
signed short analog2temp_max6675(signed short raw);
#endif


void manage_heaters(void);
void init_heaters_values(void);
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

} heater_struct;

typedef struct {
	signed short target_temp;
	signed short akt_temp;

} heater_bed_struct;



extern signed short bed_temp_celsius;
extern signed short hotend1_temp_celsius;
extern signed short hotend2_temp_celsius;
extern signed short target_hotend1;

extern heater_struct heaters[];
extern heater_bed_struct bed_heater;
