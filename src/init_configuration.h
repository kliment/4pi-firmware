#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// BASIC SETTINGS for the first start, settings can change with G-Code commands

#define false 0
#define true 1

// NO SENSOR
// 0 no Sensor is used

//// Thermistor convert with table:
// 1 is 100k thermistor B57540G0104F000
// 2 is 200k thermistor
// 3 is mendel-parts thermistor
// 4 is 10k thermistor
// 5 is ParCan supplied 104GT-2 100K 
// 6 is EPCOS 100k B57560G1104F
// 7 is 100k Honeywell thermistor 135-104LAG-J01

//// Thermistor convert with formular
// 11 EPCOS 100K Thermistor (B57540G0104F000)
// 16 EPCOS 100K Thermistor (B57560G1104F)
// 17 Honeywell 100K Thermistor (135-104LAG-J01)
// 15 RRRF 100K Thermistor; RS 198-961
// 14 RRRF 10K Thermistor
// 13 EPCOS 10K Thermistor (B57550G103J); RS 484-0149

//// AD595
// 50 get Temperatur with AD595

#define THERMISTORHEATER 1
#define THERMISTORBED 1


//-----------------------------------------------------------------------
//// Calibration variables
//-----------------------------------------------------------------------
// X, Y, Z, E steps per unit
#define _AXIS_STEP_PER_UNIT {80, 80, 3200/1.25,700}

#define _AXIS_CURRENT {128, 128, 128, 128, 128}
#define _AXIS_USTEP {3, 3, 3, 3, 3}

//-----------------------------------------------------------------------
//// Endstop Settings
//-----------------------------------------------------------------------
//The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
//If your axes are only moving in one direction, make sure the endstops are connected properly.
#define ENDSTOPPULLUPS // Comment this out (using // at the start of the line) to disable the endstop pullup resistors

//If your axes move in one direction ONLY when the endstops are triggered, set [XYZ]_ENDSTOP_INVERT to true here:
#define _X_ENDSTOP_INVERT 	false
#define _Y_ENDSTOP_INVERT 	false
#define _Z_ENDSTOP_INVERT 	false

//#define ENDSTOPS_ONLY_FOR_HOMING // If defined the endstops will only be used for homing

#define _MIN_SOFTWARE_ENDSTOPS false; //If true, axis won't move to coordinates less than zero.
#define _MAX_SOFTWARE_ENDSTOPS true; //If true, axis won't move to coordinates greater than the defined lengths below.

//-----------------------------------------------------------------------
//// ENDSTOP INPUT ACTIV:	1 --> Active // -1 --> NO ENDSTOP 
//-----------------------------------------------------------------------
#define X_MIN_ACTIV          1
#define X_MAX_ACTIV          -1

#define Y_MIN_ACTIV          1
#define Y_MAX_ACTIV          -1

#define Z_MIN_ACTIV          1
#define Z_MAX_ACTIV          -1

//-----------------------------------------------------------------------
// Disables axis when it's not being used.
//-----------------------------------------------------------------------

#define _DISABLE_X 	false
#define _DISABLE_Y 	false
#define _DISABLE_Z 	true
#define _DISABLE_E 	false

//-----------------------------------------------------------------------
// Inverting axis direction
//-----------------------------------------------------------------------
#define _INVERT_X_DIR 	false
#define _INVERT_Y_DIR 	false
#define _INVERT_Z_DIR 	true
#define _INVERT_E_DIR 	false

//-----------------------------------------------------------------------
//// HOMING SETTINGS:
//-----------------------------------------------------------------------
// Sets direction of endstops when homing; 1=MAX, -1=MIN
#define X_HOME_DIR -1
#define Y_HOME_DIR -1
#define Z_HOME_DIR -1

//-----------------------------------------------------------------------
//Max Length for Prusa Mendel, check the ways of your axis and set this Values
//-----------------------------------------------------------------------
#define _X_MAX_LENGTH	200
#define _Y_MAX_LENGTH 	200
#define _Z_MAX_LENGTH 	100

//-----------------------------------------------------------------------
//// MOVEMENT SETTINGS
//-----------------------------------------------------------------------
#define _MAX_FEEDRATE {400, 400, 2, 45}       // (mm/sec)    
#define _HOMING_FEEDRATE {1500,1500,120}      // (mm/min) !!
#define _AXIS_RELATIVE_MODES {false, false, false, false}


//-----------------------------------------------------------------------
//// Acceleration settings
//-----------------------------------------------------------------------
// X, Y, Z, E maximum start speed for accelerated moves. E default values are good for skeinforge 40+, for older versions raise them a lot.
#define _ACCELERATION 1000         // Axis Normal acceleration mm/s^2
#define _RETRACT_ACCELERATION 2000 // Extruder Normal acceleration mm/s^2
#define _MAX_XY_JERK 20.0
#define _MAX_Z_JERK 0.4
#define _MAX_E_JERK 5.0
#define _MAX_ACCELERATION_UNITS_PER_SQ_SECOND {1000,1000,50,5000}    // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts

//For the retract (negative Extruder) move this maxiumum Limit of Feedrate is used
//The next positive Extruder move use also this Limit, 
//then for the next (second after retract) move the original Maximum (_MAX_FEEDRATE) Limit is used
#define MAX_RETRACT_FEEDRATE 100    //mm/sec


// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05 // (mm/sec)

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// If defined the movements slow down when the look ahead buffer is only half full
#define SLOWDOWN


//-----------------------------------------------------------------------
// Machine UUID
//-----------------------------------------------------------------------
// This may be useful if you have multiple machines and wish to identify them by using the M115 command. 
// By default we set it to zeros.
#define _DEF_CHAR_UUID "00000000-0000-0000-0000-000000000000"


//-----------------------------------------------------------------------
//// HEATERCONTROL AND PID PARAMETERS
//-----------------------------------------------------------------------

#define PIDTEMP 1
#ifdef PIDTEMP

	//PID Controler Settings
	#define PID_INTEGRAL_DRIVE_MAX 80 // too big, and heater will lag after changing temperature, too small and it might not compensate enough for long-term errors
	#define PID_PGAIN 2560 //256 is 1.0  // value of X means that error of 1 degree is changing PWM duty by X, probably no need to go over 25
	#define PID_IGAIN 64 //256 is 1.0  // value of X (e.g 0.25) means that each degree error over 1 sec (2 measurements) changes duty cycle by 2X (=0.5) units (verify?)
	#define PID_DGAIN 4096 //256 is 1.0  // value of X means that around reached setpoint, each degree change over one measurement (half second) adjusts PWM by X units to compensate

	// magic formula 1, to get approximate "zero error" PWM duty. Take few measurements with low PWM duty and make linear fit to get the formula
	// for my makergear hot-end: linear fit {50,10},{60,20},{80,30},{105,50},{176,100},{128,64},{208,128}
	#define HEATER_DUTY_FOR_SETPOINT(setpoint) ((int)((187L*(long)setpoint)>>8)-27)  
	// magic formula 2, to make led brightness approximately linear
	#define LED_PWM_FOR_BRIGHTNESS(brightness) ((64*brightness-1384)/(300-brightness))
	
	#define HEATER_1_PWM	1
	#define HEATER_2_PWM	1

#else

	#define HEATER_1_PWM	0
	#define HEATER_2_PWM	0

#endif

// Change this value (range 1-255) to limit the current to the nozzle
#define HEATER_CURRENT 255

// How often should the heater check for new temp readings, in milliseconds
#define HEATER_CHECK_INTERVAL 200
#define BED_CHECK_INTERVAL 5000


//// Experimental watchdog and minimal temp
// The watchdog waits for the watchperiod in milliseconds whenever an M104 or M109 increases the target temperature
// If the temperature has not increased at the end of that period, the target temperature is set to zero. It can be reset with another M104/M109
//#define WATCHPERIOD 5000 //5 seconds

// Actual temperature must be close to target for this long before M109 returns success
//#define TEMP_RESIDENCY_TIME 20  // (seconds)
//#define TEMP_HYSTERESIS 5       // (C°) range of +/- temperatures considered "close" to the target one

//// The minimal temperature defines the temperature below which the heater will not be enabled
#define MINTEMP 5

//// Experimental max temp
// When temperature exceeds max temp, your heater will be switched off.
// This feature exists to protect your hotend from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define MAXTEMP 275



#endif