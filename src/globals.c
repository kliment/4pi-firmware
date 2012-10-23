
#include <inttypes.h>


unsigned char relative_mode = 0;
volatile signed short feedmultiply=100; //100->original / 200 -> Factor 2 / 50 -> Factor 0.5
signed short saved_feedmultiply = 0;
volatile char feedmultiplychanged=0;
volatile signed short extrudemultiply=100; //100->1 200->2

unsigned char active_extruder = 0;		//0 --> Exteruder 1 / 1 --> Extruder 2
signed short feedrate = 1500;
signed short next_feedrate;
signed short saved_feedrate;

unsigned char is_homing = 0;
unsigned char home_all_axis = 1;
volatile signed short extrudemultiply; // Sets extrude multiply factor (in percent)
unsigned int previous_millis_cmd;
unsigned long max_inactive_time = 0;
unsigned long stepper_inactive_time = 0;
