#ifndef GCODE_PARAMS_H_RBHAFQY3
#define GCODE_PARAMS_H_RBHAFQY3

#include <inttypes.h>

extern unsigned char relative_mode;
extern volatile signed short feedmultiply; //100->original / 200 -> Factor 2 / 50 -> Factor 0.5
extern signed short saved_feedmultiply;
extern volatile char feedmultiplychanged;
extern volatile signed short extrudemultiply; //100->1 200->2

extern unsigned char active_extruder;		//0 --> Exteruder 1 / 1 --> Extruder 2
extern signed short feedrate;
extern signed short next_feedrate;
extern signed short saved_feedrate;

extern unsigned char is_homing;
extern unsigned char home_all_axis;
extern volatile signed short extrudemultiply; // Sets extrude multiply factor (in percent)
extern unsigned int previous_millis_cmd;
extern unsigned long max_inactive_time;
extern unsigned long stepper_inactive_time;
extern volatile unsigned long timestamp;

#endif /* end of include guard: GCODE_PARAMS_H_RBHAFQY3 */
