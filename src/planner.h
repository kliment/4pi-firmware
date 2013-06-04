
/*
 Planner for smooth moves

 Originally from Grbl (http://github.com/grbl/grbl)
  
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


#define X_AXIS  0
#define Y_AXIS  1
#define Z_AXIS  2
#define E_AXIS  3
#define E1_AXIS 4		//for Stepper Control

#define disable_x()  motor_enaxis(0, 0)
#define disable_y()  motor_enaxis(1, 0)
#define disable_z()  motor_enaxis(2, 0)
#define disable_e()  motor_enaxis(3, 0)
#define disable_e1() motor_enaxis(4, 0)

#define enable_x()  motor_enaxis(0, 1)
#define enable_y()  motor_enaxis(1, 1)
#define enable_z()  motor_enaxis(2, 1)
#define enable_e()  motor_enaxis(3, 1)
#define enable_e1() motor_enaxis(4, 1)






// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis

  unsigned long step_event_count;                    // The number of step events required to complete this block
  long accelerate_until;           // The index of the step event on which to stop acceleration
  long decelerate_after;           // The index of the step event on which to start decelerating
  long acceleration_rate;          // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  unsigned char active_extruder;
  
  #ifdef ADVANCE
    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;
  #endif

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;          // Nominal mm/minute for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/min  
  float entry_speed;                                 // Entry speed at previous-current junction in mm/min
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/min
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached


  // Settings for the trapezoid generator
  long nominal_rate;                                 // The nominal step rate for this block in step_events/sec 
  long initial_rate;                        // The jerk-adjusted step rate at start of block  
  long final_rate;                          // The minimal rate at exit
  long acceleration_st;                              // acceleration steps/sec^2
  volatile char busy;
} block_t;



void manage_inactivity(char debug);
void get_coordinates();
void prepare_move();
void prepare_arc_move(char isclockwise);
void get_arc_coordinates();


void process_commands();
void get_arc_coordinates();

void kill(char debug);

void homing_routine(unsigned char axis);

void check_axes_activity();
void plan_init();
void st_init();
void tp_init();
void plan_buffer_line(float x, float y, float z, float e, float feed_rate, unsigned char extruder);
void plan_set_position(float x, float y, float z, float e);
void st_wake_up();
void st_synchronize();
void st_set_position(long x, long y, long z, long e);
void st_synchronize();
void plan_discard_current_block();
block_t *plan_get_current_block();


extern char axis_relative_modes[];
extern unsigned long minsegmenttime;

extern unsigned long axis_steps_per_sqr_second[];

extern float destination[];
extern float current_position[];
extern char axis_codes[];
extern char axis_relative_modes[];
extern float offset[];

extern unsigned short virtual_steps_x;
extern unsigned short virtual_steps_y;
extern unsigned short virtual_steps_z;

extern signed short feedrate;
extern signed short next_feedrate;
extern signed short saved_feedrate;

extern unsigned char is_homing;
extern unsigned char home_all_axis;


