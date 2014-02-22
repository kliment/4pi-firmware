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


#include <board.h>
#include <pio/pio.h>
#include <stdio.h>
#include <stdlib.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <math.h>
#include <string.h>

#include "parameters.h"
#include "init_configuration.h"
#include "gcode_parser.h"
#include "arc_func.h"
#include "planner.h"
#include "heaters.h"
#include "stepper_control.h"
#include "motoropts.h"
#include "globals.h"


float destination[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0, 0.0, 0.0};
char axis_codes[NUM_AXIS] = {'X', 'Y', 'Z', 'E'};
char axis_relative_modes[NUM_AXIS] = _AXIS_RELATIVE_MODES;
float offset[3] = {0.0, 0.0, 0.0};

unsigned long minsegmenttime = 20000;

unsigned long axis_steps_per_sqr_second[NUM_AXIS] ;

unsigned short virtual_steps_x = 0;
unsigned short virtual_steps_y = 0;
unsigned short virtual_steps_z = 0;

const int dropsegments=5; //everything with less than this number of steps will be ignored as move and joined with the next movement

extern volatile unsigned long timestamp;


//===========================================================================
//=================semi-private variables								 =====
//===========================================================================
#define BLOCK_BUFFER_SIZE 32
#define BLOCK_BUFFER_MASK 0x1f
block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions
volatile unsigned char block_buffer_head;           // Index of the next block to be pushed
volatile unsigned char block_buffer_tail;           // Index of the block to process now

// The current position of the tool in absolute steps
long position[4];   
static float previous_speed[4]; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment
static unsigned char G92_reset_previous_speed = 0;

void get_coordinates()
{
	unsigned char i=0;

	for(i = 0; i < NUM_AXIS; i++)
	{
		if (has_code(axis_codes[i]))
			destination[i] = get_float(axis_codes[i]) + (axis_relative_modes[i] || relative_mode)*current_position[i];
		else
        	destination[i] = current_position[i];
	}
    
	if(has_code('F'))
	{
		next_feedrate = get_int('F');
		if(next_feedrate > 0) 
			feedrate = next_feedrate;
	}

	//printf("new POS:%d %d %d %d %d\n\r",(int)destination[0],(int)destination[1],(int)destination[2],(int)destination[3],(int)feedrate);
}

void get_arc_coordinates()
{
	get_coordinates();
	
	if(has_code('I'))
	{
		offset[0] = get_float('I');
	} 
	else
	{
		offset[0] = 0.0f;
	}
	
	if(has_code('J'))
	{
		offset[1] = get_float('J');
	}
	else
	{
		offset[1] = 0.0;
	}
}



void prepare_move()
{
	long help_feedrate = 0;
	unsigned char i=0;

	if(!is_homing)
	{
		if (pa.min_software_endstops) 
		{
			if (destination[X_AXIS] < 0) destination[X_AXIS] = 0.0;
			if (destination[Y_AXIS] < 0) destination[Y_AXIS] = 0.0;
			if (destination[Z_AXIS] < 0) destination[Z_AXIS] = 0.0;
		}

		if (pa.max_software_endstops) 
		{
			if (destination[X_AXIS] > pa.x_max_length) destination[X_AXIS] = pa.x_max_length;
			if (destination[Y_AXIS] > pa.y_max_length) destination[Y_AXIS] = pa.y_max_length;
			if (destination[Z_AXIS] > pa.z_max_length) destination[Z_AXIS] = pa.z_max_length;
		}
	}

	if(destination[E_AXIS] > current_position[E_AXIS])
	{
		help_feedrate = ((long)feedrate*(long)feedmultiply);
	}
	else
	{
		help_feedrate = ((long)feedrate*(long)100);
	}

	//printf("new POS 1:%d %d %d %d %d\n\r",(int)destination[0],(int)destination[1],(int)destination[2],(int)destination[3],(int)feedrate);
	plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], help_feedrate/6000.0,active_extruder);

	for(i=0; i < NUM_AXIS; i++)
	{
		current_position[i] = destination[i];
	} 
}


void prepare_arc_move(char isclockwise) 
{

	float r;
	long help_feedrate = 0;
	unsigned char i=0;

	r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc

	if(destination[E_AXIS] > current_position[E_AXIS])
	{
		help_feedrate = ((long)feedrate*(long)feedmultiply);
	}
	else
	{
		help_feedrate = ((long)feedrate*(long)100);
	}

	// Trace the arc
	mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, Z_AXIS, help_feedrate/6000.0, r, isclockwise,active_extruder);

	// As far as the parser is concerned, the position is now == target. In reality the
	// motion control system might still be processing the action and the real tool position
	// in any intermediate location.
	for(i=0; i < NUM_AXIS; i++) 
	{
		current_position[i] = destination[i];
	}
}

void kill(char debug)
{
	heaters[0].target_temp = 0;
	heaters[1].target_temp = 0;
	heater_switch(1, 0);	//Heater 0
	heater_switch(2, 0);	//Heater 1

	bed_heater.target_temp = 0;
	heater_switch(0, 0);	//BED

	disable_x();
	disable_y();
	disable_z();
	disable_e();
	disable_e1();
	
	if(debug)
		printf("Kill Command\n\r");
   
}

void manage_inactivity(char debug) 
{ 
	if( (timestamp-previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill(0); 

	if( (timestamp-previous_millis_cmd) >  stepper_inactive_time ) if(stepper_inactive_time) 
	{ 
		disable_x(); 
		disable_y(); 
		disable_z(); 
		disable_e(); 
		disable_e1();
	}
	check_axes_activity();
}

//-----------------------------------------------------
// HOMING THE AXIS
//-----------------------------------------------------
void homing_routine(unsigned char axis)
{
  signed short min_pin=0, max_pin=0, home_dir=0, max_length=0, home_bounce=0;

	switch(axis)
	{
		case X_AXIS:
			min_pin = pa.x_min_endstop_aktiv;
			max_pin = pa.x_max_endstop_aktiv;
			home_dir = pa.x_home_dir;
			max_length = pa.x_max_length;
			home_bounce = 10;
		break;
		case Y_AXIS:
			min_pin = pa.y_min_endstop_aktiv;
			max_pin = pa.y_max_endstop_aktiv;
			home_dir = pa.y_home_dir;
			max_length = pa.y_max_length;
			home_bounce = 10;
		break;
		case Z_AXIS:
			min_pin = pa.z_min_endstop_aktiv;
			max_pin = pa.z_max_endstop_aktiv;
			home_dir = pa.z_home_dir;
			max_length = pa.z_max_length;
			home_bounce = 4;
		break;
		default:
			//never reached
		break;
	}

	if ((min_pin > (-1) && home_dir==(-1)) || (max_pin > (-1) && home_dir==1))
	{
		current_position[axis] = (-1.5) * max_length * home_dir;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[axis] = 0;
		feedrate = pa.homing_feedrate[axis];
		prepare_move();
		st_synchronize();

		current_position[axis] = home_bounce/2 * home_dir;
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[axis] = 0;
		prepare_move();
		st_synchronize();

		current_position[axis] = (home_bounce * home_dir)*(-1);
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[axis] = 0;
		feedrate = pa.homing_feedrate[axis]/2;
		prepare_move();
		st_synchronize();

		current_position[axis] = (home_dir == (-1)) ? 0 : max_length;
		current_position[axis] += pa.add_homing[axis];
		plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
		destination[axis] = current_position[axis];
		feedrate = 0;
	}
}



// Planner with Interrupt for Stepper

/*  
 Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 
 s == speed, a == acceleration, t == time, d == distance
 
 Basic definitions:
 
 Speed[s_, a_, t_] := s + (a*t) 
 Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 
 Distance to reach a specific speed with a constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 
 Speed after a given distance of travel with constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 m -> Sqrt[2 a d + s^2]    
 
 DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 
 When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 from initial speed s1 without ever stopping at a plateau:
 
 Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 
 IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */

void st_wake_up() 
{
	//if(busy == 0) 
		//Start Timer ?
}


// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static char next_block_index(char block_index)
{
	block_index++;
	if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
	return(block_index);
}


// Returns the index of the previous block in the ring buffer
static char prev_block_index(char block_index)
{
	if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
	block_index--;
	return(block_index);
}

// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
	if (acceleration!=0)
	{
		return((target_rate*target_rate-initial_rate*initial_rate)/(2.0*acceleration));
	}
	else 
	{
		return 0.0;  // acceleration was 0, set acceleration distance to 0
	}
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
{
	if (acceleration!=0)
	{
		return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
		(4.0*acceleration) );
	}
	else
	{
		return 0.0;  // acceleration was 0, set intersection distance to 0
	}
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor)
{
	unsigned long initial_rate;
	unsigned long final_rate; // (step/min)

	initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
	final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)


	// Limit minimal step rate (Otherwise the timer will overflow.)
	if(initial_rate <120) {initial_rate=120; }
	if(final_rate < 120) {final_rate=120;  }

	long acceleration = block->acceleration_st;
	int32_t accelerate_steps =
		ceil(estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration));
	int32_t decelerate_steps =
		floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate, -acceleration));

	// Calculate the size of Plateau of Nominal Rate.
	int32_t plateau_steps = block->step_event_count-accelerate_steps-decelerate_steps;

	// Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
	// have to use intersection_distance() to calculate when to abort acceleration and start breaking
	// in order to reach the final_rate exactly at the end of this block.
	if (plateau_steps < 0)
	{
		accelerate_steps = ceil(
		intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count));
		
		accelerate_steps = max(accelerate_steps,0); // Check limits due to numerical round-off
		accelerate_steps = min(accelerate_steps,block->step_event_count);
		plateau_steps = 0;
	}

	#ifdef ADVANCE
	volatile long initial_advance = block->advance*entry_factor*entry_factor; 
	volatile long final_advance = block->advance*exit_factor*exit_factor;
	#endif // ADVANCE

	// block->accelerate_until = accelerate_steps;
	// block->decelerate_after = accelerate_steps+plateau_steps;
	//CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
	if(block->busy == 0)
	{// Don't update variables if block is .
		block->accelerate_until = accelerate_steps;
		block->decelerate_after = accelerate_steps+plateau_steps;
		block->initial_rate = initial_rate;
		block->final_rate = final_rate;
		#ifdef ADVANCE
		block->initial_advance = initial_advance;
		block->final_advance = final_advance;
		#endif //ADVANCE
	}
	//CRITICAL_SECTION_END;
}                    

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
float max_allowable_speed(float acceleration, float target_velocity, float distance)
{ 
	return  sqrt(target_velocity*target_velocity-2*acceleration*distance);
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal 
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}



// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next)
{
	if(!current) { return; }
  
    if (next)
	{
		// If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
		// If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
		// check for maximum allowable speed reductions to ensure maximum possible planned speed.
		if (current->entry_speed != current->max_entry_speed)
		{

			// If nominal length true, max junction speed is guaranteed to be reached. Only compute
			// for max allowable speed if block is decelerating and nominal length is false.
			if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) 
			{
				current->entry_speed = min( current->max_entry_speed,
				max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
			}
			else
			{
				current->entry_speed = current->max_entry_speed;
			}
			current->recalculate_flag = 1;

		}
	} // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass() 
{
	unsigned char block_index = block_buffer_head;

	//Make a local copy of block_buffer_tail, because the interrupt can alter it
	//CRITICAL_SECTION_START;
	unsigned char tail = block_buffer_tail;
	//CRITICAL_SECTION_END;

	if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) 
	{
		block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
		block_t *block[3] = { NULL, NULL, NULL };
		
		while(block_index != tail)
		{ 
			block_index = prev_block_index(block_index); 
			block[2]= block[1];
			block[1]= block[0];
			block[0] = &block_buffer[block_index];
			planner_reverse_pass_kernel(block[0], block[1], block[2]);
		}
	}
}


// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) 
{
	if(!previous) { return; }

	// If the previous block is an acceleration block, but it is not long enough to complete the
	// full speed change within the block, we need to adjust the entry speed accordingly. Entry
	// speeds have already been reset, maximized, and reverse planned by reverse planner.
	// If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
	if (!previous->nominal_length_flag)
	{
		if (previous->entry_speed < current->entry_speed)
		{
			double entry_speed = min( current->entry_speed,
			max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );

			// Check for junction speed change
			if (current->entry_speed != entry_speed)
			{
				current->entry_speed = entry_speed;
				current->recalculate_flag = 1;
			}
		}
	}
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass()
{
	unsigned char block_index = block_buffer_tail;
	block_t *block[3] = { NULL, NULL, NULL };

	while(block_index != block_buffer_head)
	{
		block[0] = block[1];
		block[1] = block[2];
		block[2] = &block_buffer[block_index];
		planner_forward_pass_kernel(block[0],block[1],block[2]);
		block_index = next_block_index(block_index);
	}
	planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids()
{
	unsigned char block_index = block_buffer_tail;
	block_t *current;
	block_t *next = NULL;

	while(block_index != block_buffer_head)
	{
		current = next;
		next = &block_buffer[block_index];
		if (current)
		{
			// Recalculate if current block entry or exit junction speed has changed.
			if (current->recalculate_flag || next->recalculate_flag)
			{
				// NOTE: Entry and exit factors always > 0 by all previous logic operations.
				calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,
				next->entry_speed/current->nominal_speed);
				current->recalculate_flag = 0; // Reset current only to ensure next trapezoid is computed
			}
		}
		block_index = next_block_index( block_index );
	}

	// Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
	if(next != NULL)
	{
		calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,
		MINIMUM_PLANNER_SPEED/next->nominal_speed);
		next->recalculate_flag = 0;
	}
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate()
{
	planner_reverse_pass();
	planner_forward_pass();
	planner_recalculate_trapezoids();
}

void plan_init() 
{
	unsigned char cnt_c;
  
	for(cnt_c=0; cnt_c < NUM_AXIS; cnt_c++) 
	{
		axis_steps_per_sqr_second[cnt_c] = pa.max_acceleration_units_per_sq_second[cnt_c] * pa.axis_steps_per_unit[cnt_c];
	}
	
	block_buffer_head = 0;
	block_buffer_tail = 0;
	memset(position, 0, sizeof(position)); // clear position
	previous_speed[0] = 0.0;
	previous_speed[1] = 0.0;
	previous_speed[2] = 0.0;
	previous_speed[3] = 0.0;
	previous_nominal_speed = 0.0;
}



void plan_discard_current_block()
{
	if (block_buffer_head != block_buffer_tail) 
	{
		block_buffer_tail = (block_buffer_tail + 1) & BLOCK_BUFFER_MASK;  
	}
}

block_t *plan_get_current_block()
{
	if (block_buffer_head == block_buffer_tail)
	{ 
		return(NULL); 
	}
	block_t *block = &block_buffer[block_buffer_tail];
	block->busy = 1;
	return(block);
}

// Gets the current block. Returns NULL if buffer empty
unsigned char blocks_queued() 
{
	if (block_buffer_head == block_buffer_tail) 
	{ 
		return 0; 
	}
	else
	{
		return 1;
	}
}

// Block until all buffered steps are executed
void st_synchronize()
{
	while(blocks_queued()) 
	{
		manage_inactivity(1);
	}   
}


void check_axes_activity()
{
	unsigned char x_active = 0;
	unsigned char y_active = 0;  
	unsigned char z_active = 0;
	unsigned char e_active = 0;
	
	block_t *block;

	if(block_buffer_tail != block_buffer_head) 
	{
		unsigned char block_index = block_buffer_tail;
		while(block_index != block_buffer_head) 
		{
			block = &block_buffer[block_index];
			if(block->steps_x != 0) x_active++;
			if(block->steps_y != 0) y_active++;
			if(block->steps_z != 0) z_active++;
			if(block->steps_e != 0) e_active++;
			block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
		}
	}

	if((pa.disable_x_en) && (x_active == 0)) disable_x();
	if((pa.disable_y_en) && (y_active == 0)) disable_y();
	if((pa.disable_z_en) && (z_active == 0)) disable_z();
	if((pa.disable_e_en) && (e_active == 0)) {disable_e(); disable_e1();}
}


float junction_deviation = 0.1;
float max_E_feedrate_calc = MAX_RETRACT_FEEDRATE;
unsigned char retract_feedrate_aktiv = 0;

// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(float x, float y, float z, float e, float feed_rate, unsigned char extruder)
{
	// Calculate the buffer head after we push this byte
	short next_buffer_head = next_block_index(block_buffer_head);

	//printf("next head:%u\n\r",next_buffer_head);

	// If the buffer is full: good! That means we are well ahead of the robot. 
	// Rest here until there is room in the buffer.
	while(block_buffer_tail == next_buffer_head)
	{ 
		//manage_heater(); 
		manage_inactivity(1); 
	}
    
		// The target position of the tool in absolute steps
		// Calculate target position in absolute steps
		//this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
		long target[4];
		target[X_AXIS] = lround(x*pa.axis_steps_per_unit[X_AXIS]);
		target[Y_AXIS] = lround(y*pa.axis_steps_per_unit[Y_AXIS]);
		target[Z_AXIS] = lround(z*pa.axis_steps_per_unit[Z_AXIS]);     
		target[E_AXIS] = lround(e*pa.axis_steps_per_unit[E_AXIS]);

	// Prepare to set up new block
	block_t *block = &block_buffer[block_buffer_head];

	// Mark block as not busy (Not executed by the stepper interrupt)
	block->busy = 0;

	block->active_extruder = extruder;

	// Number of steps for each axis
	block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
	block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
	block->steps_z = labs(target[Z_AXIS]-position[Z_AXIS]);
	block->steps_e = labs(target[E_AXIS]-position[E_AXIS]);
	block->steps_e *= extrudemultiply;
	block->steps_e /= 100;
	block->step_event_count = max(block->steps_x, max(block->steps_y, max(block->steps_z, block->steps_e)));

	// Bail if this is a zero-length block
	if (block->step_event_count <=dropsegments) { return; };

	// Compute direction bits for this block 
	block->direction_bits = 0;
	if (target[X_AXIS] < position[X_AXIS]) { block->direction_bits |= (1<<X_AXIS); }
	if (target[Y_AXIS] < position[Y_AXIS]) { block->direction_bits |= (1<<Y_AXIS); }
	if (target[Z_AXIS] < position[Z_AXIS]) { block->direction_bits |= (1<<Z_AXIS); }
	if (target[E_AXIS] < position[E_AXIS]) 
	{ 
		block->direction_bits |= (1<<E_AXIS); 
		//High Feedrate for retract
		max_E_feedrate_calc = MAX_RETRACT_FEEDRATE;
		retract_feedrate_aktiv = 1;
	}
	else
	{
		if(retract_feedrate_aktiv)
		{
			if(block->steps_e > 0)
			retract_feedrate_aktiv = 0;
		}
		else
		{
			max_E_feedrate_calc = pa.max_feedrate[E_AXIS]; 
		}
	}


	#ifdef DELAY_ENABLE
	if(block->steps_x != 0)
	{
		enable_x();
		delayMicroseconds(DELAY_ENABLE);
	}
	if(block->steps_y != 0)
	{
		enable_y();
		delayMicroseconds(DELAY_ENABLE);
	}
	if(block->steps_z != 0)
	{
		enable_z();
		delayMicroseconds(DELAY_ENABLE);
	}
	if(block->steps_e != 0)
	{
		enable_e();
		enable_e1();
		delayMicroseconds(DELAY_ENABLE);
	}
	#else

	//enable active axes
	if(block->steps_x != 0) enable_x();
	if(block->steps_y != 0) enable_y();
	if(block->steps_z != 0) enable_z();
	if(block->steps_e != 0) { enable_e(); enable_e1(); }
	#endif 

	if (block->steps_e == 0)
	{
		if(feed_rate<pa.mintravelfeedrate) feed_rate=pa.mintravelfeedrate;
	}
	else
	{
		if(feed_rate<pa.minimumfeedrate) feed_rate=pa.minimumfeedrate;
	} 

	// slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
	int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
	#ifdef SLOWDOWN  
	if(moves_queued < (BLOCK_BUFFER_SIZE * 0.5) && moves_queued > 1) feed_rate = feed_rate*moves_queued / (BLOCK_BUFFER_SIZE * 0.5); 
	#endif

	float delta_mm[4];
	delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/pa.axis_steps_per_unit[X_AXIS];
	delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/pa.axis_steps_per_unit[Y_AXIS];
	delta_mm[Z_AXIS] = (target[Z_AXIS]-position[Z_AXIS])/pa.axis_steps_per_unit[Z_AXIS];
	//delta_mm[E_AXIS] = (target[E_AXIS]-position[E_AXIS])/pa.axis_steps_per_unit[E_AXIS];
	delta_mm[E_AXIS] = ((target[E_AXIS]-position[E_AXIS])/pa.axis_steps_per_unit[E_AXIS])*extrudemultiply/100.0;

	if ( block->steps_x <= dropsegments && block->steps_y <= dropsegments && block->steps_z <= dropsegments )
	{
		block->millimeters = fabs(delta_mm[E_AXIS]);
	} 
	else
	{
		block->millimeters = sqrt(pow(delta_mm[X_AXIS],2) + pow(delta_mm[Y_AXIS],2) + pow(delta_mm[Z_AXIS],2));
	}

	float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides 

	// Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
	float inverse_second = feed_rate * inverse_millimeters;

	block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
	block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0





	/*
	//  segment time im micro seconds
	long segment_time = lround(1000000.0/inverse_second);
	if ((blockcount>0) && (blockcount < (BLOCK_BUFFER_SIZE - 4))) {
		if (segment_time<minsegmenttime)  { // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
			segment_time=segment_time+lround(2*(minsegmenttime-segment_time)/blockcount);
		}
	}
	else {
		if (segment_time<minsegmenttime) segment_time=minsegmenttime;
	}
	//  END OF SLOW DOWN SECTION    
	*/


	// Calculate and limit speed in mm/sec for each axis
	float current_speed[4];
	float speed_factor = 1.0; //factor <=1 do decrease speed
	unsigned char cnt_c;

	for(cnt_c=0; cnt_c < 3; cnt_c++) 
	{
		current_speed[cnt_c] = delta_mm[cnt_c] * inverse_second;
		if(fabs(current_speed[cnt_c]) > pa.max_feedrate[cnt_c])
			speed_factor = min(speed_factor, pa.max_feedrate[cnt_c] / fabs(current_speed[cnt_c]));
	}

		current_speed[E_AXIS] = delta_mm[E_AXIS] * inverse_second;
		if(fabs(current_speed[E_AXIS]) > max_E_feedrate_calc)
			speed_factor = min(speed_factor, max_E_feedrate_calc / fabs(current_speed[E_AXIS]));


	// Correct the speed  
	if( speed_factor < 1.0) 
	{
		for(cnt_c=0; cnt_c < 4; cnt_c++)
		{
			current_speed[cnt_c] *= speed_factor;
		}
		block->nominal_speed *= speed_factor;
		block->nominal_rate *= speed_factor;
	}

	// Compute and limit the acceleration rate for the trapezoid generator.  
	float steps_per_mm = block->step_event_count/block->millimeters;
	if(block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0)
	{
		block->acceleration_st = ceil(pa.retract_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
	}
	else
	{
		block->acceleration_st = ceil(pa.move_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
		// Limit acceleration per axis
		if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
			block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
		if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
			block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
		if(((float)block->acceleration_st * (float)block->steps_e / (float)block->step_event_count) > axis_steps_per_sqr_second[E_AXIS])
			block->acceleration_st = axis_steps_per_sqr_second[E_AXIS];
		if(((float)block->acceleration_st * (float)block->steps_z / (float)block->step_event_count ) > axis_steps_per_sqr_second[Z_AXIS])
			block->acceleration_st = axis_steps_per_sqr_second[Z_AXIS];
	}
	block->acceleration = block->acceleration_st / steps_per_mm;
	block->acceleration_rate = (long)((float)block->acceleration_st * 8.388608);

	#if 0  // Use old jerk for now
	// Compute path unit vector
	double unit_vec[3];

	unit_vec[X_AXIS] = delta_mm[X_AXIS]*inverse_millimeters;
	unit_vec[Y_AXIS] = delta_mm[Y_AXIS]*inverse_millimeters;
	unit_vec[Z_AXIS] = delta_mm[Z_AXIS]*inverse_millimeters;

	// Compute maximum allowable entry speed at junction by centripetal acceleration approximation.
	// Let a circle be tangent to both previous and current path line segments, where the junction
	// deviation is defined as the distance from the junction to the closest edge of the circle,
	// colinear with the circle center. The circular segment joining the two paths represents the
	// path of centripetal acceleration. Solve for max velocity based on max acceleration about the
	// radius of the circle, defined indirectly by junction deviation. This may be also viewed as
	// path width or max_jerk in the previous grbl version. This approach does not actually deviate
	// from path, but used as a robust way to compute cornering speeds, as it takes into account the
	// nonlinearities of both the junction angle and junction velocity.
	double vmax_junction = MINIMUM_PLANNER_SPEED; // Set default max junction speed

	// Skip first block or when previous_nominal_speed is used as a flag for homing and offset cycles.
	if ((block_buffer_head != block_buffer_tail) && (previous_nominal_speed > 0.0))
	{
		// Compute cosine of angle between previous and current path. (prev_unit_vec is negative)
		// NOTE: Max junction velocity is computed without sin() or acos() by trig half angle identity.
		double cos_theta = 	- previous_unit_vec[X_AXIS] * unit_vec[X_AXIS]
							- previous_unit_vec[Y_AXIS] * unit_vec[Y_AXIS]
							- previous_unit_vec[Z_AXIS] * unit_vec[Z_AXIS] ;
	   
		// Skip and use default max junction speed for 0 degree acute junction.
		if (cos_theta < 0.95)
		{
			vmax_junction = min(previous_nominal_speed,block->nominal_speed);
			// Skip and avoid divide by zero for straight junctions at 180 degrees. Limit to min() of nominal speeds.
			if (cos_theta > -0.95)
			{
				// Compute maximum junction velocity based on maximum acceleration and junction deviation
				double sin_theta_d2 = sqrt(0.5*(1.0-cos_theta)); // Trig half angle identity. Always positive.
				vmax_junction = min(vmax_junction,
				sqrt(block->acceleration * junction_deviation * sin_theta_d2/(1.0-sin_theta_d2)) );
			}
		}
	}
	#endif
	
	// Start with a safe speed
	float vmax_junction = pa.max_xy_jerk/2; 
	float vmax_junction_factor = 1.0; 

	if(fabs(current_speed[Z_AXIS]) > pa.max_z_jerk/2) 
		vmax_junction = min(vmax_junction, pa.max_z_jerk/2);

	if(fabs(current_speed[E_AXIS]) > pa.max_e_jerk/2) 
		vmax_junction = min(vmax_junction, pa.max_e_jerk/2);

	if(G92_reset_previous_speed == 1)
	{
		vmax_junction = 0.1;
		G92_reset_previous_speed = 0;  
	}

	vmax_junction = min(vmax_junction, block->nominal_speed);
	float safe_speed = vmax_junction;

	if ((moves_queued > 1) && (previous_nominal_speed > 0.0001))
	{
		float jerk = sqrt(pow((current_speed[X_AXIS]-previous_speed[X_AXIS]), 2)+pow((current_speed[Y_AXIS]-previous_speed[Y_AXIS]), 2));
		//    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
		vmax_junction = block->nominal_speed;
		//    }
		if (jerk > pa.max_xy_jerk)
		{
			vmax_junction_factor = (pa.max_xy_jerk/jerk);
		} 

		if(fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS]) > pa.max_z_jerk)
		{
			vmax_junction_factor= min(vmax_junction_factor, (pa.max_z_jerk/fabs(current_speed[Z_AXIS] - previous_speed[Z_AXIS])));
		} 

		if(fabs(current_speed[E_AXIS] - previous_speed[E_AXIS]) > pa.max_e_jerk)
		{
			vmax_junction_factor = min(vmax_junction_factor, (pa.max_e_jerk/fabs(current_speed[E_AXIS] - previous_speed[E_AXIS])));
		} 
		vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
	}

	block->max_entry_speed = vmax_junction;

	// Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
	double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
	block->entry_speed = min(vmax_junction, v_allowable);

	// Initialize planner efficiency flags
	// Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
	// If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
	// the current block and next block junction speeds are guaranteed to always be at their maximum
	// junction speeds in deceleration and acceleration, respectively. This is due to how the current
	// block nominal speed limits both the current and next maximum junction speeds. Hence, in both
	// the reverse and forward planners, the corresponding block junction speed will always be at the
	// the maximum junction speed and may always be ignored for any speed reduction checks.
	if (block->nominal_speed <= v_allowable)
	{ 
		block->nominal_length_flag = 1; 
	}
	else
	{ 
		block->nominal_length_flag = 0; 
	}
	block->recalculate_flag = 1; // Always calculate trapezoid for new block

	// Update previous path unit_vector and nominal speed
	memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
	previous_nominal_speed = block->nominal_speed;

	#ifdef ADVANCE
	// Calculate advance rate
	if((block->steps_e == 0) || (block->steps_x == 0 && block->steps_y == 0 && block->steps_z == 0))
	{
		block->advance_rate = 0;
		block->advance = 0;
	}
	else
	{
		long acc_dist = estimate_acceleration_distance(0, block->nominal_rate, block->acceleration_st);
		float advance = (STEPS_PER_CUBIC_MM_E * EXTRUDER_ADVANCE_K) * 
		(current_speed[E_AXIS] * current_speed[E_AXIS] * EXTRUTION_AREA * EXTRUTION_AREA)*256;
		
		block->advance = advance;
		if(acc_dist == 0)
		{
			block->advance_rate = 0;
		} 
		else
		{
			block->advance_rate = advance / (float)acc_dist;
		}
	}

	#endif // ADVANCE


	calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed,
	safe_speed/block->nominal_speed);

	// Move buffer head
	block_buffer_head = next_buffer_head;

	// Update position
	memcpy(position, target, sizeof(target)); // position[] = target[]

	planner_recalculate();
	st_wake_up();
}

short calc_plannerpuffer_fill(void)
{
	short moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
	return(moves_queued);
}

void plan_set_position(float x, float y, float z, float e)
{
	position[X_AXIS] = lround(x*pa.axis_steps_per_unit[X_AXIS]);
	position[Y_AXIS] = lround(y*pa.axis_steps_per_unit[Y_AXIS]);
	position[Z_AXIS] = lround(z*pa.axis_steps_per_unit[Z_AXIS]);     
	position[E_AXIS] = lround(e*pa.axis_steps_per_unit[E_AXIS]);  

	virtual_steps_x = 0;
	virtual_steps_y = 0;
	virtual_steps_z = 0;

	previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
	previous_speed[0] = 0.0;
	previous_speed[1] = 0.0;
	previous_speed[2] = 0.0;
	previous_speed[3] = 0.0;

	G92_reset_previous_speed = 1;
}