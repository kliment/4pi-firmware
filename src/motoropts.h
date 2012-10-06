



extern unsigned char axis_current[];
extern unsigned char axis_ustep[];


unsigned char microstep_mode(unsigned char usteps);
void motor_setopts(unsigned char axis, unsigned char ustepbits, unsigned char current);
void motor_setup();

 

