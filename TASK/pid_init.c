#include "pid_init.h"
#include "pid.h"
#include "main.h"


	pid_Motor all_motor_pid;

void pid_init(void){
	
			PID_clear(&all_motor_pid.pid_yaw_angle);
			PID_clear(&all_motor_pid.pid_yaw_speed);
			PID_clear(&all_motor_pid.pid_pitchI_angle);
			PID_clear(&all_motor_pid.pid_pitchI_speed);
	
			pid_assignment(all_motor_pid.pid_front_right.pid_parameter,5,0.2,1);
			pid_assignment(all_motor_pid.pid_front_left.pid_parameter,5,0.2,1);
			pid_assignment(all_motor_pid.pid_back_left.pid_parameter,5,0.2,1);
			pid_assignment(all_motor_pid.pid_back_right.pid_parameter,5,0.2,1);
	
			pid_assignment(all_motor_pid.pid_pitchI_angle.pid_parameter,20 ,		0,	1) ;
			pid_assignment(all_motor_pid.pid_pitchI_speed.pid_parameter,130	 ,	0.7, 0) ;
	
			pid_assignment(all_motor_pid.pid_yaw_angle.pid_parameter,7.5, 0,0) ;
			pid_assignment(all_motor_pid.pid_yaw_speed.pid_parameter,45,5, 0) ;
	
			pid_assignment(all_motor_pid.pid_follow_angle.pid_parameter,0.1,		0,	0) ;
			pid_assignment(all_motor_pid.pid_follow_speed.pid_parameter,25 ,	3.5, 0) ;
	
			pid_assignment(all_motor_pid.pid_friction_left.pid_parameter,5,		0,	0) ;
			pid_assignment(all_motor_pid.pid_friction_right.pid_parameter,5 ,	0, 0) ;
			
			
			pid_assignment(all_motor_pid.pid_pop_up_wheel_angle.pid_parameter,0.1,		0,	0) ;
			pid_assignment(all_motor_pid.pid_pop_up_wheel_speed.pid_parameter,11,		2,	0) ;
	
	PID_Init(&all_motor_pid.pid_front_right,
		 PID_POSITION,
		 all_motor_pid.pid_front_right.pid_parameter,
		 12000,
		 100);
	
	PID_Init(&all_motor_pid.pid_front_left,
		 PID_POSITION,
		 all_motor_pid.pid_back_left.pid_parameter,
		 12000,
		 100);
	
	PID_Init(&all_motor_pid.pid_back_left,
		 PID_POSITION,
		 all_motor_pid.pid_back_left.pid_parameter,
		 12000,
		 100);
		 
	PID_Init(&all_motor_pid.pid_back_right,
		 PID_POSITION,
		 all_motor_pid.pid_back_left.pid_parameter,
		 12000,
		 100);
		 
PID_Init(&all_motor_pid.pid_pitchI_angle,
		 PID_DELTA,
		 all_motor_pid.pid_pitchI_angle.pid_parameter,
		 4000,
		 10);	
 		 
PID_Init(&all_motor_pid.pid_pitchI_speed,
		 PID_POSITION,
		 all_motor_pid.pid_pitchI_speed.pid_parameter,
		 25000, 
		 10);	
		 
PID_Init(&all_motor_pid.pid_yaw_angle,
		 PID_DELTA,
		 all_motor_pid.pid_yaw_angle.pid_parameter,
		4000,
		 10);	
 		 
PID_Init(&all_motor_pid.pid_yaw_speed,
		 PID_POSITION,
		 all_motor_pid.pid_yaw_speed.pid_parameter,
		 25000, 
		 10);		 



		PID_Init(&all_motor_pid.pid_follow_angle,
		 PID_DELTA,
		 all_motor_pid.pid_follow_angle.pid_parameter,
		 1000,
		 2);	
 		 
		PID_Init(&all_motor_pid.pid_follow_speed,
		 PID_POSITION,
		 all_motor_pid.pid_follow_speed.pid_parameter,
		 5000, 
		 10);		 
		 
		 PID_Init(&all_motor_pid.pid_friction_left,
		 PID_POSITION,
		 all_motor_pid.pid_friction_left.pid_parameter,
		 13000, 
		 10);		 
		 
		 PID_Init(&all_motor_pid.pid_friction_right,
		 PID_POSITION,
		 all_motor_pid.pid_friction_right.pid_parameter,
		 13000, 
		 10);		 
		 
		 PID_Init(&all_motor_pid.pid_pop_up_wheel_angle,
		 PID_DELTA,
		 all_motor_pid.pid_pop_up_wheel_angle.pid_parameter,
		 3000, 
		 10);	

		PID_Init(&all_motor_pid.pid_pop_up_wheel_speed,
		 PID_POSITION,
		 all_motor_pid.pid_pop_up_wheel_speed.pid_parameter,
		 7000, 
		 10);		

}

 
void pid_assignment(fp32 arr[3], fp32 p, fp32 i, fp32 d)
{
	arr[0]=p;
	arr[1]=i;
	arr[2]=d;
    
}
