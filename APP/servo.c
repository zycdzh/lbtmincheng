#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "servo.h"
uint16_t pulse=650;
	
const RC_ctrl_t *RC_c; 

  
 
void servo_task(void*argument)
{
	     RC_c = get_remote_control_point();
	
HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
 __HAL_TIM_SetCompare (&htim5,TIM_CHANNEL_3,650);

	
while (1){
	if(RC_c->rc.s[1]==0x02)
		{
	
		if (RC_c->rc.ch[0]==-660)
	{
      	pulse=1900;
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,pulse);
		
		
	}
	if (RC_c->key.v & KEY_PRESSED_OFFSET_C)
	{
      	pulse=1900;
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,pulse);
		
		
	}
	if(RC_c->rc.ch[0]==660) 
		{
		
		pulse=650;
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,pulse);
		}
	
   

		if(RC_c->key.v & KEY_PRESSED_OFFSET_V) 
		{
		
		pulse=650;
			__HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,pulse);
		}
	}
		 osDelay(2);
	
  }  
}      
