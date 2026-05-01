
#include "stm32f4xx.h"                  // Device header
#include "can_function.h" 
#include "main.h"



motor_system_t motor;


static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

extern CAN_HandleTypeDef hcan1;	
extern CAN_HandleTypeDef hcan2;		


/*can1 和 can2的过滤器配置*/		
void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;	            
    can_filter_st.FilterActivation = ENABLE;			
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;	
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;	
    can_filter_st.FilterIdHigh = 0x0000;				
    can_filter_st.FilterIdLow = 0x0000;					
    can_filter_st.FilterMaskIdHigh = 0x0000;		
    can_filter_st.FilterMaskIdLow = 0x0000;			
		
    can_filter_st.FilterBank = 0;						
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;	
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);		
    HAL_CAN_Start(&hcan1);								
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    can_filter_st.SlaveStartFilterBank = 14;   
    can_filter_st.FilterBank = 14;					
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);		
    HAL_CAN_Start(&hcan2);								
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void open24V(void){

HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

}
void olose24V(void){

HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

}
/*can1的中断回调函数*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
  if (hcan->Instance == hcan1.Instance){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

			 if(rx_header.StdId==0x201)
		 {
			get_motor_measure(&motor.can1_motors.front_left, rx_data);
		 }
		  if(rx_header.StdId==0x202)
		 {
			get_motor_measure(&motor.can1_motors.front_right, rx_data);
		 }
		 	if(rx_header.StdId==0x203)
		 {
			get_motor_measure(&motor.can1_motors.back_left, rx_data);
		 }
		  if(rx_header.StdId==0x204)
		 {
			get_motor_measure(&motor.can1_motors.back_right, rx_data);
		 }
		 if(rx_header.StdId==0x205)
		 {
			get_motor_measure(&motor.can1_motors.yaw, rx_data);
			get_motor_ecd(&motor.can1_motors.yaw); 
		 }
		 if(rx_header.StdId==0x206)
		 {
			get_motor_measure(&motor.can1_motors.super_cap, rx_data);
		 }
	 }
 
  	if (hcan->Instance == hcan2.Instance){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
		if(rx_header.StdId==0x206)
		 {
			get_motor_measure(&motor.can2_motors.pitchI, rx_data);
		 }
		 if(rx_header.StdId==0x202)
		 {
			get_motor_measure(&motor.can2_motors.friction_left, rx_data);
		 }
		 if(rx_header.StdId==0x203){
			get_motor_measure(&motor.can2_motors.friction_right, rx_data);
		 }
		  if(rx_header.StdId==0x201){
			get_motor_measure(&motor.can2_motors.pop_up_wheel, rx_data);
			get_motor_ecd(&motor.can2_motors.pop_up_wheel);	
		 }
	}
}
		


void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	
}

void CAN_cmd_yow(int16_t motor1)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x1FF;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x02;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    

    HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	
}

void CAN_cmd_gimbal(int16_t motor1, int16_t motor2,int16_t motor3)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	
}

void CAN_cmd_pitch(int16_t motor2)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x1FF;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x04;
		chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
   
    

    HAL_CAN_AddTxMessage(&hcan2, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
	
}
void get_motor_ecd(motor_measure_temp *v){

		
	v->diff = v->ecd - v->last_ecd;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		//	v->ecd_raw_rate = v->diff + 8191;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		//v->ecd_raw_rate = v->diff - 8191;
	}		
	else
	{
		//v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->ecd + v->round_cnt * 8191;



}
 const motor_system_t *get_motor_control_point(void)
{
    return &motor;
}





