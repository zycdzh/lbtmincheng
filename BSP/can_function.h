/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.h
 * @brief      this file contains sd card basic operating function
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-28-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __CAN_FUNCTION__H
#define __CAN_FUNCTION__H

#include "stm32f4xx.h"
#include <stdint.h>

#include "main.h"


typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
	
		int32_t ecd_value; //编码连续值
		int32_t diff;							     //两次编码器之间的差值
		int32_t temp_count;                      //计数用
		int32_t round_cnt;		
	
}motor_measure_temp;



typedef struct {         
    motor_measure_temp front_left;  // 底盘前左 (ID1)
    motor_measure_temp front_right;   // 底盘前左 (ID2)
    motor_measure_temp back_left;    // 底盘后右 (ID3)
    motor_measure_temp back_right;   // 底盘后右 (ID4)
    motor_measure_temp yaw;          // yaw (ID5)
    motor_measure_temp super_cap;    // 超级电容 (ID6)
} can1_motors_t;

// CAN2 总线的设备结构体（包含 4 个设备）
typedef struct {
    motor_measure_temp pitchI;      // pitchI (ID6)
    motor_measure_temp friction_left; // 摩擦轮左 (ID1)
    motor_measure_temp friction_right; // 摩擦轮右 (ID2)
    motor_measure_temp pop_up_wheel; // 拨弹轮 (ID3)
} can2_motors_t;

// 电机系统结构体，管理整个系统的电机信息
typedef struct {
    can1_motors_t can1_motors; // 存储 CAN1 上 6 个设备的信息
    can2_motors_t can2_motors; // 存储 CAN2 上 4 个设备的信息
} motor_system_t;


void can_filter_init(void);
void open24V(void);
void olose24V(void);
const motor_measure_temp *get_3508_motor_measure_point(uint8_t i);
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN_cmd_yow(int16_t motor1);
void CAN_cmd_gimbal(int16_t motor1, int16_t motor2,int16_t motor3);
void get_motor_ecd(motor_measure_temp *v);
extern const motor_system_t *get_motor_control_point(void);
void CAN_cmd_pitch(int16_t motor2);
#endif
