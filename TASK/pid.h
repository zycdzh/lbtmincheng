/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "main.h"
enum PID_MODE
{
    PID_POSITION = 0,   //常规算pid     p*error  正负号
    PID_DELTA				//位置pid
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //最大输出
    fp32 max_iout; //最大积分输出

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //微分项 0最新 1上一次 2上上次
    fp32 error[3]; //误差项 0最新 1上一次 2上上次
	
		fp32 pid_parameter[3];
} PidTypeDef;

typedef struct
{
	PidTypeDef  pid_front_right;
	PidTypeDef  pid_front_left;
	PidTypeDef  pid_back_left;  
	PidTypeDef  pid_back_right; 
	PidTypeDef  pid_yaw_angle;  
	PidTypeDef  pid_yaw_speed; 	
	PidTypeDef  pid_super_cap;  
	PidTypeDef  pid_pitchI_angle;
	PidTypeDef  pid_pitchI_speed;
	
	PidTypeDef  pid_friction_left;
	PidTypeDef  pid_friction_right;
	
	PidTypeDef  pid_pop_up_wheel_angle;
	PidTypeDef  pid_pop_up_wheel_speed;
	PidTypeDef pid_follow_angle;
	PidTypeDef pid_follow_speed;


}pid_Motor;


extern void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);
#endif
