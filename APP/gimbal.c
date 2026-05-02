#include "main.h"
#include "pid.h"
#include "gimbal.h"

#define YUNTAI 4060  // 定义常量YUNTAI，可能用于云台角度计算

// 云台角度和输出变量
float yaw_flag = 0;  // 云台的偏航角标志
float pitch_flag = 0;  // 云台的俯仰角标志
float pitch_out1 = 0;  // 云台俯仰角的PID输出1
int pitch_out2 = 0;  // 云台俯仰角的PID输出2（控制命令）
float yaw_out1 = 0;  // 云台偏航角的PID输出1
int yaw_out2 = 0;  // 云台偏航角的PID输出2（控制命令）

float test;  // 用于存储角度计算结果



const RC_ctrl_t *RC_g;  // RC遥控器数据
const motor_system_t *g;  // 电机控制数据
const imu_angle_t *imu_g;  // IMU传感器角度数据
extern pid_Motor all_motor_pid;  // 所有电机的PID控制器实例

// 计算角度函数
float compute_angle(const motor_system_t *motor , int middle);

void gimbal_task(void *argument) {
    // 获取遥控器、IMU传感器和电机控制点数据
    g = get_motor_control_point();
    RC_g = get_remote_control_point();
    imu_g = gyroscope_control_point();

    osDelay(20);  // 延时20ms，等待数据更新

    while (1) {
        // 云台控制函数，设置云台目标角度
        gimbal_set_control();

        // 云台速度控制函数，根据PID调整速度
        gimbal_set_v(); 

        // 如果DBUS有错误，停止云台运动
        if (toe_is_error(DBUS_TOE)) {
            CAN_cmd_yow(0);  // 停止偏航运动
            CAN_cmd_pitch(0);  // 停止俯仰运动
        } else {
            // 发送偏航和俯仰控制命令到云台
            CAN_cmd_yow(yaw_out2);
            CAN_cmd_pitch(pitch_out2);
        }

        osDelay(2);  // 延时2ms，等待下一周期
    }
}

// 云台角度控制函数，根据遥控器输入调节角度
void gimbal_set_control(void) {
    // 如果RC遥控器的pitch控制通道（RC_g->rc.s[1]）不是0x02，进行常规控制
    if (RC_g->rc.s[1] != 0x02) {
        // 调节俯仰角度（pitch），遥控器的y轴控制
        if (RC_g->rc.ch[1] > 50) {
            pitch_flag = pitch_flag + (RC_g->rc.ch[1] * 1.0 / 5);
        }
        if (RC_g->rc.ch[1] < -50) {
            pitch_flag = pitch_flag + (RC_g->rc.ch[1] * 1.0 / 5);
        }

        // 调节偏航角度（yaw），遥控器的x轴控制
        if (RC_g->rc.ch[0] < -15) {
            yaw_flag = yaw_flag + (-RC_g->rc.ch[0] * 1.0 / 10) + 0.4;
        }
        if (RC_g->rc.ch[0] > 15) {
            yaw_flag = yaw_flag + (-RC_g->rc.ch[0] * 1.0 / 10) - 0.4;
        }
    } else {
        // 如果RC遥控器处于鼠标模式，根据鼠标输入调整云台角度
        if (RC_g->mouse.y > 5) {
            pitch_flag = pitch_flag - RC_g->mouse.y ;  // 控制俯仰角度
        }
        if (RC_g->mouse.y < -5) {
            pitch_flag = pitch_flag - RC_g->mouse.y ;
        }

        if (RC_g->mouse.x > 10) {
            yaw_flag = yaw_flag - (RC_g->mouse.x - 10) / 5;  // 控制偏航角度
        }
        if (RC_g->mouse.x < -10) {
            yaw_flag = yaw_flag - (RC_g->mouse.x + 10) / 5;
        }
    }

    // 限制俯仰角度在有效范围内
    if (pitch_flag > 18000) pitch_flag = 18000;
    if (pitch_flag < -10000) pitch_flag = -10000;
}


 fp32	yaw_real_filtered=0 ;
	fp32			gz_filtered=0 ;
// 云台速度控制函数，计算PID控制输出，控制电机速度
void gimbal_set_v(void) {
    // 计算云台当前的角度
	
    test = compute_angle(g, YUNTAI);

    // 计算俯仰角度的PID输出，控制俯仰速度
    pitch_out1 = (float)PID_Calc(&all_motor_pid.pid_pitchI_angle, test, pitch_flag / 1000);
    pitch_out2 = (int)PID_Calc(&all_motor_pid.pid_pitchI_speed, g->can2_motors.pitchI.speed_rpm, -pitch_out1);

    // 计算偏航角度的PID输出，控制偏航速度
    // 定义低通滤波系数
 // 低通滤波系数，可根据需求调整

// 低通滤波函数


// 在 IMU 数据更新时进行滤波
	
	
 

		
// 使用滤波后的数据进行 PID 计算
yaw_out1 = (float)PID_Calc(&all_motor_pid.pid_yaw_angle, imu_g->yaw_real, yaw_flag/100 );
yaw_out2 = (int)PID_Calc(&all_motor_pid.pid_yaw_speed, imu_g->gz, yaw_out1);
}



// 计算云台角度函数，根据电机反馈和中间值计算角度
float compute_angle(const motor_system_t *motor, int middle) {
    return (middle - motor->can2_motors.pitchI.ecd) / 22.766666;
}
