#include "friction.h"
#include "main.h"
#include "pid.h"

// 电机速度变量
fp32 i_1 = 0;               // 弹轮电机速度
fp32 i_2 = 0;               // 摩擦轮右侧电机速度
fp32 i_3 = 0;               // 摩擦轮左侧电机速度
fp32 i_1_angle = 0;         // 弹轮电机角度

extern pid_Motor all_motor_pid;  // 所有电机PID控制器

const RC_ctrl_t *RC_f;           // 遥控器控制指针
const motor_system_t *f;         // 电机系统控制指针
friction_mode_e friction_mode;   // 射击模式

int32_t pop_up_wheel_ecd = 0;    // 弹轮编码器值

// 热量发射的时间变量
static uint32_t last_fire_time = 0;  // 上次发射时间戳（ms）
static uint8_t firing_locked = 0;   // 锁定标志（0=未锁定，1=临时锁定，2=永久锁定）

// 函数声明
void motor_Stalled(const motor_system_t *f, fp32 *speed);
void friction_task(void *argument);
void friction_set_mode(friction_mode_e *friction_mode);
void friction_set_control(friction_mode_e *friction_mode);

// 主任务函数：摩擦轮任务
void friction_task(void *argument) {
    // 获取遥控器和电机控制地址
    RC_f = get_remote_control_point();
    f = get_motor_control_point();

    while (1) {
        // 设置摩擦轮模式和控制
        friction_set_mode(&friction_mode);
        friction_set_control(&friction_mode);
        motor_Stalled(f, &i_1);

        // 如果遥控器出错，停止电机
        if (toe_is_error(DBUS_TOE)) {
            CAN_cmd_gimbal(0, 0, 0);
        } else {
            // 发送电机控制命令
            CAN_cmd_gimbal(i_1, i_2, i_3);
        }

        osDelay(2);  // 延迟2ms
    }
}

// 设置摩擦轮模式
void friction_set_mode(friction_mode_e *friction_mode) {
    // 根据遥控器状态设置摩擦轮模式
    if (RC_f->rc.s[1] != 0x02) {
        if (RC_f->rc.s[0] == 0x02) *friction_mode = Friction_OFF;
        if (RC_f->rc.s[0] == 0x03) *friction_mode = Friction_ON;
    } else {
        if ((RC_f->key.v & KEY_PRESSED_OFFSET_Q) && (*friction_mode != Friction_OFF)) {
            *friction_mode = Friction_OFF;
        }
        if ((RC_f->key.v & KEY_PRESSED_OFFSET_E) && (*friction_mode != Friction_ON)) {
            *friction_mode = Friction_ON;
        }
    }
}

// 根据摩擦轮模式设置电机速度
void friction_set_control(friction_mode_e *friction_mode) {
    // 摩擦轮关闭时，设置电机速度为0
    if (*friction_mode == Friction_OFF) {
        i_2 = PID_Calc(&all_motor_pid.pid_friction_right, f->can2_motors.friction_right.speed_rpm, 0);
        i_3 = PID_Calc(&all_motor_pid.pid_friction_left, f->can2_motors.friction_left.speed_rpm, 0);
    }
    // 摩擦轮开启时，设置电机速度为7000或-7000
    if (*friction_mode == Friction_ON) {
        i_2 = PID_Calc(&all_motor_pid.pid_friction_right, f->can2_motors.friction_right.speed_rpm, 7000);
        i_3 = PID_Calc(&all_motor_pid.pid_friction_left, f->can2_motors.friction_left.speed_rpm, -7000);
    }
}

// 弹轮发射逻辑：基于热量计算发射频率
void motor_Stalled(const motor_system_t *f, fp32 *speed) {
    uint16_t heat_limit;      // Q0 - 热量上限
    uint16_t current_heat;    // Q1 - 当前热量
    uint16_t cooling_rate;    // 每秒冷却速度
    uint16_t heat_threshold;  // Q2 = Q0 + 100

    // 从裁判系统获取热量数据
    get_shoot_heat0_limit_and_heat0(&heat_limit, &current_heat);
    cooling_rate = robot_state.shooter_heat0_cooling_rate;

    // 计算Q2阈值（Q0 + 100，17mm发射机构）
    heat_threshold = heat_limit + HEAT_LIMIT_EXTRA;

    // 检查发射锁定状态
    if (current_heat >= heat_threshold) {
        // 永久锁定 - 超过Q2
        firing_locked = 2;
    } else if (current_heat > heat_limit) {
        // 临时锁定 - 在Q0和Q2之间
        if (firing_locked == 0) {
            firing_locked = 1;
        }
    } else {
        // 低于Q0 - 可以发射，如果是临时锁定则解除
        if (current_heat == 0 && firing_locked == 1) {
            firing_locked = 0;
        }
    }

    // 获取当前时间用于发射间隔计算（使用tick值，单位ms）
    uint32_t current_time = xTaskGetTickCount();

    // 发射逻辑 - 按住鼠标左键触发发射
    if (friction_mode == Friction_ON && RC_f->mouse.press_l) {
        // 检查是否可以发射（未锁定且时间间隔足够）
        if (firing_locked == 0) {
            // 计算发射间隔：基础间隔 × (当前热量 / 热量上限) 的影响
            // 热量越低，间隔越小（发射越快）；热量越高，间隔越大（发射越慢）
            // 基础间隔 = 10000 / cooling_rate ms
            // 当 current_heat 接近 heat_limit 时，interval 会变大以匹配冷却速度
            if (cooling_rate == 0) {
                return;
            }
            fp32 heat_ratio = (fp32)current_heat / heat_limit;
            uint32_t base_interval = 10000 / cooling_rate;
            uint32_t bullet_interval = (uint32_t)(base_interval * (1.0f + heat_ratio * 2.0f));

            // 检查距离上次发射是否已经超过间隔时间
            if (current_time - last_fire_time >= pdMS_TO_TICKS(bullet_interval)) {
                // 发射一发
                pop_up_wheel_ecd = f->can2_motors.pop_up_wheel.ecd_value;
                int bullet_num = 1;
                if (*speed < -6000) {
                    bullet_num = -1;
                }
                pop_up_wheel_ecd += bullet_num * ball_ecd;
                last_fire_time = current_time;
            }
        }
    }

    // 计算弹轮角度和速度，并更新速度值
    i_1_angle = (float)PID_Calc(&all_motor_pid.pid_pop_up_wheel_angle, f->can2_motors.pop_up_wheel.ecd_value, pop_up_wheel_ecd);
    *speed = (float)PID_Calc(&all_motor_pid.pid_pop_up_wheel_speed, f->can2_motors.pop_up_wheel.speed_rpm, i_1_angle);
}