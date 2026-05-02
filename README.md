# 步兵橙min工程 代码架构文档

## 1. 工程概述

本工程是基于 STM32F427 + FreeRTOS 的 RoboMaster 步兵机器人控制系统，实现底盘运动、云台控制、摩擦轮发射等功能。

---

## 2. 目录结构

```
步兵min(橙子)/
├── APP/                    # 应用层任务模块
│   ├── chassis.c/h          # 底盘运动控制
│   ├── gimbal.c/h           # 云台控制（俯仰+偏航）
│   ├── friction.c/h         # 摩擦轮发射控制
│   ├── remote_control.c/h   # 遥控器SBUS协议解析
│   ├── detect_task.c/h      # 设备离线检测任务
│   └── servo.c/h            # 舵机控制
│
├── BSP/                     # 板级支持包（硬件抽象层）
│   ├── can_function.c/h     # CAN总线通信（电机驱动）
│   ├── bsp_rc.c/h           # 遥控器接收驱动
│   ├── bsp_uart.c/h         # 串口通信驱动
│   ├── gyroscope.c/h        # BMI088陀螺仪驱动
│   ├── LED.c/h              # LED指示
│   └── beep.c/h             # 蜂鸣器驱动
│
├── TASK/                    # 公共任务组件
│   ├── pid.c/h              # PID控制器实现
│   └── pid_init.c/h         # PID参数初始化
│
├── support/                 # 通用支持库
│   ├── referee.c/h          # 裁判系统协议解析
│   ├── referee_usart_task.c/h  # 裁判系统串口任务
│   ├── protocol.h           # 裁判系统协议定义
│   ├── fifo.c/h             # FIFO环形缓冲区
│   ├── CRC8_CRC16.c/h       # CRC校验
│   ├── struct_typedef.h     # 基础类型定义
│   └── bsp_usart.c/h        # 串口硬件驱动
│
├── Core/                    # STM32 HAL库与启动
│   ├── Src/
│   │   ├── main.c           # 主程序入口
│   │   ├── freertos.c       # FreeRTOS任务创建
│   │   ├── can.c/h          # CAN外设配置
│   │   ├── usart.c/h        # 串口外设配置
│   │   ├── gpio.c/h         # GPIO配置
│   │   ├── dma.c/h          # DMA配置
│   │   ├── tim.c/h          # 定时器配置
│   │   └── stm32f4xx_it.c   # 中断处理
│   └── Inc/
│
├── Middlewares/             # 第三方中间件
│   └── Third_Party/FreeRTOS/ # FreeRTOS内核
│
├── Drivers/                 # STM32外设驱动
├── CMakeLists.txt          # CMake构建配置
├── project.ioc             # STM32CubeMX工程文件
└── STM32F427XX_FLASH.ld    # 链接脚本
```

---

## 3. 任务列表与优先级

FreeRTOS 任务创建采用"启动任务"模式：`StartDefaultTask` 创建所有应用任务后自我删除。

| 任务名称 | 优先级 | 堆栈大小 | 功能描述 |
|----------|--------|----------|----------|
| gimbal_task | osPriorityNormal3 (最高) | 512 words | 云台俯仰和偏航控制 |
| friction_task | osPriorityNormal2 | 256 words | 摩擦轮发射控制 |
| chassis_task | osPriorityNormal1 | 512 words | 底盘运动控制 |
| referee_usart_task | osPriorityNormal1 | 256 words | 裁判系统数据接收 |
| servo_task | osPriorityNormal1 | 256 words | 舵机PWM控制 |
| detect_task | osPriorityNormal (最低) | 256 words | 设备离线检测 |

---

## 4. 主要模块功能

### 4.1 APP 目录

| 文件 | 功能 |
|------|------|
| **chassis.c** | 底盘电机控制，包含麦轮运动学解算、功率限制、云台跟随模式/小陀螺模式切换 |
| **gimbal.c** | 云台控制，yaw偏航+俯仰角度控制，使用IMU反馈计算角度，双环PID控制 |
| **friction.c** | 摩擦轮+弹轮发射控制，基于裁判系统热量反馈实现发射频率控制 |
| **remote_control.c** | 解析SBUS遥控器协议（18字节），提供通道值/开关状态/鼠标键盘数据 |
| **detect_task.c** | 设备在线检测，通过时间戳判断设备是否离线，支持17种设备检测 |
| **servo.c** | 舵机PWM控制 |

### 4.2 BSP 目录

| 文件 | 功能 |
|------|------|
| **can_function.c/h** | CAN总线通信，支持CAN1(6电机-底盘 yaw super_cap)和CAN2(4电机-pitch 摩擦轮 弹轮) |
| **bsp_rc.c/h** | 遥控器DBUS接收DMA双缓冲配置 |
| **gyroscope.c/h** | BMI088陀螺仪SPI通信，解析66字节数据包，提供IMU角度信息 |
| **LED.c/h** | LED驱动 |
| **beep.c/h** | 蜂鸣器驱动 |

### 4.3 TASK 目录

| 文件 | 功能 |
|------|------|
| **pid.c/h** | PID控制器实现，支持位置式(PID_POSITION)和增量式(PID_DELTA)两种模式 |
| **pid_init.c/h** | 所有电机PID参数初始化，共16个PID控制器 |

---

## 5. 关键数据结构

### 5.1 电机系统结构体

```c
// can_function.h
typedef struct {
    uint16_t ecd;           // 编码器值
    int16_t speed_rpm;      // 转速
    int16_t given_current;  // 给定电流
    uint8_t temperate;      // 温度
    int16_t last_ecd;
    int32_t ecd_value;      // 累计编码器值
    int32_t diff;           // 编码器差值
} motor_measure_temp;

typedef struct {
    can1_motors_t can1_motors; // 底盘4轮 + yaw + super_cap (ID1-6)
    can2_motors_t can2_motors; // pitchI + 摩擦轮x2 + 弹轮 (ID1-3,6)
} motor_system_t;
```

### 5.2 PID控制器

```c
// pid.h
typedef struct {
    uint8_t mode;           // PID_POSITION 或 PID_DELTA
    fp32 Kp, Ki, Kd;        // 比例/积分/微分
    fp32 max_out, max_iout; // 输出限幅/积分限幅
    fp32 set, fdb;          // 设定值/反馈值
    fp32 out, Pout, Iout, Dout;
    fp32 Dbuf[3];           // 微分缓冲
    fp32 error[3];          // 误差历史
} PidTypeDef;
```

### 5.3 遥控器数据结构

```c
// remote_control.h
typedef struct {
    struct { int16_t ch[5]; char s[2]; } rc;    // 5通道+2开关
    struct { int16_t x, y, z; uint8_t press_l, press_r; } mouse;
    struct { uint16_t v; } key;                 // 键盘bit掩码
} RC_ctrl_t;
```

---

## 6. 模块间调用关系

```
                    StartDefaultTask
                    (创建所有任务后删除自身)
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
        ▼                 ▼                 ▼
  ┌──────────┐    ┌──────────┐    ┌──────────────┐
  │chassis   │    │ gimbal   │    │  friction     │
  │ _task    │    │  _task   │    │   _task       │
  └────┬─────┘    └────┬─────┘    └──────┬───────┘
       │                │                 │
       ▼                ▼                 ▼
  CAN_cmd_chassis   CAN_cmd_yow        CAN_cmd_gimbal
  (4轮电机)       CAN_cmd_pitch       (弹轮+摩擦轮)
                        │
                        ▼
               ┌────────────────┐
               │ referee_usart  │
               │     _task      │
               └────────┬───────┘
                        │ 获取裁判系统数据
                        ▼
               ┌────────────────┐
               │  robot_state  │
               │  (功率/热量)   │
               └────────────────┘
```

---

## 7. 主要控制逻辑

### 7.1 底盘控制 (chassis.c)

**模式切换**：
- `CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW`：正常模式，底盘跟随云台相对角度
- `CHASSIS_VECTOR_revolve`：小陀螺模式，底盘自转

**控制流程**：
1. 读取遥控器输入（ch[2]=左右，ch[3]=前后）
2. 根据按键WASD/Q/E/F/R切换模式
3. 云台跟随模式下：使用6020 yaw电机角度做PID跟随
4. 小陀螺模式下：全向移动+自转速度
5. 功率限制：当Power_Buffer<50时，按比例缩放所有电机输出
6. 通过CAN发送4个底盘电机电流值

### 7.2 云台控制 (gimbal.c)

**双环PID控制**：
- 外环：角度环（pitch_flag/yaw_flag目标角度）
- 内环：速度环（电机实际速度 → 最终电流输出）

**控制源**：
- 非鼠标模式：摇杆ch[0]/ch[1]直接叠加到目标角度
- 鼠标模式：鼠标移动相对调整pitch_flag/yaw_flag

### 7.3 摩擦轮+弹轮控制 (friction.c)

**热量控制发射逻辑**：
- 开关量s[0]=2时关闭，s[0]=3时开启
- 热量超过 `heat_limit + 100` 永久锁定发射
- 热量在 `heat_limit` 和 `heat_limit + 100` 之间临时锁定
- 鼠标左键按住且摩擦轮开启且未锁定时，按动态间隔发射

---

## 8. 硬件中断与任务配合

```
USART1_IRQHandler  → 解析SBUS遥控器数据
USART6_IRQHandler  → 裁判系统数据 → FIFO缓冲区
CAN1_RX_IRQHandler → 底盘电机反馈数据
CAN2_RX_IRQHandler → 云台/摩擦轮电机反馈数据
SPI5_IRQHandler    → BMI088陀螺仪数据
TIM2_IRQHandler    → 定时器中断（1ms基础时钟）
```

---

## 9. 共享数据指针

所有任务通过全局指针共享数据：

| 指针 | 类型 | 获取方式 | 用途 |
|------|------|----------|------|
| `RC` | `RC_ctrl_t*` | `get_remote_control_point()` | 遥控器数据 |
| `c/g` | `motor_system_t*` | `get_motor_control_point()` | 电机反馈数据 |
| `imu_g` | `imu_angle_t*` | `gyroscope_control_point()` | IMU角度数据 |

---

## 10. CAN总线分配

| CAN通道 | 电机ID | 功能 |
|---------|--------|------|
| CAN1 | ID1-ID4 | 底盘四个麦轮电机 |
| CAN1 | ID5 | yaw云台电机 |
| CAN1 | ID6 | 超级电容 |
| CAN2 | ID1 | pitch俯仰电机 |
| CAN2 | ID2 | 摩擦轮左侧电机 |
| CAN2 | ID3 | 摩擦轮右侧电机 |
| CAN2 | ID6 | 弹轮电机 |

---

## 11. 开发注意事项

1. **USER CODE 区域**：在 CubeMX 重新生成代码时，只保留 USER CODE BEGIN/END 区域内的代码

2. **任务优先级**：云台任务优先级最高（实时控制），检测任务优先级最低（仅监控）

3. **中断安全**：CAN/USART 中断回调中更新共享数据结构，不涉及任务间通信

4. **功率限制**：底盘任务会根据裁判系统反馈的功率缓冲动态调整输出