#ifndef friction_H
#define friction_H
#include "main.h"

#define ball_ecd  -49256
#define HEAT_PER_SHOT 10       // 17mm单发消耗热量（J）
#define HEAT_LIMIT_EXTRA 100   // Q2 = Q0 + 100

typedef enum
{
    Friction_OFF,   // 摩擦轮关闭
    Friction_ON,    // 摩擦轮开启
} friction_mode_e;

void friction_task(void *argument);
void friction_set_mode(friction_mode_e *friction_mode);
void friction_set_control(friction_mode_e *friction_mode);

#endif