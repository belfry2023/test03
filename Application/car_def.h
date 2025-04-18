#include "ins_task.h"
#include "stdint.h"

#pragma pack(1)
typedef struct car_state_s
{
    float vx; // 速度
    float wz; // 旋转角度
} car_state_t;

typedef struct car_control_s
{
    float vx; // 速度
    float wz; // 旋转
} car_control_t;
#pragma pack()
