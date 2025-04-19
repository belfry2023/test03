#include "car.h"
#include "dji_motor.h"
#include "message_center.h"
#include "car_def.h"
#include "ins_task.h"

static DJIMotorInstance *motor_l, *motor_r; // 左右电机实例指针
static Subscriber_t *car_sub;
static Publisher_t *car_pub; // 发布者实例指针
static car_control_t car_control; // 车的控制指令
static car_state_t car_state; // 车的状态
static float32_t motor_l_ref, motor_r_ref, yaw; // 左右电机的设定值
static PIDInstance follow_to_yaw;
static attitude_t *car_IMU_data; // 云台IMU数据

void car_task_init(void)
{
    car_IMU_data = INS_Init(); // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    Motor_Init_Config_s motor_config = {
        .can_init_config.can_handle = &hcan1,
        .can_init_config.tx_id = 1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000
                },
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 3000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL
        },
        .motor_type = M2006,
    };

    motor_l = DJIMotorInit(&motor_config);
    motor_config.can_init_config.tx_id = 2;
    motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE; // 反转
    motor_r = DJIMotorInit(&motor_config);

    PID_Init_Config_s pid_config = {
        .Kp = 3500,
        .Ki = 0,
        .Kd = 5,
        .Derivative_LPF_RC = 0,
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
        .MaxOut = 15000,
        .DeadBand = 3
    };

    PIDInit(&follow_to_yaw, &pid_config);

    car_sub = SubRegister("car_control", sizeof(car_control_t)); // 订阅车的控制指令
    car_pub = PubRegister("car_state", sizeof(car_state_t)); // 发布车的状态
}

void Motion_calculation(void)
{
    float32_t wz = PIDCalculate(&follow_to_yaw, car_IMU_data->YawTotalAngle, yaw); // 计算偏航角速度
    if(car_control.wz)// 转向时
    {   
        motor_l_ref = car_control.vx + car_control.wz; // 左电机的设定值
        motor_r_ref = car_control.vx - car_control.wz; // 右电机的设定值
        yaw = car_IMU_data->YawTotalAngle;
    }
    else
    {
        motor_l_ref = car_control.vx - wz; // 左电机的设定值
        motor_r_ref = car_control.vx + wz; // 右电机的设定值
    }
}

void car_control_task(void)
{
    Motion_calculation(); // 计算电机的设定值
    DJIMotorSetRef(motor_l, motor_l_ref); // 设置左电机的设定值
    DJIMotorSetRef(motor_r, motor_r_ref); // 设置右电机的设定值
}

void car_task(void)
{
    SubGetMessage(car_sub, &car_control); // 获取车的控制指令
    car_control_task(); // 计算电机的设定值
    PubPushMessage(car_pub, &car_state); // 发布车的状态
}