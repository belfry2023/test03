#include "remote_control.h"
#include "cmd.h"
#include "message_center.h"
#include "controller.h"

#include "car_def.h"
static RC_ctrl_t *rc_data;              // 遥控器数据,初始化时返回
static Publisher_t *car_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *car_feed_sub;          // 云台反馈信息订阅者
static car_control_t car_cmd_send;      // 传递给云台的控制信息
static car_state_t car_fetch_data; // 从云台获取的反馈信息

void cmd_task_init(void)
{
    rc_data = RemoteControlInit(&huart3);
    
    car_cmd_pub = PubRegister("car_control", sizeof(car_control_t));
    car_feed_sub = SubRegister("car_state", sizeof(car_state_t));
}

void Accept_unpacking(void)
{
    if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[下],底盘和云台分离,底盘保持不转动
    {
        // nav_recv_data->vx = 0;
        // nav_recv_data->vy = 0;
        // nav_recv_data->wz = 0;
    }else
    if (switch_is_mid(rc_data[TEMP].rc.switch_left)) // 左侧开关状态[中],导航模式 && (vision_recv_data->yaw || vision_recv_data->pitch)
    {
        
        // car_cmd_send.vy = nav_recv_data->vx * 19500; // _水平方向
        // car_cmd_send.vx = nav_recv_data->vy * 19500; // 1数值方向
        // car_cmd_send.wz = nav_recv_data->wz;
    }
    // 底盘参数,目前没有加入小陀螺(调试似乎暂时没有必要),系数需要调整
    if(rc_data[TEMP].rc.rocker_l1 || rc_data[TEMP].rc.rocker_l_ || switch_is_down(rc_data[TEMP].rc.switch_left)){
        car_cmd_send.vx = + 26 * (float)rc_data[TEMP].rc.rocker_l_; // _水平方向
        car_cmd_send.wz = + 26 * (float)rc_data[TEMP].rc.rocker_l1; // 1数值方向
    }
    if(car_cmd_send.wz)
    {
        
    }
}


void cmd_task(void)
{
    SubGetMessage(car_feed_sub, &car_fetch_data);



    PubPushMessage(car_cmd_pub, &car_cmd_send);
}