#ifndef __ROBOMASTER_SHOOT_H
#define __ROBOMASTER_SHOOT_H
#include "robomaster_common.h"
#include "robomaster_userlib.h"
#include "bsp_can.h"
#define SHOOT_CONTROL_TIME 1
#define FRIC1 TIM_CHANNEL_3 //摩擦轮通道
#define FRIC2 TIM_CHANNEL_4 //摩擦轮通道

//射击摩擦轮激光打开 关闭
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E


//鼠标长按判断
#define PRESS_LONG_TIME 200
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME 2000
//摩擦轮高速 加速 时间
#define UP_ADD_TIME 80
//电机反馈码盘值范围
#define Half_ecd_range 4096
#define ecd_range 8191
//电机rmp 变化成 旋转速度的比例
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18



//卡单时间 以及反转时间
#define BLOCK_TIME 500
#define REVERSE_TIME 500
#define REVERSE_SPEED_LIMIT 13.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Ten 0.314f
typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_DONE,
} shoot_mode_e;


typedef struct
{
//    const moto_measure_t *shoot_motor_measure;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    bool_t move_flag;
    uint32_t cmd_time;
    uint32_t run_time;
    bool_t key;
    uint16_t key_time;
    bool_t shoot_done;
    uint8_t shoot_done_time;
    int16_t BulletShootCnt;
    int16_t last_butter_count;
} Shoot_Motor_t;

extern Shoot_Motor_t trigger_motor;          //射击数据
int16_t shoot_control_loop(void);
void shoot_init(void);

//void single_shoot(void);
//void shoot_control(void);
//void single_shoot1(void);//单发程序，使用限位开关
#endif

