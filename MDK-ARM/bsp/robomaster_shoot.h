#ifndef __ROBOMASTER_SHOOT_H
#define __ROBOMASTER_SHOOT_H
#include "robomaster_common.h"
#include "robomaster_userlib.h"
#include "bsp_can.h"
#define SHOOT_CONTROL_TIME 1
#define FRIC1 TIM_CHANNEL_3 //摩擦轮通道
#define FRIC2 TIM_CHANNEL_4 //摩擦轮通道
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




void single_shoot(void);
void shoot_control(void);
void single_shoot1(void);//单发程序，使用限位开关
#endif

