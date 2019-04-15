#ifndef __ROBOMASTER_SHOOT_H
#define __ROBOMASTER_SHOOT_H
#include "robomaster_common.h"
#include "robomaster_userlib.h"
#include "bsp_can.h"
#define SHOOT_CONTROL_TIME 1
#define FRIC1 TIM_CHANNEL_3 //Ħ����ͨ��
#define FRIC2 TIM_CHANNEL_4 //Ħ����ͨ��

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E


//��곤���ж�
#define PRESS_LONG_TIME 200
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME 2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME 80
//�����������ֵ��Χ
#define Half_ecd_range 4096
#define ecd_range 8191
//���rmp �仯�� ��ת�ٶȵı���
#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18



//����ʱ�� �Լ���תʱ��
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

extern Shoot_Motor_t trigger_motor;          //�������
int16_t shoot_control_loop(void);
void shoot_init(void);

//void single_shoot(void);
//void shoot_control(void);
//void single_shoot1(void);//��������ʹ����λ����
#endif

