/*************************************************************************
 * Copyright (c) 2018, 力创RM 
 * All rights reserved.
 * 
 * File name    :   referee_sys.h
 * Brief        :   referee_sys.c的头文件
 * Revision     :   1.0
 * Author       :   九月阁
 * Date         :   2018.03.24
 修改：2019.1.14，加入不同数据标志位
*************************************************************************/
#ifndef __REFEREE_SYS_H
#define __REFEREE_SYS_H

/* Includes -----------------------------------------------------------*/
#include "usart.h"

/* 宏定义数据最大长度*/
#define datamaxlen 255
/* 宏定义数据包头*/
#define DataHeader 0xA5


/* 宏定义数据包CMD*/
#define RM_status      0x01
#define RM_damage_data 0x02
#define RM_shoot_data  0x03
#define RM_WQ_data     0x04
#define RM_RFID_data   0x05
#define Match_results  0x06
#define Buff_status    0x07
#define RM_self_data   0x08

/* 宏定义数据包CMD*/
#define CMD1_LEN      8
#define CMD2_LEN      1
#define CMD3_LEN      6
#define CMD4_LEN     20
#define CMD5_LEN      2
#define CMD6_LEN      1
#define CMD7_LEN      2
#define CMD8_LEN     16

#define cal_cmd_len(x)    5 + 2 + x + 2     //FH = 5 2个CMD x个长度的数据 2个CRC16校验

/* 比赛机器人状态--0x001*/
typedef __packed struct
{
	uint16_t stageremaintime;   //当前剩余时间，单位S

	uint16_t gameprogress;      //比赛进程：0-未开始比赛；1-准备阶段；2-自检阶段；3-5s倒计时；4-对战阶段；5-比赛结算中

  uint8_t  robotlevel;        //机器人当前等级

	uint16_t remainHP;          //机器人当前血量

  uint16_t maxHP;             //机器人满血量


}extGameRobotState_t;



/* 伤害数据--0x002
armorType 装甲ID(0-3bit)
0x0--0号装甲（前）
0x1--1号装甲（左）
0x2--2号装甲（后）
0x3--3号装甲（右）
0x4--4号装甲（上1）
0x5--5号装甲（上2）

hurtType   伤害类型(4-7bit)
0x0-装甲伤害
0x1-模块掉线

*/
typedef __packed struct
{

    uint8_t  armorType;
    uint8_t  hurtType;

}extRobotHurt_t;


/* 实时射击信息--0x003
bulletType：弹丸类型
1: 17mm弹丸
2: 43mm弹丸

bulletFreq：发每秒

bulletSpeed：米每秒
*/
typedef __packed struct
{

    uint8_t  bulletType;
    uint8_t  bulletFreq;
    float bulletSpeed;

}extShoot_t;


/* 实时功率热量数据--0x004
chassisVolt:底盘输出电压 伏
chassisCurrent:底盘输出电流 安
chassisPower:底盘输出功率 瓦
chassisPowerBuffer:底盘功率缓冲 瓦
枪口热量
ShooterHeat_17mm:
ShooterHeat_42mm:
*/
typedef __packed struct
{
	float chassisVolt;
    float chassisCurrent;
    float chassisPower;
    float chassisPowerBuffer;

    uint16_t  ShooterHeat_17mm;
    uint16_t  ShooterHeat_42mm;


}extPowerHeatData_t;

/* 场地交互数据--0x005
cardType：卡类型
0：攻击加成卡
1：防御加成卡
2：红方加血卡
3：蓝方加血卡
4：红方治疗卡
5：蓝方治疗卡
6：红方冷却卡
7：蓝方冷却卡
8：碉堡卡
9：保留
10：资源岛卡
11：ICRA大能量机关打击点卡

cardldx：卡索引号
*/
typedef __packed struct
{
    uint8_t  cardType;
    uint8_t  cardldx;

}extRfidDetect_t;

/* 比赛胜负数据--0x006
0:平局
1:红方胜
2:蓝方胜
*/
typedef __packed struct
{
    uint8_t  winner;

}extGameResult_t;

/* Buff获取数据--0x007
bit0: 补血点回血
bit1: 工程机器人回血
bit2: 治疗卡回血
bit3: 资源岛防御
bit4: 己方激活最大能量机关
bit5: 敌方激活最大能量机关
bit6: 己方激活最小能量机关
bit7: 敌方激活最小能量机关
bit8: 加速冷却
bit9: 碉堡防御
bit10: 百分百防御
bit11: 无哨兵基地防御
bit12: 有哨兵基地防御

*/
typedef __packed struct
{

    uint16_t  buffMusk;

}extBuffMusk_t;



/* 机器人位置朝向消息--0x008
字节偏移
0       --位置x米
4       --位置y米
8       --位置z米
12      --枪口朝向角度值，度
*/
typedef __packed struct
{
	  float x;
    float y;
    float z;
    float yaw;

}extGameRobotPos_t;

typedef __packed struct
{
    extGameRobotState_t GameRobotState_t;

    extRobotHurt_t RobotHurt_t;

    extShoot_t Shoot_t;

    extPowerHeatData_t PowerHeatData_t;

    extRfidDetect_t RfidDetect_t;

    extBuffMusk_t BuffMusk_t;

    extGameRobotPos_t GameRobotPos_t;

}refDataStruct;

/* Declaration ------------------函数申明--------------------------------------*/
void get_referee_data(void);
void uart_receive_idle(UART_HandleTypeDef *huart);
void ref_sys_init(void);
void display(void);
extern refDataStruct  refSysData; 
#endif

