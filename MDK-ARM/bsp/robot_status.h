#ifndef __ROBOT_STATUS_
#define __ROBOT_STATUS_
#include "robomaster_common.h"
#include "vector3.h"

typedef struct
{
    uint8_t  init;           //初始化状态
    uint8_t  failsafe;       //失控保护状态
    uint8_t  armed;          //电机锁定状态
    uint8_t  flight;         //飞行状态
    uint8_t  placement;      //放置状态
    uint8_t  altControl;     //高度控制状态
    uint8_t  posControl;     //位置控制状态
    uint8_t  mode;
    uint32_t initFinishTime; //初始化完成时间
} ROBOT_STATUS_t;


//初始化状态
enum
{
    HEATING,		        //加热中
    HEAT_FINISH,		    //加热完成
    INIT_FINISH             //初始化完成 （完成加速度零偏计算）
};

//放置状态
enum
{
    STATIC,		            //静止
    MOTIONAL			    //运动
};
extern ROBOT_STATUS_t robot_status;
void PlaceStausCheck(Vector3f_t gyro_status);
#endif

