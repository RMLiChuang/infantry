/*
* @文件     robot_status.c
 * @说明     机器人状态分类与检测
 * @版本  	 V1.0
 * @作者     izh20
 * @网站     
 * @日期     2018.11.25
*/
#include "robot_status.h"






ROBOT_STATUS_t robot_status;
/**********************************************************************************************************
*函 数 名: PlaceStausCheck
*功能说明: 放置状态检测：静止或运动
*形    参: 角速度
*返 回 值: 无
**********************************************************************************************************/
void PlaceStausCheck(Vector3f_t gyro_status)
{
    Vector3f_t gyroDiff;
    static Vector3f_t lastGyro;
    static float threshold = 1.0f;
    static uint16_t checkNum = 0;
    static int16_t count = 0;

    gyroDiff.x = gyro_status.x - lastGyro.x;
    gyroDiff.y = gyro_status.y - lastGyro.y;
    gyroDiff.z = gyro_status.z - lastGyro.z;
    lastGyro = gyro_status;

    if(count < 100)
    {
        count++;
        //陀螺仪数值变化大于阈值
        if(abs(gyroDiff.x) > threshold || abs(gyroDiff.y) > threshold || abs(gyroDiff.z) > threshold)
        {
            checkNum++;
        }
    }
    else
    {
        //陀螺仪数据抖动次数大于一定值时认为步兵不处于静止状态
        if(checkNum > 1)
            robot_status.placement = MOTIONAL;
        else
            robot_status.placement = STATIC;

        checkNum = 0;
        count = 0;
    }
}
/**********************************************************************************************************
*函 数 名: robot_status_init
*功能说明: 初始化机器人各个状态
*形    参: 角速度
*返 回 值: 无
**********************************************************************************************************/
void robot_status_init()
{
	
	robot_status.mode=INITIAL;
	robot_status.imu_status=HEATING;
	robot_status.anomaly=NORMAL;
	robot_status.placement=MOTIONAL;//初始化时需要让它为运动状态，来进入运动检测，避免陀螺仪获取错误零飘值
	robot_status.vision_status=VISION_LOSE;
	robot_status.chassis_control=CONTROL;
	robot_status.vision_mode=DORMANT;//视觉模式默认休眠
	robot_status.fric_mode=DORMANT;  //摩擦轮状态为休眠
}
/**********************************************************************************************************
*函 数 名: robot_status_init
*功能说明: 初始化机器人各个状态
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void robot_status_detection()
{
	robot_status_display();
	
	if(HAL_GetTick()-dbus_time>20)
	{
		robot_status.anomaly=REMOTE_CONTROL_OFFLINE;//遥控器离线
	}
		
	if(robot_status.anomaly==REMOTE_CONTROL_OFFLINE)//如果遥控器离线
	{
		set_chassis_moto_target_zero();//关闭所有电机
		HAL_GPIO_WritePin(LED_USER_GPIO_PORT, LED_B_Pin,GPIO_PIN_RESET);
	}
	if(HAL_GetTick()-vision_time>500)
	{
		robot_status.vision_status=VISION_LOSE;//视觉检测离线
		HAL_GPIO_WritePin(LED_USER_GPIO_PORT, LED_D_Pin,GPIO_PIN_RESET);
		
	}
		
}

/**********************************************************************************************************
*函 数 名: robot_status_display
*功能说明: oled显示机器人各个状态
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void robot_status_display()
{
	if(robot_status.vision_status==VISION_LOSE)
	{
		oled_showstring(1,4,"LOSE");
	}
	if(robot_status.vision_status==VISION_SUCCESS)
	{
		oled_showstring(1,4,"SUCCESS");
	}
}
