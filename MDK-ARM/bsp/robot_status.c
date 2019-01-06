/*
* @�ļ�     robot_status.c
 * @˵��     ������״̬��������
 * @�汾  	 V1.0
 * @����     izh20
 * @��վ     
 * @����     2018.11.25
*/
#include "robot_status.h"






ROBOT_STATUS_t robot_status;
/**********************************************************************************************************
*�� �� ��: PlaceStausCheck
*����˵��: ����״̬��⣺��ֹ���˶�
*��    ��: ���ٶ�
*�� �� ֵ: ��
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
        //��������ֵ�仯������ֵ
        if(abs(gyroDiff.x) > threshold || abs(gyroDiff.y) > threshold || abs(gyroDiff.z) > threshold)
        {
            checkNum++;
        }
    }
    else
    {
        //���������ݶ�����������һ��ֵʱ��Ϊ���������ھ�ֹ״̬
        if(checkNum > 1)
            robot_status.placement = MOTIONAL;
        else
            robot_status.placement = STATIC;

        checkNum = 0;
        count = 0;
    }
}
/**********************************************************************************************************
*�� �� ��: robot_status_init
*����˵��: ��ʼ�������˸���״̬
*��    ��: ���ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
void robot_status_init()
{
	
	robot_status.mode=INITIAL;
	robot_status.imu_status=HEATING;
	robot_status.anomaly=NORMAL;
	robot_status.placement=MOTIONAL;//��ʼ��ʱ��Ҫ����Ϊ�˶�״̬���������˶���⣬���������ǻ�ȡ������Ʈֵ
}
/**********************************************************************************************************
*�� �� ��: robot_status_init
*����˵��: ��ʼ�������˸���״̬
*��    ��: ���ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
void robot_status_detection()
{
	if(HAL_GetTick()-dbus_time>20)
	{
		robot_status.anomaly=REMOTE_CONTROL_OFFLINE;
	}
		
	if(robot_status.anomaly==REMOTE_CONTROL_OFFLINE)
	{
		set_chassis_moto_target_zero();
		HAL_GPIO_WritePin(LED_USER_GPIO_PORT, LED_B_Pin,GPIO_PIN_RESET);
	}
		
}



