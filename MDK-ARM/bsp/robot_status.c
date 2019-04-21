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
	robot_status.vision_status=VISION_LOSE;
	robot_status.chassis_control=CONTROL;
	robot_status.vision_mode=DORMANT;//�Ӿ�ģʽĬ������
	robot_status.fric_mode=DORMANT;  //Ħ����״̬Ϊ����
}
/**********************************************************************************************************
*�� �� ��: robot_status_init
*����˵��: ��ʼ�������˸���״̬
*��    ��: 
*�� �� ֵ: ��
**********************************************************************************************************/
void robot_status_detection()
{
	robot_status_display();
	
	if(HAL_GetTick()-dbus_time>20)
	{
		robot_status.anomaly=REMOTE_CONTROL_OFFLINE;//ң��������
	}
		
	if(robot_status.anomaly==REMOTE_CONTROL_OFFLINE)//���ң��������
	{
		set_chassis_moto_target_zero();//�ر����е��
		HAL_GPIO_WritePin(LED_USER_GPIO_PORT, LED_B_Pin,GPIO_PIN_RESET);
	}
	if(HAL_GetTick()-Armour_attack.current_time>100)
	{
		robot_status.vision_status=VISION_LOSE;//�Ӿ��������
		HAL_GPIO_WritePin(LED_USER_GPIO_PORT, LED_D_Pin,GPIO_PIN_RESET);
		
	}
		
}

/**********************************************************************************************************
*�� �� ��: robot_status_display
*����˵��: oled��ʾ�����˸���״̬
*��    ��: 
*�� �� ֵ: ��
**********************************************************************************************************/
void robot_status_display()
{
	oled_showstring(0,1,"Pit:");
	oled_shownum(0,5,imu.pit,0,3);
	oled_showstring(0,9,"Target:");
	oled_shownum(0,16,pan_tilt_pitch.target,0,3);
	
	oled_showstring(1,1,"Yaw:");
	oled_shownum(1,5,imu.yaw,0,3);
	oled_showstring(1,9,"Target:");
	oled_shownum(1,16,pan_tilt_yaw.target,0,3);
	
	oled_showstring(2,1,"Vision:");
	if(robot_status.vision_status==VISION_LOSE)
	{
		oled_showstring(2,8,"lose");
	}
	if(robot_status.vision_status==VISION_SUCCESS)
	{
		oled_showstring(2,8,"success");
	}
	oled_showstring(2,15,"fps:");
	oled_shownum(2,19, Armour_attack.vision_frame,0,2);
	oled_showstring(3,1,"BulletNum:");
	oled_shownum(3,11, trigger_motor.BulletShootCnt,0,3);
	
	
	//oled_shownum(4,18, Armour_attack.delta_time,0,3);
}
