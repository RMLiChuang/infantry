/**
  *@file robomaster_control.c
  *@date 2018-10-5
  *@author izh20
  *@brief 
  */


#include "robomaster_control.h"
#include "robomaster_common.h"

#define twist_speed 1000

void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty)
	{
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (10000*duty) - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = (10000*duty) - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = (10000*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (10000*duty) - 1;break;
	}
	
}

void shoot_control()
{
	if(remote_control.switch_left!=3)
		{
				if(remote_control.switch_right==3)
				{
				  //motor_pid[6].target=4000;
					//motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					//set_rammer_current(&hcan1,motor_pid[6].output);
					//set_rammer_current(&hcan1,0);
					HeadTxData[4]=0;
					HeadTxData[5]=0;//�����ֵ���ֵ
					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.16);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.16);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.16);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.16);
				}
				if(remote_control.switch_right==2)
				{	
					motor_pid[6].target=2500;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//�����������ֵ
					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
					//set_rammer_current(&hcan1,motor_pid[6].output);
					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.14);
				}
				if(remote_control.switch_right==1)
				{
					HeadTxData[4]=0;
					HeadTxData[5]=0;//�����ֵ���ֵ
					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
					//set_rammer_current(&hcan1,0);
					init_TIM5_PWM();
				}

		}else
		{
			HeadTxData[4]=0;
			HeadTxData[5]=0;//�����ֵ���ֵ
			//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
			//set_rammer_current(&hcan1,0);
			init_TIM5_PWM();
		}
}

//IMU_Type chassis_imu;
/**********************************************************************************************************
*�� �� ��: chassis_control
*����˵��: ���̿��Ƴ��򣬼�����imu��z��(yaw��)�ĽǶȻ��ͽ��ٶȻ����ƣ����̼����ٶȻ����п���
*��    ��: ��Ҫyaw��Ƕȣ����ٶȣ�����ٶȷ���
*�� �� ֵ: �������
**********************************************************************************************************/
extern int16_t moto_ctr[6];
int32_t set_spd = 0;//�ٶȲ���
int32_t turn=0;     //ת��
long yaw_flag=0;
extern int cnt1;
extern int cnt2;
char cnt_steer=0;
int yaw_cnt=0;
int chassis_yaw_correct=0;
void chassis_control()
{
	if(remote_control.switch_right!=3)
	{
//			if(cnt1==100)//0.5s����һ�Σ�ʹ��4��led��2HZƵ����˸���жϵ��̳�����������
//			{
//					HAL_GPIO_TogglePin(LED_USER_GPIO_PORT,LED_G_Pin);
//					cnt1=0;
//			}
			Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_G_Pin,1);//����ledG��˸Ƶ��
			DBUS_Deal();//��ȡң���������ݲ������ݸ�ֵ�������Ŀ��ת��
			
			cnt_steer++;
			if(cnt_steer==4)
			{
				steer_control();
				cnt_steer=0;
			}
			
			
			if(remote_control.switch_right==2)	
			{
          /*************************yaw�����    begin**********/		
	
					if(remote_control.ch3==0)//ƫ����������λ
					{
//						if(yaw_cnt<500)//�����ϵ��һ��ʱ������ƫ���ǣ������ơ��������ں���Ҫһ��ʱ�䣬����ȡ500
//						yaw_cnt++;
							if(chassis_yaw.target==0)  //����ʱ���Ƕ�����ֵ
							{
								chassis_yaw.target=imu.yaw;
							}
							PID_Control_Yaw(&chassis_yaw,imu.yaw);//�ú������0�ȵ�360�ȵ�ͻ��
								//chassis_yaw.f_cal_pid(&chassis_yaw,imu.yaw);    //ƫ���Ƕȿ���
								chassis_yaw_speed.target=chassis_yaw.output;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
					}
			
					else//����ƫ������˺�ֻ�����ڻ����ٶȿ���
					{
							chassis_yaw.target=0;//ƫ����������0,�����нǶȿ���
							chassis_yaw_speed.target=remote_control.ch3*300/660;//yaw_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
					}
		
					/*************************yaw�����  end**********/		
			
					motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //�����趨ֵ����PID���㡣
					motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //�����趨ֵ����PID���㡣        �ٶ�Ϊ����ֵ
					motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //�����趨ֵ����PID���㡣
					motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //�����趨ֵ����PID���㡣
	
	

					chassis_yaw.f_cal_pid(&chassis_yaw_speed,-imu.gz);	
	
//					set_moto_current(&hcan1,motor_pid[0].output+chassis_yaw_speed.output,   //��PID�ļ�����ͨ��CAN���͵����
//																	motor_pid[1].output+chassis_yaw_speed.output,
//																	motor_pid[2].output+chassis_yaw_speed.output,
//																	motor_pid[3].output+chassis_yaw_speed.output);
			}
			
			
		if(remote_control.switch_right==1)  //ң����������ֱֵ����Ϊ�ٶ�Ŀ��
		{
			
			
			
			

						motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //�����趨ֵ����PID���㡣
						motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //�����趨ֵ����PID���㡣        �ٶ�Ϊ����ֵ
						motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //�����趨ֵ����PID���㡣
						motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //�����趨ֵ����PID���㡣
						set_moto_current(&hcan1,motor_pid[0].output,   //��PID�ļ�����ͨ��CAN���͵����
															motor_pid[1].output,
															motor_pid[2].output,
															motor_pid[3].output);
			//set_moto_current(&hcan1,500,500,0,500);
		}
	
	}
		else
		{
			//chassis_yaw.target=imu.yaw;  //�ϵ��궨���̵ĳ�ʼyawֵ��
		  set_moto_current(&hcan1,0,0,0,0);
	  }
}
/**********************************************************************************************************
*�� �� ��: chassis_speed_control
*����˵��: �����ٶȻ�����
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
void chassis_speed_control()
{
//		DBUS_Deal();//��ȡң���������ݲ������ݸ�ֵ�������Ŀ��ת��
		motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //�����趨ֵ����PID���㡣
		motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //�����趨ֵ����PID���㡣        �ٶ�Ϊ����ֵ
		motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //�����趨ֵ����PID���㡣
		motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //�����趨ֵ����PID���㡣
	
//		//�����̵�����ٶȽ��뷢��
//	MotorTxData[0] = ((motor_pid[0].output>>8)&0xFF);
//	MotorTxData[1] = (motor_pid[0].output&0xFF); 
//	MotorTxData[2] = ((motor_pid[1].output>>8)&0xFF);
//	MotorTxData[3] = (motor_pid[1].output&0xFF); 
//	MotorTxData[4] = ((motor_pid[2].output>>8)&0xFF);
//	MotorTxData[5] = (motor_pid[2].output&0xFF); 
//	MotorTxData[6] = ((motor_pid[3].output>>8)&0xFF);
//	MotorTxData[7] = (motor_pid[3].output&0xFF); 

}
/**********************************************************************************************************
*�� �� ��: chassis_current_mix
*����˵��: ���̵�������ں�
*��    ��: ��Ҫ�ٶȻ�������λ�û����������ʻ�����
*�� �� ֵ: �������
**********************************************************************************************************/
void chassis_current_mix()
{
	MotorTxData[0] = (((motor_pid[0].output+chassis_yaw_angle.output)>>8)&0xFF);
	MotorTxData[1] = ((motor_pid[0].output+chassis_yaw_angle.output)&0xFF); 
	MotorTxData[2] = (((motor_pid[1].output+chassis_yaw_angle.output)>>8)&0xFF);
	MotorTxData[3] = ((motor_pid[1].output+chassis_yaw_angle.output)&0xFF); 
	MotorTxData[4] = (((motor_pid[2].output+chassis_yaw_angle.output)>>8)&0xFF);
	MotorTxData[5] = ((motor_pid[2].output+chassis_yaw_angle.output)&0xFF); 
	MotorTxData[6] = (((motor_pid[3].output+chassis_yaw_angle.output)>>8)&0xFF);
	MotorTxData[7] = ((motor_pid[3].output+chassis_yaw_angle.output)&0xFF); 
}

/**********************************************************************************************************
*�� �� ��: set_current_zero
*����˵��: ������ֵ����Ϊ0
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
void set_current_zero()
{
	char i;
	for(i=0;i<8;i++)
	{
		MotorTxData[i]=0;
		HeadTxData[i]=0;
	}
	CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
	//CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);  //����̵�����͸����ĵ���ֵ
}
/**********************************************************************************************************
*�� �� ��: chassis_twist_control
*����˵��: ��������̨���ϵ�Ť������ �����è����
*��    ��: ��Ҫyaw��Ƕȣ����ٶȣ�����ٶȷ���
*�� �� ֵ: �������
**********************************************************************************************************/


void chassis_twist_control()
{
	if(remote_control.switch_left!=3)
	{
		Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_G_Pin,1);//����ledG��˸Ƶ��
		DBUS_Deal();//��ȡң���������ݲ������ݸ�ֵ�������Ŀ��ת��
		if(robot_status.mode!=TWIST)
		{
			chassis_yaw_angle.initial=moto_chassis[5].angle;//����è��ʱ����¼���̳�ʼֵΪ��̨yaw�����ڻ�е�Ƕ�ֵ
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			
			pan_tilt_pitch.initial=moto_chassis[4].angle;
			robot_status.mode=TWIST;//������ģʽ����Ϊè��ģʽ
		}
			
			
			chassis_speed_control();//�����ٶȻ�����
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw�����������õĽǶ���Ϊ����ֵ
			pan_tilt_lock_control(); //��̨��ͷ��������
			chassis_current_mix();
			
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
			
		}
	else if(remote_control.switch_left==3)//ʹ������������̨֮��ĽǶȱ�ɳ�ʼ��״̬
	{
		robot_status.mode=STANDBY;//�����������״̬
//		chassis_yaw_angle.target=chassis_yaw_angle.initial;
//		PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw�����������õĽǶ���Ϊ����ֵ
//		chassis_speed_control();//�����ٶȻ�����
//		//set_moto_current(&hcan1,chassis_yaw_angle.output,chassis_yaw_angle.output,chassis_yaw_angle.output,chassis_yaw_angle.output);
//		chassis_current_mix();
//		CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
		set_current_zero();
	}
	
}
/**********************************************************************************************************
*�� �� ��: chassis_follow_pan_tilt_control
*����˵��: ���̸�����̨�������ֱ�ߣ��ӽ�Ϊ��̨����ӽǣ���������ʵ����ֱ�߲���
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
bool follow_flag=0;
void chassis_follow_pan_tilt_control()
{
	
	if(robot_status.mode!=FOLLOW)//�����ӷǸ���ģʽ�������ģʽʱ����Ҫ��¼��������̨�ľ��ԽǶ�
		{
			chassis_yaw_angle.initial=moto_chassis[5].angle;//�������̸���ʱ����¼���̳�ʼֵΪ��̨yaw�����ڻ�е�Ƕ�ֵ
			robot_status.mode=FOLLOW;//�������ڵ��̸�˭ģʽ
			//chassis_yaw.target=chassis_yaw.initial;
		}
		chassis_speed_control();//�����ٶȻ�����
		chassis_yaw_angle.target=chassis_yaw_angle.initial;
		PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw�����������õĽǶ���Ϊ����ֵ
		pan_tilt_lock_control(); //��̨��ͷ��������
		
		set_pan_tilt_current(&hcan1,pan_tilt_pitch_speed.output,pan_tilt_yaw_speed.output);
		set_moto_current(&hcan1,motor_pid[0].output+chassis_yaw_angle.output,
														motor_pid[1].output+chassis_yaw_angle.output,
														motor_pid[2].output+chassis_yaw_angle.output,
														0);
		
}




/******************************************************
                      ���̵������2017(��һ��)

1, �����������̬�����
2��������̬�ĵ���
3��׼��CAN���������
4����ʼ�������
__________________________________________________________________
| ���λ�� |�����| ���\���� | ǰ | �� | �� | �� | ˳ | �� | ͣ | 													ǰ			��
``````````````````````````````````````````````````````````````````
|   ��ǰ   |0X201 |moto_ctr[0]| >0 | <0 | <0 | >0 | >0 | <0 | =0 |      									>0	|		<0
``````````````````````````````````````````````````````````````````	
|   ��ǰ   |0X202 |moto_ctr[1]| <0 | >0 | <0 | >0 | >0 | <0 | =0 |												<0	|		>0
``````````````````````````````````````````````````````````````````   //2018.10.6 �޸� 												2018.10.7�������ԣ�ԭ��û���⣩
|   ���   |0X203 |moto_ctr[2]| <0 | >0 | >0 | <0 | >0 | <0 | =0 |												>0  |		<0
``````````````````````````````````````````````````````````````````	
|   �Һ�   |0X204 |moto_ctr[3]| >0 | <0 | >0 | <0 | >0 | <0 | =0 |												<0	|		>0
``````````````````````````````````````````````````````````````````
|  ��ע:|moto_ctr[0]|=|moto_ctr[1]|=|moto_ctr[2]|=|moto_ctr[3]|  |
``````````````````````````````````````````````````````````````````											
*************************** ***************************/
//static float record=0.0;
//void walk_straight(void){
//	if(moto_ctr[0]>0&&moto_ctr[1]>0&&moto_ctr[2]>0&&moto_ctr[3]>0){
//			record=imu.yaw;
//	}else if(moto_ctr[0]<0&&moto_ctr[1]<0&&moto_ctr[2]<0&&moto_ctr[3]<0){
//	}
//}

