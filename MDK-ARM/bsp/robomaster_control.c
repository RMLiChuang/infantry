/**
  *@file robomaster_control.c
  *@date 2019-1-6
  *@author izh20
  *@brief 
  */


#include "robomaster_control.h"
#include "robomaster_common.h"


#define twist_speed        1000
#define chassis_limit      1000       //��è��ʱ�ĵ�����λ��е�Ƕ�
#define chassis_dead_band  100        //���̻�е�Ƕȵ�����
#define twist_dead_band    150				//����è����ʶ��ң��������ʱ��������̨λ�õ����

//Ħ���ֳ�ʼ��
void RM2312_Init(void)
{
	uint16_t pulse=1050;					
	TIM_SetTIM5Compare(pulse);
	delay_ms(400);
	char  i;
  for(i=0;i<10;i++)
	{
		pulse+=80;
		TIM_SetTIM5Compare(pulse);
		delay_ms(300);		
	} 
}
	void TIM_SetTIM5Compare(uint16_t compare)
{
	TIM5->CCR1=compare;
	TIM5->CCR2=compare;
	TIM5->CCR3=compare;
	TIM5->CCR4=compare;
	
}

void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,int duty)
	{
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = duty - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = duty - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = duty - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = duty - 1;break;
	}
	
}
	
char BD_zhuangtai=0;
char BD_tuidan=0;
int i,keep;
int pwm_output;
void shoot_control()
{
	if(remote_control.switch_left!=3)
		{
				
				
//					motor_pid[6].target=3000;
//					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
//					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//�����������ֵ
//					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
				single_shot();
					
					if(motor_pid[6].err<500&&moto_chassis[6].speed_rpm>600)		//�����ֹ���״̬�ȶ�
					{
						BD_zhuangtai=1;
					}
					if((BD_zhuangtai==1)&&(motor_pid[6].err>400)&&(moto_chassis[6].speed_rpm<600))//�������ɹ����ȶ�������
					{
						BD_zhuangtai=2;
						BD_tuidan=1;
					}
					if(BD_tuidan==1)
					{
						BD_tuidan=2;
						moto_chassis[6].round_cnt=0;
					}
					if(BD_tuidan==2)
					{
						if(moto_chassis[6].round_cnt>-1)
						{
							motor_pid[6].target=-2500;
							motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
							HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//�����������ֵ
					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
						}
						else
						{
							BD_tuidan=0;
							BD_zhuangtai=3;
						}
					}
					if(BD_zhuangtai==3)
					{
						keep++;
						if(keep>500)
							{BD_zhuangtai=0;keep=0;}
					}
					
					
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,1800);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,1800);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,1800);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,1800);
				}
				if(remote_control.switch_right==1)
				{
					HeadTxData[4]=0;
					HeadTxData[5]=0;//�����ֵ���ֵ
					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
					//set_rammer_current(&hcan1,0);
					init_TIM5_PWM();
				}

		if(remote_control.switch_left==3)
		{
			HeadTxData[4]=0;
			HeadTxData[5]=0;//�����ֵ���ֵ
			//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
			//set_rammer_current(&hcan1,0);
			init_TIM5_PWM();
		}
}

/**********************************************************************************************************
*�� �� ��: single_shot
*����˵��: �����ֵ���
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
void single_shot()
{
	if(remote_control.switch_right==2)
	{
		moto_chassis[6].round_cnt=0;
		HeadTxData[4]=0;
		HeadTxData[5]=0;
	}
	if(remote_control.switch_right==3)
	{
		if(moto_chassis[6].round_cnt<4)
			{
					motor_pid[6].target=2500;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//�����������ֵ
					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
			}
		else
		{
			HeadTxData[4]=0;
			HeadTxData[5]=0;
		}
			PWM_SetDuty(&htim5,TIM_CHANNEL_1,1000);
			PWM_SetDuty(&htim5,TIM_CHANNEL_2,1000);
			PWM_SetDuty(&htim5,TIM_CHANNEL_3,1000);
			PWM_SetDuty(&htim5,TIM_CHANNEL_4,1000);
	}
}


//IMU_Type chassis_imu;
///**********************************************************************************************************
//*�� �� ��: chassis_control
//*����˵��: ���̿��Ƴ��򣬼�����imu��z��(yaw��)�ĽǶȻ��ͽ��ٶȻ����ƣ����̼����ٶȻ����п���
//*��    ��: ��Ҫyaw��Ƕȣ����ٶȣ�����ٶȷ���
//*�� �� ֵ: �������
//**********************************************************************************************************/
//extern int16_t moto_ctr[6];
//int32_t set_spd = 0;//�ٶȲ���
//int32_t turn=0;     //ת��
//long yaw_flag=0;
//extern int cnt1;
//extern int cnt2;
//char cnt_steer=0;
//int yaw_cnt=0;
//int chassis_yaw_correct=0;
//void chassis_control()
//{
//	if(remote_control.switch_right!=3)
//	{
////			if(cnt1==100)//0.5s����һ�Σ�ʹ��4��led��2HZƵ����˸���жϵ��̳�����������
////			{
////					HAL_GPIO_TogglePin(LED_USER_GPIO_PORT,LED_G_Pin);
////					cnt1=0;
////			}
//			Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_G_Pin,1);//����ledG��˸Ƶ��
//			DBUS_Deal();//��ȡң���������ݲ������ݸ�ֵ�������Ŀ��ת��
//			
//			cnt_steer++;
//			if(cnt_steer==4)
//			{
//				steer_control();
//				cnt_steer=0;
//			}
//			
//			
//			if(remote_control.switch_right==2)	
//			{
//          /*************************yaw�����    begin**********/		
//	
//					if(remote_control.ch3==0)//ƫ����������λ
//					{
////						if(yaw_cnt<500)//�����ϵ��һ��ʱ������ƫ���ǣ������ơ��������ں���Ҫһ��ʱ�䣬����ȡ500
////						yaw_cnt++;
//							if(chassis_yaw.target==0)  //����ʱ���Ƕ�����ֵ
//							{
//								chassis_yaw.target=imu.yaw;
//							}
//							PID_Control_Yaw(&chassis_yaw,imu.yaw);//�ú������0�ȵ�360�ȵ�ͻ��
//								//chassis_yaw.f_cal_pid(&chassis_yaw,imu.yaw);    //ƫ���Ƕȿ���
//								chassis_yaw_speed.target=chassis_yaw.output;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
//					}
//			
//					else//����ƫ������˺�ֻ�����ڻ����ٶȿ���
//					{
//							chassis_yaw.target=0;//ƫ����������0,�����нǶȿ���
//							chassis_yaw_speed.target=remote_control.ch3*300/660;//yaw_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
//					}
//		
//					/*************************yaw�����  end**********/		
//			
//					motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //�����趨ֵ����PID���㡣
//					motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //�����趨ֵ����PID���㡣        �ٶ�Ϊ����ֵ
//					motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //�����趨ֵ����PID���㡣
//					motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //�����趨ֵ����PID���㡣
//	
//	

//					chassis_yaw.f_cal_pid(&chassis_yaw_speed,-imu.gz);	
//	
////					set_moto_current(&hcan1,motor_pid[0].output+chassis_yaw_speed.output,   //��PID�ļ�����ͨ��CAN���͵����
////																	motor_pid[1].output+chassis_yaw_speed.output,
////																	motor_pid[2].output+chassis_yaw_speed.output,
////																	motor_pid[3].output+chassis_yaw_speed.output);
//			}
//			
//			
//		if(remote_control.switch_right==1)  //ң����������ֱֵ����Ϊ�ٶ�Ŀ��
//		{
//			
//			
//			
//			

//						motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //�����趨ֵ����PID���㡣
//						motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //�����趨ֵ����PID���㡣        �ٶ�Ϊ����ֵ
//						motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //�����趨ֵ����PID���㡣
//						motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //�����趨ֵ����PID���㡣
//						set_moto_current(&hcan1,motor_pid[0].output,   //��PID�ļ�����ͨ��CAN���͵����
//															motor_pid[1].output,
//															motor_pid[2].output,
//															motor_pid[3].output);
//			//set_moto_current(&hcan1,500,500,0,500);
//		}
//	
//	}
//		else
//		{
//			//chassis_yaw.target=imu.yaw;  //�ϵ��궨���̵ĳ�ʼyawֵ��
//		  set_moto_current(&hcan1,0,0,0,0);
//	  }
//}
/**********************************************************************************************************
*�� �� ��: GildeAverageValueFilter
*����˵��: �����ٶȻ�����
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
float Data[N];
float GildeAverageValueFilter(float NewValue,float *Data)
{
  float max,min;
  float sum;
  unsigned char i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=N-1;i!=0;i--)
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];
  }
  i=N-2;
  sum=sum-max-min;
  sum=sum/i;
  return(sum);
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
	CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);  //����̵�����͸����ĵ���ֵ
}
/**********************************************************************************************************
*�� �� ��: chassis_twist_control
*����˵��: ��������̨���ϵ�Ť������ �����è����
*��    ��: ��Ҫyaw��Ƕȣ����ٶȣ�����ٶȷ���
*�� �� ֵ: �������
**********************************************************************************************************/
bool chassis_position_flag=0;
void chassis_twist_control()  //�ڽ���è�������У���Ҫ����ң���������
{
		
		//DBUS_Deal();
		if(robot_status.mode!=TWIST)
		{
			//Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_B_Pin,1);//����ledB��˸Ƶ��
			pan_tilt_lock_control(); //��̨��ͷ��������
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw�����������õĽǶ���Ϊ����ֵ
			chassis_current_mix();
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
			if((moto_chassis[5].angle<(chassis_yaw_angle.initial+chassis_dead_band))&&(moto_chassis[5].angle>(chassis_yaw_angle.initial-chassis_dead_band)))
			robot_status.mode=TWIST;//������ģʽ����Ϊè��ģʽ
		}
		if(robot_status.mode==TWIST)
		{	
			//Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_B_Pin,1);//����ledB��˸Ƶ��
			chassis_speed_control();//�����ٶȻ�����
			if((remote_control.ch2+remote_control.ch1)==0||(moto_chassis[5].angle>(chassis_yaw_angle.initial+twist_dead_band))||(moto_chassis[5].angle<(chassis_yaw_angle.initial-twist_dead_band)))//��ң��û��ǰ�������˶����ߵ�������̨��һ��λ�ò����è��
			{
					if(chassis_position_flag==0)
					{
						chassis_yaw_angle.target=(chassis_yaw_angle.initial+chassis_limit);
						PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw�����������õĽǶ���Ϊ����ֵ
						if((moto_chassis[5].angle<(chassis_yaw_angle.target+chassis_dead_band))&&(moto_chassis[5].angle>(chassis_yaw_angle.target-chassis_dead_band)))
							chassis_position_flag=1;
				
				
					}
					if(chassis_position_flag==1)
					{
						chassis_yaw_angle.target=(chassis_yaw_angle.initial-chassis_limit);
						PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw�����������õĽǶ���Ϊ����ֵ
						if((moto_chassis[5].angle<(chassis_yaw_angle.target+chassis_dead_band))&&(moto_chassis[5].angle>(chassis_yaw_angle.target-chassis_dead_band)))
							chassis_position_flag=0;
					}
				}
			if((remote_control.ch2+remote_control.ch1)!=0&&(moto_chassis[5].angle<(chassis_yaw_angle.initial+twist_dead_band))&&(moto_chassis[5].angle>(chassis_yaw_angle.initial-twist_dead_band)))//����ң�в����ҵ��̻ص�����̨��ͬ��λʱ���е��̸�����̨�˶�
			{
				chassis_follow_pan_tilt_control();
			}
			pan_tilt_lock_control(); //��̨��ͷ��������
			chassis_current_mix();
			
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
		}
}



/**********************************************************************************************************
*�� �� ��: infantry_control
*����˵��: ��������̨���ϵĿ��Ƴ��� 
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/

bool calibrate_flag=0;
void infantry_control()
{
	if(calibrate_flag==0)
	{
		calibrate_initial_position();
		calibrate_flag=1;
	}
	if(remote_control.switch_left==2)//���̸�����̨ģʽ
	{
			chassis_follow_pan_tilt_control();		
	}
	else if(remote_control.switch_left==1)//è��ģʽ
	{
		chassis_twist_control();
	}
	else if(remote_control.switch_left==3)//ʹ������������̨֮��ĽǶȱ�ɳ�ʼ��״̬
	{
		robot_status.mode=STANDBY;//�����������״̬
		set_current_zero();
	}
	
}
/**********************************************************************************************************
*�� �� ��: chassis_follow_pan_tilt_control
*����˵��: ���̸�����̨�������ֱ�ߣ��ӽ�Ϊ��̨����ӽǣ���������ʵ����ֱ�߲���
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
//bool follow_flag=0;
void chassis_follow_pan_tilt_control()
{
	
		//Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_A_Pin,1);//����ledA��˸Ƶ��
		DBUS_Deal();//��ȡң���������ݲ������ݸ�ֵ�������Ŀ��ת��
		if(robot_status.mode!=FOLLOW)
		{
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			
			//pan_tilt_pitch.initial=moto_chassis[4].angle;
			robot_status.mode=FOLLOW;//������ģʽ����Ϊ���̸�����̨ģʽ
		}
		if(robot_status.mode==FOLLOW)
		{	
			chassis_speed_control();//�����ٶȻ�����
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw�����������õĽǶ���Ϊ����ֵ
			pan_tilt_lock_control(); //��̨��ͷ��������
			chassis_current_mix();
			
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
		}
}

/**********************************************************************************************************
*�� �� ��: calibrate_initial_position
*����˵��: �궨��ʼ������̨�����λ�ã����ڵ��̻���
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
void calibrate_initial_position()
{
	chassis_yaw_angle.initial=moto_chassis[5].angle;//��¼�ϵ�ʱ�̵��������̨�ĳ�ʼλ��
}
/**********************************************************************************************************
*�� �� ��: set_chassis_moto_target_zero
*����˵��:���õ��̵��Ŀ��ֵΪ0  ������ʧ��ʱ����ô˺���
*��    ��: 
*�� �� ֵ: 
**********************************************************************************************************/
void set_chassis_moto_target_zero()
{
	char i;
	for(i=0;i<8;i++)
	{
		MotorTxData[i]=0;
		//HeadTxData[i]=0;
		//CAN_Send_Msg(&hcan1,0,HEADID,8);
		
	}
	CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
	//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
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

