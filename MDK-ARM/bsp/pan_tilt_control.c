#include "pan_tilt_control.h"
float steer_output;
void steer_control()
{
	
		steer_output=15+(remote_control.ch4*10/660);
		PWM_SetDuty(&htim5,TIM_CHANNEL_1,steer_output/100);
}

/**********************************************************************************************************
*�� �� ��: pan_tilt_control
*����˵��: ��̨���Ƴ���
*��    ��: ��Ҫyaw��Ƕȣ����ٶȣ�����ٶȷ���
*�� �� ֵ: �������
**********************************************************************************************************/
int cnt_cat=0;
void pan_tilt_control()
{
	if(remote_control.switch_left==2)
	{
//		
//		/********************��������ڵ��̿��ƣ������ź�Ϊ����Ļ�е�Ƕ�***********************/
//		motor_pid[4].target=remote_control.ch2*1000/660+5440;//����     pitch��    ������̨Ϊyaw
//		motor_pid[4].f_cal_pid(&motor_pid[4],moto_chassis[4].angle);//��е�ǶȻ�
//		pan_tilt_yaw_speed.target=motor_pid[4].output;
//		motor_pid[4].f_cal_pid(&pan_tilt_yaw_speed,imu.gz);
//		
//		
//		
//		motor_pid[5].target=remote_control.ch4+4900;//����     yaw��       ������̨Ϊpitch
//		motor_pid[5].f_cal_pid(&motor_pid[5],moto_chassis[5].angle);//��е�ǶȻ�
//		pan_tilt_pithch_speed.target=motor_pid[5].output;
//		motor_pid[5].f_cal_pid(&pan_tilt_pithch_speed,imu.gy);//���ٶȻ�
//		set_pan_tilt_current(&hcan1,pan_tilt_yaw_speed.output,pan_tilt_pithch_speed.output);
		
	}
	if(remote_control.switch_left==1)
	{	
		cnt_cat++;
		/********************YAW�����***********************/
		if(remote_control.ch2==0)//ƫ����������λ
		{
			if(pan_tilt_yaw.target==0)  //����ʱ���Ƕ�����ֵ
			{
				pan_tilt_yaw.target=imu.yaw;
				
			}
			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
			//motor_pid[5].f_cal_pid(&motor_pid[5],imu.yaw);//imu�ǶȻ�
			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
	  }
		else
		{
			pan_tilt_yaw.target=0;////ƫ����������0,�����нǶȿ���
			pan_tilt_yaw_speed.target=remote_control.ch2*300/660;//yaw_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
		}
		
		pan_tilt_yaw.f_cal_pid(&pan_tilt_yaw_speed,imu.gz);
		
		/*************************PITCH�����**************************/		
		
		
		if(remote_control.ch3==0)//ƫ����������λ
		{
			if(motor_pid[4].target==0)  //����ʱ���Ƕ�����ֵ
			{
				motor_pid[4].target=moto_chassis[4].angle;
				
			}
			PID_Control_Yaw(&motor_pid[4],moto_chassis[4].angle);
			pan_tilt_pitch_speed.target=motor_pid[4].output;
	  }
		else
		{
			motor_pid[4].target=0;////ƫ����������0,�����нǶȿ���
			pan_tilt_pitch_speed.target=remote_control.ch3*300/660;//pitch_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
		}
		
		
		
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gx);//���ٶȻ�
		set_pan_tilt_current(&hcan1,pan_tilt_pitch_speed.output,pan_tilt_yaw_speed.output);
		
		if(cnt_cat<200)
			set_moto_current(&hcan1,1000,1000,1000,1000);
		if(cnt_cat>200&&cnt_cat<400)
		{
			set_moto_current(&hcan1,-1000,-1000,-1000,-1000);
			
		}
		else if(cnt_cat>400)
			cnt_cat=0;
	}		
	
	else
	{
		set_pan_tilt_current(&hcan1,0,0);
		set_moto_current(&hcan1,0,0,0,0);
	}
}

/**********************************************************************************************************
*�� �� ��: pan_tilt_machine_home
*����˵��: ��̨yaw���pitch���е���У����ڳ����ʼ��
*��    ��: ��Ҫyaw��imu�ĽǶȣ����ٶ�(imu.yaw,imu.gz)
*�� �� ֵ: �������
**********************************************************************************************************/
void pan_tilt_machine_home(void)
{
	if(remote_control.switch_left!=3)
	{
		pan_tilt_yaw_mechanical_angle_control();
		pan_tilt_pitch_mechanical_angle_control();
		if(remote_control.switch_right!=3)
		set_pan_tilt_current(&hcan1,pan_tilt_pitch_speed.output,pan_tilt_yaw_speed.output);
		else
			set_pan_tilt_current(&hcan1,0,0);
		//set_pan_tilt_current(&hcan1,0,pan_tilt_yaw_speed.output);
	}
	else
		set_pan_tilt_current(&hcan1,0,0);
}

/**********************************************************************************************************
*�� �� ��: pan_tilt_lock_control
*����˵��: ��̨yaw�����λ������,�����ͷ
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
void pan_tilt_lock_control()
{
	pan_tilt_yaw_imu_angle_control();
	pan_tilt_pitch_imu_angle_control();
	
	HeadTxData[0]=(uint8_t)((pan_tilt_pitch_speed.output>>8)&0xFF);
  HeadTxData[1]=(uint8_t)(pan_tilt_pitch_speed.output&0xFF);           //205   pitch
	  
	HeadTxData[2]=(uint8_t)((pan_tilt_yaw_speed.output>>8)&0xFF);
  HeadTxData[3]=(uint8_t)(pan_tilt_yaw_speed.output&0xFF);           //206  yaw 
	if(remote_control.switch_left!=3)
	{
		CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
	}
	else if(remote_control.switch_left==3)
	{
		HeadTxData[0]=0;
		HeadTxData[1]=0;           //205   pitch
		HeadTxData[2]=0;
		HeadTxData[3]=0;           //206  yaw 
		CAN_Send_Msg(&hcan1,0,HEADID,8);
	}
	
	//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
}


/**********************************************************************************************************
*�� �� ��: pan_tilt_yaw_imu_angle_control
*����˵��: ��̨yaw����ƣ���Ҫimu��yaw��Ƕ���Ϊ����ֵ
*��    ��: ��Ҫyaw��imu�ĽǶȣ����ٶ�(imu.yaw,imu.gz)
*�� �� ֵ: �������
**********************************************************************************************************/
void pan_tilt_yaw_imu_angle_control(void)
{	
	if(remote_control.switch_left==2)
	{
	if(remote_control.ch3==0)//ƫ����������λ
		{
			if(pan_tilt_yaw.target==0)  //����ʱ���Ƕ�����ֵ
			{
				pan_tilt_yaw.target=imu.yaw;
				
			}
			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
	  }
	
		else
		{
			pan_tilt_yaw.target=0;////ƫ����������0,�����нǶȿ���
			pan_tilt_yaw_speed.target=remote_control.ch3*120/660;//yaw_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
		}
	}
	else if(remote_control.switch_left==1)
	{
	if(remote_control.ch4==0)//ƫ����������λ
		{
			if(pan_tilt_yaw.target==0)  //����ʱ���Ƕ�����ֵ
			{
				pan_tilt_yaw.target=imu.yaw;
				
			}
			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
	  }
	
		else
		{
			pan_tilt_yaw.target=0;////ƫ����������0,�����нǶȿ���
			pan_tilt_yaw_speed.target=remote_control.ch4*120/660;//yaw_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
		}
	}
	
		
		pan_tilt_yaw.f_cal_pid(&pan_tilt_yaw_speed,-imu.gz);
		
}
/**********************************************************************************************************
*�� �� ��: pan_tilt_yaw_mechanical_angle_control
*����˵��: ��̨yaw����ƣ���Ҫ����������ĽǶ���Ϊ����ֵ
*��    ��: ��Ҫyaw�����������ĽǶȣ�imu�Ľ��ٶȣ�moto_chassis[4].angle,imu.gz��
*�� �� ֵ: �������
**********************************************************************************************************/
void pan_tilt_yaw_mechanical_angle_control(void)
{
	/*************************������**************************/
//	if(remote_control.ch2==0)//ƫ����������λ
//		{
//			if(motor_pid[5].target==0)  //����ʱ���Ƕ�����ֵ
//			{
//				motor_pid[5].target=moto_chassis[4].angle;
//				
//			}
//			PID_Control_Pitch(&motor_pid[5],moto_chassis[4].angle);
//			pan_tilt_yaw_speed.target=motor_pid[5].output;
//	  }
//		else
//		{
//			motor_pid[5].target=0;////ƫ����������0,�����нǶȿ���
//			pan_tilt_yaw_speed.target=remote_control.ch2*300/660;//yaw_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
//		}
	
	/*************************��е������***********************/
	if(motor_pid[5].target==0)  //����ʱ���Ƕ�����ֵ
	{
		motor_pid[5].target=moto_chassis[5].angle;
	}
		PID_Control_Yaw(&motor_pid[5],moto_chassis[5].angle);
		pan_tilt_yaw_speed.target=-motor_pid[5].output;
	/**********************************************************/
		motor_pid[5].f_cal_pid(&pan_tilt_yaw_speed,-imu.gz);
}



/**********************************************************************************************************
*�� �� ��: pan_tilt_pitch_imu_angle_control
*����˵��: ��̨pitch����ƣ���Ҫimu��pitch��Ƕ���Ϊ����ֵ
*��    ��: ��Ҫyaw��imu�ĽǶȣ����ٶ�(imu.pitch,imu.gx)
*�� �� ֵ: �������
**********************************************************************************************************/
void pan_tilt_pitch_imu_angle_control(void)
{
//	if(remote_control.ch3==0)//ƫ����������λ
//		{
//			if(pan_tilt_pitch.target==0)  //����ʱ���Ƕ�����ֵ
//			{
//				pan_tilt_pitch.target=imu.pit;
//			}
//			PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
//			pan_tilt_pitch_speed.target=pan_tilt_pitch.output;
//	  }
//		else
//		{
//			pan_tilt_pitch.target=0;////ƫ����������0,�����нǶȿ���
//			pan_tilt_pitch_speed.target=remote_control.ch3*300/660;//pitch_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
//		}
		pan_tilt_pitch.target=185;//pan_tilt_pitch.initial;
		PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
		pan_tilt_pitch_speed.target=-pan_tilt_pitch.output;
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gy);//���ٶȻ�
		pan_tilt_pitch_speed.output=GildeAverageValueFilter(pan_tilt_pitch_speed.output,Data);
}
/**********************************************************************************************************
*�� �� ��: pan_tilt_pitch_mechanical_angle_control
*����˵��: ��̨pitch����ƣ���Ҫ����������ĽǶ���Ϊ����ֵ
*��    ��: ��Ҫyaw��imu�ĽǶȣ����ٶ�(moto_chassis[4].angle,imu.gx)
*�� �� ֵ: �������
**********************************************************************************************************/
void pan_tilt_pitch_mechanical_angle_control(void)
{
	/*************************������**************************/
//	if(remote_control.ch3==0)//ƫ����������λ
//		{
//			if(motor_pid[4].target==0)  //����ʱ���Ƕ�����ֵ
//			{
//				motor_pid[4].target=moto_chassis[4].angle;
//			}
//			PID_Control_Pitch(&motor_pid[4],moto_chassis[4].angle);
//			pan_tilt_pitch_speed.target=motor_pid[4].output;
//	  }
//		else
//		{
//			motor_pid[4].target=0;////ƫ����������0,�����нǶȿ���
//			pan_tilt_pitch_speed.target=remote_control.ch3*300/660;//pitch_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
//		}
	/*************************��е������***********************/
	  motor_pid[4].target=3310;
		PID_Control_Pitch(&motor_pid[4],moto_chassis[4].angle);
		pan_tilt_pitch_speed.target=-motor_pid[4].output;
		
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gy);//���ٶȻ�
		
}

