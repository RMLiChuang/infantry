#include "pan_tilt_control.h"
#define PAN_TILT_YAW_NUM 0.07f
#define PAN_TILT_PIT_NUM 0.07f
//���̿�������
#define PAN_TILT_CONTROL_TIME	0.005f

/**********************************************************************************************************
*�� �� ��: pan_tilt_init
*����˵��: ��̨��ʼ��
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
void pan_tilt_init()
{
		pitch_angle_target=179.0f;																																																																																
    const static fp32 pan_tilt_yaw_order_filter[1] = {PAN_TILT_YAW_NUM};
    const static fp32 pan_tilt_pit_order_filter[1] = {PAN_TILT_PIT_NUM};
		init_Fric_PWM();			//��ʼ��Ħ���ֵ�PWM
    //��һ���˲�����б����������
    first_order_filter_init(&pan_tilt_cmd_slow_set_yaw, PAN_TILT_CONTROL_TIME, pan_tilt_yaw_order_filter);
    first_order_filter_init(&pan_tilt_cmd_slow_set_pit, PAN_TILT_CONTROL_TIME, pan_tilt_pit_order_filter);
		//Ħ����б�º�����ʼ��
		ramp_init(&fric_ramp,SHOOT_CONTROL_TIME * 0.001f, Close_Fric_ON, Fric_OFF);
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
//		if(remote_control.switch_right!=3)
//		set_pan_tilt_current(&hcan1,pan_tilt_pitch_speed.output,pan_tilt_yaw_speed.output);
//		else
//			set_pan_tilt_current(&hcan1,0,0);
		//set_pan_tilt_current(&hcan1,0,pan_tilt_yaw_speed.output);
//	}
//	else
//		set_pan_tilt_current(&hcan1,0,0);
		
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
}
}

/**********************************************************************************************************
*�� �� ��: pan_tilt_lock_control
*����˵��: ��̨yaw�����λ������,�����ͷ
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
void pan_tilt_lock_control()
{
	if(robot_status.vision_status==VISION_SUCCESS&&
		robot_status.vision_mode==ACTIVATE)
		//int_abs(VISION_YAW_TARGET-Armour_attack.pan_tilt_angel_err.Yaw_Err)<60&&
		//int_abs(VISION_PIT_TARGET-Armour_attack.pan_tilt_angel_err.Pitch_Err)<60)//����Ӿ�û�����߲�����߿���Ϊ1�������Ӿ����
	{
		armour_attack();
		//pan_tilt_pitch_imu_angle_control();
	}
	else 
	{
		pan_tilt_yaw_imu_angle_control();
		pan_tilt_pitch_imu_angle_control();
	}
	HeadTxData[0]=(uint8_t)((pan_tilt_pitch_speed.output>>8)&0xFF);
  HeadTxData[1]=(uint8_t)(pan_tilt_pitch_speed.output&0xFF);           //205   pitch
	  
	HeadTxData[2]=(uint8_t)((pan_tilt_yaw_speed.output>>8)&0xFF);
  HeadTxData[3]=(uint8_t)(pan_tilt_yaw_speed.output&0xFF);           //206  yaw 
	HeadTxData[4]=(uint8_t)((shoot_control_loop()>>8)&0xFF); //Ħ���ֿ���  
	HeadTxData[5]=(uint8_t)(shoot_control_loop()&0xFF);
	if(robot_status.control_mode==REMOTE_CONTROL)
	{
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
			if(remote_control.switch_right!=3)//���ڼ���ģʽʱ���ر�Ħ����
			{
				HeadTxData[4]=0;
				HeadTxData[5]=0; 
			}
			CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
		}
	}
	else if(robot_status.control_mode==KEYBOARD_CONTROL)
	{
		CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
	}
	
	//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
}


/**********************************************************************************************************
*�� �� ��: pan_tilt_yaw_imu_angle_control
*����˵��: ��̨yaw����ƣ���Ҫimu��yaw��Ƕ���Ϊ����ֵ
*��    ��: ��Ҫyaw��imu�ĽǶȣ����ٶ�(imu.yaw,imu.gz)
*�� �� ֵ: �������
**********************************************************************************************************/
int yaw_velocity_target;//yaw����ٶ�����Ŀ��ֵ
void pan_tilt_yaw_imu_angle_control(void)
{	
//	if(remote_control.switch_left==2)
//	{
	if(yaw_velocity_target==0)//ƫ����������λ
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
			pan_tilt_yaw_speed.target=yaw_velocity_target*chassis_pan_tilt_max_rotate_speed/660;//yaw_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
		}
	//}
		
//	else if(remote_control.switch_left==1)
//	{
//	if(remote_control.ch4==0)//ƫ����������λ
//		{
//			if(pan_tilt_yaw.target==0)  //����ʱ���Ƕ�����ֵ
//			{
//				pan_tilt_yaw.target=imu.yaw;
//				
//			}
//			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
//			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
//	  }
//	
//		else
//		{
//			pan_tilt_yaw.target=0;////ƫ����������0,�����нǶȿ���
//			pan_tilt_yaw_speed.target=remote_control.ch4*120/660;//yaw_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
//		}
//	}
	
		
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
*�� �� ��: PID_Pitch_control
*����˵��: ��̨pitch������ʽ�ǶȻ����ƣ�
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
	float error[3]={0},Dbuf[3]={0};
void PID_Pitch_control()
{	
	error[2]=error[1];
	error[1]=error[0];
	error[0]=pan_tilt_pitch.target-imu.pit;
	if((int_abs(pan_tilt_pitch.err) > pan_tilt_pitch.DeadBand))
	{
		pan_tilt_pitch.pout = pan_tilt_pitch.kp *(error[0]-error[1]);
		pan_tilt_pitch.iout = pan_tilt_pitch.ki * error[0];
//		if(pan_tilt_pitch.iout > pan_tilt_pitch.IntegralLimit)
//			pan_tilt_pitch.iout = pan_tilt_pitch.IntegralLimit;
//		if(pan_tilt_pitch.iout < - pan_tilt_pitch.IntegralLimit)
//			pan_tilt_pitch.iout = - pan_tilt_pitch.IntegralLimit;
		Dbuf[2] = Dbuf[1];
    Dbuf[1] = Dbuf[0];
    Dbuf[0] = (error[0] - 2.0f * error[1] + error[2]);
    pan_tilt_pitch.dout = pan_tilt_pitch.kd * Dbuf[0];
		pan_tilt_pitch.output += pan_tilt_pitch.pout + pan_tilt_pitch.iout + pan_tilt_pitch.dout;
		if(pan_tilt_pitch.output>pan_tilt_pitch.MaxOutput)         
		{
			pan_tilt_pitch.output = pan_tilt_pitch.MaxOutput;
		}
		if(pan_tilt_pitch.output < -(pan_tilt_pitch.MaxOutput))
		{
			pan_tilt_pitch.output = -(pan_tilt_pitch.MaxOutput);
		}
	}
}

/**********************************************************************************************************
*�� �� ��: pan_tilt_pitch_imu_angle_control
*����˵��: ��̨pitch����ƣ���Ҫimu��pitch��Ƕ���Ϊ����ֵ
*��    ��: ��Ҫyaw��imu�ĽǶȣ����ٶ�(imu.pitch,imu.gx)
*�� �� ֵ: �������
**********************************************************************************************************/
int pitch_velocity_target;//pitch����ٶ�����Ŀ��ֵ
float pitch_angle_target=179.0f;//pitch���������Ŀ��ֵ
void pan_tilt_pitch_imu_angle_control(void)
{
	if(robot_status.control_mode==REMOTE_CONTROL)
	{
		if(pitch_velocity_target==0)//������������λ
		{
			if(pan_tilt_pitch.target==0)  //����ʱ���Ƕ�����ֵ
			{
				pan_tilt_pitch.target=imu.pit;
			}
		PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
			pan_tilt_pitch_speed.target=pan_tilt_pitch.output;
	  }
		else
		{
			pan_tilt_pitch.target=0;////ƫ����������0,�����нǶȿ���
			pan_tilt_pitch_speed.target=pitch_velocity_target*300/660;//pitch_control;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
		}
		#ifdef INFANTRY_1
		PID_Control_Pitch(&pan_tilt_pitch_speed,-imu.gy);//���ٶȻ�
		#endif
		#ifdef INFANTRY_2
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gy);//���ٶȻ�
		#endif
		//pan_tilt_pitch_speed.output=GildeAverageValueFilter(pan_tilt_pitch_speed.output,Data);//�˲���
	}
	else if(robot_status.control_mode==KEYBOARD_CONTROL)
	{
		pan_tilt_pitch.target=pitch_angle_target;//��Դ��������
		
		//PID_Pitch_control();
		#ifdef INFANTRY_1
		pan_tilt_pitch.target=fp32_constrain(pan_tilt_pitch.target,150.0f,190.0f);//�޷�
		PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
		#endif
		#ifdef INFANTRY_2
		pan_tilt_pitch.target=fp32_constrain(pan_tilt_pitch.target,165.0f,197.0f);//�޷�
		PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
		#endif
		pan_tilt_pitch_speed.target=pan_tilt_pitch.output;
		#ifdef INFANTRY_1
		PID_Control_Pitch(&pan_tilt_pitch_speed,-imu.gy);//���ٶȻ�
		#endif
		#ifdef INFANTRY_2
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gy);//���ٶȻ�
		#endif
	}
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

