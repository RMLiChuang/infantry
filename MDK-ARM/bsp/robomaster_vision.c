/**
  ******************************************************************************
  * File Name          : robomaster_vision.c
	* author						 : �ܺ�
	* Date							 :2019.1.23
  * Description        :�Ӿ������ļ�
	
  ******************************************************************************
  ******************************************************************************
  */

#include "robomaster_vision.h"
Enemy_Struct EnemyDataBuffer[ENEMYDATABUFFERLENGHT];
uint8_t EnemyDataBufferPoint;//�����������ݻ���ָ��
#define MAX_ERR 600//�Ӿ�����޷�
#define MIN_ERR 0


#define YAW_AUTO_SHOT_THRESHOLD 3//�Ӿ��Զ������ֵ���� ��λΪ���ص�
#define PIT_AUTO_SHOT_THRESHOLD 15
#define VISION_DATE_SIZE 6

uint8_t UART6_Date[8]={0};//����6����pid��������
Vision_Attack Armour_attack;
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
////	printf("test");
//	 if(huart == &huart6)
//  {
////		printf("test");
//		__HAL_UART_CLEAR_PEFLAG(&huart6);
//		
//		
//		
//		HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);
//	}
//}
/**
  * @brief  �Ӿ���ʼ��
  * @param  void
  * @retval void
  */
void Vision_InitConfig(void)
{
    EnemyDataBufferPoint = 0;   
    EnemyDataBuffer[EnemyDataBufferPoint].ID = 1;
    EnemyDataBuffer[EnemyDataBufferPoint].Time = 0;
    EnemyDataBuffer[EnemyDataBufferPoint].X = 0;
    EnemyDataBuffer[EnemyDataBufferPoint].Y = 0;
    EnemyDataBuffer[EnemyDataBufferPoint].Z = 1;
}
/**********************************************************************************************************
*�� �� ��: armour_attack
*����˵��: �Ӿ�ʶ��װ�װ岢������   ����˼·��ֻ�е����ƫ������ң������ƫ����С��һ��ֵ��ʱ��(����λ����)�����ܼ����Ӿ�ʶ��
*��    ��: ��Ҫ�Ӿ��ṩ��̨�����װ�װ��yaw��pitch��err
*�� �� ֵ: �������
**********************************************************************************************************/
void armour_attack()
{
	/***********YAW��ƫ�����***************/
	vision_yaw.target=VISION_YAW_TARGET;
	vision_yaw.f_cal_pid(&vision_yaw,Armour_attack.pan_tilt_angel_err.Yaw_Err);
	pan_tilt_yaw_speed.target=-vision_yaw.output;
	pan_tilt_yaw_speed.f_cal_pid(&pan_tilt_yaw_speed,-imu.gz);
	/***********PITCH��ƫ�����***************/
	vision_pitch.target=VISION_PIT_TARGET;
	vision_pitch.f_cal_pid(&vision_pitch,Armour_attack.pan_tilt_angel_err.Pitch_Err);
	pan_tilt_pitch_speed.target=-vision_pitch.output;
	pan_tilt_pitch_speed.f_cal_pid(&pan_tilt_pitch_speed,-imu.gy);
	/**********ʹ�Ӿ����ֶ�ƽ�ȹ���*******/
	pan_tilt_yaw.target=imu.yaw;//
	pan_tilt_pitch.target=imu.pit;
/**********�Ӿ�ʶ����ɺ�����Զ����*******/
	if(int_abs(vision_yaw.err)<YAW_AUTO_SHOT_THRESHOLD&&int_abs(vision_pitch.err)<PIT_AUTO_SHOT_THRESHOLD)//��Ŀ���׼װ�װ�ʱ���Զ���������������
	{
		if (shoot_mode != SHOOT_DONE && trigger_motor.key == SWITCH_TRIGGER_ON)
     {
			 if(robot_status.fric_mode!=STOP)//��ֹδ��Ħ���ֵ�ʱ�򲦶�����
          shoot_mode = SHOOT_BULLET;
     }
	}
	else
	{
		 shoot_mode = SHOOT_READY;
	}
}
/**********************************************************************************************************
*�� �� ��: get_armour_err
*����˵��: ��ȡװ�װ��������̨�����
*��    ��:��Ҫ�Ӿ��ṩ��̨�����װ�װ��yaw��pitch��err
*�� �� ֵ: 
**********************************************************************************************************/
void  get_armour_err()
{
	
	//Latest_Vision_Control_Pack_Time = HAL_GetTick();
	Armour_attack.current_time=HAL_GetTick();
	Armour_attack.delta_time=Armour_attack.current_time-Armour_attack.last_time;
	Armour_attack.vision_frame=1000/Armour_attack.delta_time;
		if((UART6_Date[3]<3) && (UART6_Date[5]<3))
		{
			Armour_attack.pan_tilt_angel_err.origin_yaw=(UART6_Date[2] | UART6_Date[3]<<8);
			Armour_attack.pan_tilt_angel_err.origin_pitch=(UART6_Date[4] | UART6_Date[5]<<8);
			Armour_attack.check_sum=(UART6_Date[6] | UART6_Date[7]<<8) ;
		}
		if(Armour_attack.check_sum==(Armour_attack.pan_tilt_angel_err.origin_yaw+Armour_attack.pan_tilt_angel_err.origin_pitch))
		{
			Armour_attack.pan_tilt_angel_err.Yaw_Err=Armour_attack.pan_tilt_angel_err.origin_yaw;
			Armour_attack.pan_tilt_angel_err.Pitch_Err=Armour_attack.pan_tilt_angel_err.origin_pitch;
			//�޷�
			Armour_attack.pan_tilt_angel_err.Yaw_Err=int16_constrain(Armour_attack.pan_tilt_angel_err.Yaw_Err,MIN_ERR,MAX_ERR);
			Armour_attack.pan_tilt_angel_err.Pitch_Err=int16_constrain(Armour_attack.pan_tilt_angel_err.Pitch_Err,MIN_ERR,MAX_ERR);
			
			//�˲�
//			Armour_attack.pan_tilt_angel_err.Yaw_Err=Butterworth_Filter(Armour_attack.pan_tilt_angel_err.Yaw_Err,&Vision_BufferData[0],&Accel_Parameter);
//			Armour_attack.pan_tilt_angel_err.Pitch_Err=Butterworth_Filter(Armour_attack.pan_tilt_angel_err.Pitch_Err,&Vision_BufferData[1],&Accel_Parameter);
			
	
//			Armour_attack.pan_tilt_angel_err.Yaw_Err=GildeAverageValueFilter(Armour_attack.pan_tilt_angel_err.Yaw_Err,Data);
//			Armour_attack.pan_tilt_angel_err.Pitch_Err=GildeAverageValueFilter(Armour_attack.pan_tilt_angel_err.Pitch_Err,Data);
			//vision_time=HAL_GetTick();
			EnemyDataBuffer[EnemyDataBufferPoint].ID = EnemyDataBufferPoint;
			EnemyDataBuffer[EnemyDataBufferPoint].Time = HAL_GetTick();
			EnemyDataBuffer[EnemyDataBufferPoint].X = Armour_attack.pan_tilt_angel_err.Yaw_Err;
			EnemyDataBuffer[EnemyDataBufferPoint].Y = Armour_attack.pan_tilt_angel_err.Pitch_Err;
			EnemyDataBuffer[EnemyDataBufferPoint].Z = 0;
			EnemyDataBufferPoint++;
			if(EnemyDataBufferPoint>(ENEMYDATABUFFERLENGHT-1))
				EnemyDataBufferPoint=0;
			Armour_attack.last_time=HAL_GetTick();
			robot_status.vision_status=VISION_SUCCESS;//���Ӿ���Ϣ������
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);//����
		}

	}



//����6�����жϷ������
//֡ͷ0xfe

uint8_t counter=0;
uint8_t USART6_RX_BUF=0;
void USART6_IRQHandler(void)                	
{
	HAL_UART_IDLE_IRQHandler(&huart6);	
}                             


/**
  * @brief  ·����Ϻ��ĺ����������������һ�����
  * @param  Ԥ������ʱ�䳤��
  * @param  Ԥ��ʱ��
  * @param  Ԥ��λ��
  * @retval 0 �ɹ�        1 ʧ��
  */
uint8_t ForcastCore(uint16_t SampleTime, uint16_t ForcastTime, Point_Struct *ForcastPoint)
{
    int RelativeTime;                       //���ʱ�䣬��ֹ����ʱ�䳬��Χ
    uint16_t index = 0, Currentindex;       
    uint16_t SampleNum = 0;
    

    float A = 0;
    float B = 0;
    float C = 0;
    float D = 0;
    float E = 0;
    
    float Fx = 0;
    float Gx = 0;
    float Hx = 0;
    float Ix = 0;
    float Jx = 0;
    float Kx = 0;
    float Lx = 0;
    float Mx = 0;
    float Nx = 0;
    
    float Fz = 0;
    float Gz = 0;
    float Hz = 0;
    float Iz = 0;
    float Jz = 0;
    float Kz = 0;
    float Lz = 0;
    float Mz = 0;
    float Nz = 0;
    
    float Pax, Pbx, Pcx;
    float Paz, Pbz, Pcz;
    
    //Ѱ�����
    for(SampleNum = 0; SampleNum < ENEMYDATABUFFERLENGHT; SampleNum++)
    {
        if(EnemyDataBuffer[(EnemyDataBufferPoint + ENEMYDATABUFFERLENGHT - SampleNum - 1) % 
					ENEMYDATABUFFERLENGHT].Time + SampleTime < EnemyDataBuffer[EnemyDataBufferPoint].Time)//ȡ��ȥ300msʱ���ڵ����ݽ������
        {
            break;
        }
    }
    
    //�������������
    if(SampleNum < 5)
    {
        return 1;
    }
    
    E =  -(1 + SampleNum);
    
    //�������
    for(index = 0; index <= SampleNum; index++)
    {
        Currentindex = (EnemyDataBufferPoint + ENEMYDATABUFFERLENGHT - index) % ENEMYDATABUFFERLENGHT;
        
        RelativeTime = EnemyDataBuffer[Currentindex].Time - EnemyDataBuffer[(EnemyDataBufferPoint + ENEMYDATABUFFERLENGHT - SampleNum) % ENEMYDATABUFFERLENGHT].Time;//        
        //��ѭ�������� RelativeTime���ϼ���
        A = A - RelativeTime * RelativeTime * RelativeTime * RelativeTime;
        B = B - RelativeTime * RelativeTime * RelativeTime;
        C = C - RelativeTime * RelativeTime;
        D = D - RelativeTime;
        
        Fx = Fx + RelativeTime * RelativeTime * EnemyDataBuffer[Currentindex].X;
        Gx = Gx + RelativeTime * EnemyDataBuffer[Currentindex].X;
        Hx = Hx + EnemyDataBuffer[Currentindex].X;
        
        Fz = Fz + RelativeTime * RelativeTime * EnemyDataBuffer[Currentindex].Z;
        Gz = Gz + RelativeTime * EnemyDataBuffer[Currentindex].Z;
        Hz = Hz + EnemyDataBuffer[Currentindex].Z;
    }
    

    Ix = D * Fx - C * Gx;
    Jx = A * D - B * C;
    Kx = B * D - C * C;
    Lx = E * Fx - Hx * C;
    Mx = A * E - C * C;
    Nx = B * E - C * D;
    
    Iz = D * Fz - C * Gz;
    Jz = A * D - B * C;
    Kz = B * D - C * C;
    Lz = E * Fz - Hz * C;
    Mz = A * E - C * C;
    Nz = B * E - C * D;
    
    //���ݷǷ�
    if((!(Mx * Kx - Jx * Nx)) ||
        (!Kx) ||
        (!C) ||
        (!(Mz * Kz - Jz * Nz)) ||
        (!Kz))
    {
        return 1;
    }

    Pax = (Ix * Nx - Lx * Kx) / (Mx * Kx - Jx * Nx);
    Pbx = -(Ix + Pax * Jx)  / Kx;
    Pcx = - (Fx + Pax * A + Pbx * B) / C;
    
    Paz = (Iz * Nz - Lz * Kz) / (Mz * Kz - Jz * Nz);
    Pbz = -(Iz + Paz * Jz) / Kz;
    Pcz = - (Fz + Paz * A + Pbz * B) / C;
    
    ForcastTime += EnemyDataBuffer[EnemyDataBufferPoint].Time - EnemyDataBuffer[(EnemyDataBufferPoint + ENEMYDATABUFFERLENGHT - SampleNum) % ENEMYDATABUFFERLENGHT].Time;
    
    ForcastPoint->X = (ForcastTime * ForcastTime * Pax + Pbx * ForcastTime + Pcx);
    ForcastPoint->Y = EnemyDataBuffer[Currentindex].Y;
    ForcastPoint->Z = (Paz * ForcastTime * ForcastTime + Pbz * ForcastTime + Pcz);
    
    return 0;
}
