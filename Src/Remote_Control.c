/**
  ******************************************************************************
  * @file    Remote_Control.c
  * @author  DJI 
  * @version V1.0.0
  * @date    2015/11/15
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "Remote_Control.h"
#include "pid.h"

RC_Type remote_control;
uint32_t  Latest_Remote_Control_Pack_Time = 0;
uint32_t  LED_Flash_Timer_remote_control = 0;
/*******************************************************************************************
  * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief  DR16���ջ�Э��������
  * @Param		RC_Type* rc���洢ң�������ݵĽṹ�塡��uint8_t* buff�����ڽ���Ļ���
  * @Retval		None
  * @Date    
 *******************************************************************************************/
void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
//	rc->ch1 = (*buff | *(buff+1) << 8) & 0x07FF;	offset  = 1024
	rc->ch1 = (buff[0] | buff[1]<<8) & 0x07FF;
	rc->ch1 -= 1024;
	rc->ch2 = (buff[1]>>3 | buff[2]<<5 ) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (buff[2]>>6 | buff[3]<<2 | buff[4]<<10) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (buff[4]>>1 | buff[5]<<7) & 0x07FF;		
	rc->ch4 -= 1024;
	
	rc->switch_left = ( (buff[5] >> 4)& 0x000C ) >> 2;
	rc->switch_right =  (buff[5] >> 4)& 0x0003 ;
	
	rc->mouse.x = buff[6] | (buff[7] << 8);	//x axis
	rc->mouse.y = buff[8] | (buff[9] << 8);
	rc->mouse.z = buff[10]| (buff[11] << 8);
	
	rc->mouse.press_left 	= buff[12];	// is pressed?
	rc->mouse.press_right = buff[13];
	
	rc->keyBoard.key_code = buff[14] | buff[15] << 8; //key borad code
	
	Latest_Remote_Control_Pack_Time = HAL_GetTick();
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500){
			
			HAL_GPIO_TogglePin(LED_T_GPIO_Port,LED_T_Pin);
			
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;
		
			
	}
	
}

extern uint16_t TIM_COUNT[];
int16_t HighTime;


/*******************************************************************************************
  * @Func		void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
  * @Brief  PWM���ջ��������
  * @Param		TIM_HandleTypeDef *htim ���ڲ���PWM����Ķ�ʱ����
  * @Retval		None
  * @Date    
 *******************************************************************************************/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	HighTime = (TIM_COUNT[1] - TIM_COUNT[0])>0?(TIM_COUNT[1] - TIM_COUNT[0]):((TIM_COUNT[1] - TIM_COUNT[0])+10000);
	
	Latest_Remote_Control_Pack_Time = HAL_GetTick();
			
  remote_control.ch4 = (HighTime - 4000)*660/4000;
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500){
			
			HAL_GPIO_TogglePin(LED_T_GPIO_Port,LED_T_Pin);		
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;
					
	}
	
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
int16_t moto_ctr[6];
//ң�ؿ���
const int THRESHOLD=2500;  //�����ٶ�
const int TURNSPEED=2500;   //ת�伫���ٶ�
int yaw_control=0;
void DBUS_Deal()
{
	if(remote_control.switch_left==1)//��imu�ںϵ�������
	{
		moto_ctr[0]=remote_control.ch2+remote_control.ch1;
		moto_ctr[1]=-remote_control.ch2+remote_control.ch1;   //�Ƴ�����ͨ���Ե���������ת�Ŀ��ƣ���3ͨ�����ڿ���yawƫת  2018.11.21  12��16  �޸� �ܺ�
		moto_ctr[2]=-remote_control.ch2-remote_control.ch1;
		moto_ctr[3]=remote_control.ch2-remote_control.ch1;
//		/************�趨��λ������Ϊ20  ************/
//		if(remote_control.ch3<-40)
//			yaw_control=(remote_control.ch3+40)*150/620;
//		if(remote_control.ch3>40)
//			yaw_control=(remote_control.ch3-40)*150/620;
//		else
//			yaw_control=0;
	}
	if(remote_control.switch_left==2)
	{
		moto_ctr[0]=remote_control.ch2+remote_control.ch3+remote_control.ch1;
		moto_ctr[1]=-remote_control.ch2+remote_control.ch3+remote_control.ch1;
		moto_ctr[2]=-remote_control.ch2+remote_control.ch3-remote_control.ch1;   
		moto_ctr[3]=remote_control.ch2+remote_control.ch3-remote_control.ch1;
			
	}
	motor_pid[0].target=moto_ctr[0]*2500/660;
   motor_pid[1].target=moto_ctr[1]*2500/660;
   motor_pid[2].target=moto_ctr[2]*2500/660;
   motor_pid[3].target=moto_ctr[3]*2500/660;
	
	if((motor_pid[0].target>THRESHOLD)&&(motor_pid[1].target>THRESHOLD)&&(motor_pid[2].target>THRESHOLD)&&(motor_pid[3].target>THRESHOLD))			//˳ת�ٶȿ���
		{
			motor_pid[0].target=TURNSPEED;
			motor_pid[1].target=TURNSPEED;
			motor_pid[2].target=TURNSPEED;
			motor_pid[3].target=TURNSPEED;
		}
		
		if((motor_pid[0].target<-THRESHOLD)&&(motor_pid[1].target<-THRESHOLD)&&(motor_pid[2].target<-THRESHOLD)&&(motor_pid[3].target<-THRESHOLD))			//˳ת�ٶȿ���
		{
	    motor_pid[0].target=-TURNSPEED;
			motor_pid[1].target=-TURNSPEED;
			motor_pid[2].target=-TURNSPEED;
			motor_pid[3].target=-TURNSPEED;
		}	  
}
