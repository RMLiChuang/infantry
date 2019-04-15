/**
  ******************************************************************************
  * File Name          : robomaster_vision.c
	* author						 : 周恒
	* Date							 :2019.1.23
  * Description        :视觉处理文件
	
  ******************************************************************************
  ******************************************************************************
  */

#include "robomaster_vision.h"

#define MAX_ERR 600//视觉误差限幅
#define MIN_ERR 0



#define VISION_DATE_SIZE 6

uint16_t UART6_Date[8]={0};//串口6接收pid调参数据

u8 Usart_Flag=0;
Vision_Attack Armour_attack;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	printf("test");
	 if(huart == &huart6)
  {
//		printf("test");
		__HAL_UART_CLEAR_PEFLAG(&huart6);
		
		
		
		HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);
	}
}

/**********************************************************************************************************
*函 数 名: armour_attack
*功能说明: 视觉识别装甲板并攻击，   控制思路，只有当鼠标偏移量或遥控器的偏移量小于一定值的时候(即中位死区)，才能激活视觉识别
*形    参: 需要视觉提供云台相对于装甲板的yaw和pitch的err
*返 回 值: 电流输出
**********************************************************************************************************/
void armour_attack()
{
//	/***********YAW轴偏差矫正***************/
////	vision_yaw.target=300;
////	vision_yaw.f_cal_pid(&vision_yaw,Armour_attack.pan_tilt_angel_err.Yaw_Err);
//	int yaw_err,pitch_err;
//	yaw_err=(Armour_attack.pan_tilt_angel_err.Yaw_Err-VISION_YAW_TARGET);
//	if(int_abs(yaw_err)<10)//视觉误差小于一定值
//	{
//		if(pan_tilt_yaw.target==0)  //回中时赋角度期望值
//			{
//				pan_tilt_yaw.target=imu.yaw;
//			}
//			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
//			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
//	}
//	else
//	{
//		pan_tilt_yaw.target=0;//偏航角期望给0,不进行角度控制
//		pan_tilt_yaw_speed.target=yaw_err;
//	}
//	//pan_tilt_yaw_speed.f_cal_pid(&pan_tilt_yaw_speed,-imu.gz);
//	
//	/***********PITCH轴偏差矫正***************/
////	vision_pitch.target=300;
////	vision_pitch.f_cal_pid(&vision_pitch,Armour_attack.pan_tilt_angel_err.Pitch_Err);
////	pan_tilt_pitch_speed.target=-vision_pitch.output;
//	pitch_err=(Armour_attack.pan_tilt_angel_err.Pitch_Err-VISION_PIT_TARGET)*1.3;
//	if(int_abs(pitch_err)<5)//视觉误差小于一定值
//	{
//		if(pan_tilt_pitch.target==0)  //回中时赋角度期望值
//			{
//				pan_tilt_pitch.target=imu.pit;
//			}
//			PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
//			pan_tilt_pitch_speed.target=pan_tilt_pitch.output;
//	}
//	else
//	{
//		pan_tilt_pitch.target=0;////偏航角期望给0,不进行角度控制
//		pan_tilt_pitch_speed.target=pitch_err;
//	}

	//pan_tilt_pitch_speed.f_cal_pid(&pan_tilt_pitch_speed,-imu.gy);
	
	
	/***********YAW轴偏差矫正***************/
	vision_yaw.target=VISION_YAW_TARGET;
	vision_yaw.f_cal_pid(&vision_yaw,Armour_attack.pan_tilt_angel_err.Yaw_Err);
	pan_tilt_yaw_speed.target=-vision_yaw.output;
	if(int_abs(vision_yaw.err)<40&&int_abs(vision_pitch.err)<40)
		pan_tilt_yaw_speed.f_cal_pid(&pan_tilt_yaw_speed,-imu.gz);
	else
		pan_tilt_yaw_speed.output=0;
	/***********PITCH轴偏差矫正***************/
	vision_pitch.target=VISION_PIT_TARGET;
	vision_pitch.f_cal_pid(&vision_pitch,Armour_attack.pan_tilt_angel_err.Pitch_Err);
	pan_tilt_pitch_speed.target=-vision_pitch.output;
	if(int_abs(vision_yaw.err)<40&&int_abs(vision_pitch.err)<40)
		pan_tilt_pitch_speed.f_cal_pid(&pan_tilt_pitch_speed,-imu.gy);
	else
		pan_tilt_pitch_speed.output=0;
}
/**********************************************************************************************************
*函 数 名: get_armour_err
*功能说明: 获取装甲板相对于云台的误差
*形    参:需要视觉提供云台相对于装甲板的yaw和pitch的err
*返 回 值: 
**********************************************************************************************************/
void  get_armour_err()
{
	if(USART6_RX_BUF==1)
	{
		USART6_RX_BUF=0;
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
			//限幅
			Armour_attack.pan_tilt_angel_err.Yaw_Err=int16_constrain(Armour_attack.pan_tilt_angel_err.Yaw_Err,MIN_ERR,MAX_ERR);
			Armour_attack.pan_tilt_angel_err.Pitch_Err=int16_constrain(Armour_attack.pan_tilt_angel_err.Pitch_Err,MIN_ERR,MAX_ERR);
			//滤波
//			Armour_attack.pan_tilt_angel_err.Yaw_Err=Butterworth_Filter(Armour_attack.pan_tilt_angel_err.Yaw_Err,&Vision_BufferData[0],&Accel_Parameter);
//			Armour_attack.pan_tilt_angel_err.Pitch_Err=Butterworth_Filter(Armour_attack.pan_tilt_angel_err.Pitch_Err,&Vision_BufferData[1],&Accel_Parameter);
			
	
//			Armour_attack.pan_tilt_angel_err.Yaw_Err=GildeAverageValueFilter(Armour_attack.pan_tilt_angel_err.Yaw_Err,Data);
//			Armour_attack.pan_tilt_angel_err.Pitch_Err=GildeAverageValueFilter(Armour_attack.pan_tilt_angel_err.Pitch_Err,Data);
		}
//		for(i=0;i<8;i++)
//		{
		
//			send_data(UART6_Date[i]);
//		}
		//printf("%d",UART6_Date[3]);
//		printf("%d  ",Armour_attack.pan_tilt_angel_err.Yaw_Err);
//		printf("%d  ",Armour_attack.pan_tilt_angel_err.Pitch_Err);
//		printf("%d  ",Armour_attack.check_sum);
//		printf("\r\n");
	}
	
}


//串口6接收中断服务程序
//帧头0xfe
uint32_t  Latest_Vision_Control_Pack_Time = 0;
uint32_t vision_time=0;//用于检测使视觉识别是否离线
uint8_t counter=0;
uint8_t USART6_RX_BUF=0;

void USART6_IRQHandler(void)                	
{
	uint8_t Res;
	
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_RXNE)!=RESET))//接收中断
	{
        HAL_UART_Receive(&huart6,&Res,1,1000); 
	}
	//协议为5a a5 x[0] x[1] y[0] y[1] sum[0] sum[1]
	
	Latest_Vision_Control_Pack_Time = HAL_GetTick();		
	UART6_Date[counter] =Res;	
		
		if(counter == 0 && UART6_Date[0] != 0x5a)
		{
			counter=0;
			return;
		}
		if(counter == 1 && UART6_Date[1] != 0xa5)
		{
			counter=0;
			return;
		}
		//send_data(counter);
			counter++;		
		if(counter==8) 
		{ 
			counter=0; 
			USART6_RX_BUF=1;	
		}
		get_armour_err();
		vision_time=Latest_Vision_Control_Pack_Time;
		robot_status.vision_status=VISION_SUCCESS;//有视觉信息，正常
		HAL_GPIO_WritePin(LED_USER_GPIO_PORT, LED_D_Pin,GPIO_PIN_SET);
}                             




//void  get_armour_err(unsigned char *str)
//{
//    char i=0;
//    char success=0;
//  
//    for(i=0;i<8;i++)
//    {
//        if((str[i]==0x03)&&(str[i+1]==0xfc))
//        {
//          success=1;
//          break;
//        }
//    }
//    
//    if(success==0x01)
//    {
//      Armour_attack.pan_tilt_angel_err.Yaw_Err=((str[i+3]<<8)+str[i+2]);
//      Armour_attack.pan_tilt_angel_err.Pitch_Err=((str[i+5]<<8)+str[i+4]);
//    }   
//		else
//		{
//			Armour_attack.pan_tilt_angel_err.Yaw_Err=500;
//			Armour_attack.pan_tilt_angel_err.Pitch_Err=500;
//		}
//}


