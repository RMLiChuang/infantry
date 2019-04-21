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
Enemy_Struct EnemyDataBuffer[ENEMYDATABUFFERLENGHT];
uint8_t EnemyDataBufferPoint;//主机解码数据缓存指针
#define MAX_ERR 600//视觉误差限幅
#define MIN_ERR 0


#define YAW_AUTO_SHOT_THRESHOLD 3//视觉自动打击阈值设置 单位为像素点
#define PIT_AUTO_SHOT_THRESHOLD 15
#define VISION_DATE_SIZE 6

uint8_t UART6_Date[8]={0};//串口6接收pid调参数据
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
  * @brief  视觉初始化
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
*函 数 名: armour_attack
*功能说明: 视觉识别装甲板并攻击，   控制思路，只有当鼠标偏移量或遥控器的偏移量小于一定值的时候(即中位死区)，才能激活视觉识别
*形    参: 需要视觉提供云台相对于装甲板的yaw和pitch的err
*返 回 值: 电流输出
**********************************************************************************************************/
void armour_attack()
{
	/***********YAW轴偏差矫正***************/
	vision_yaw.target=VISION_YAW_TARGET;
	vision_yaw.f_cal_pid(&vision_yaw,Armour_attack.pan_tilt_angel_err.Yaw_Err);
	pan_tilt_yaw_speed.target=-vision_yaw.output;
	pan_tilt_yaw_speed.f_cal_pid(&pan_tilt_yaw_speed,-imu.gz);
	/***********PITCH轴偏差矫正***************/
	vision_pitch.target=VISION_PIT_TARGET;
	vision_pitch.f_cal_pid(&vision_pitch,Armour_attack.pan_tilt_angel_err.Pitch_Err);
	pan_tilt_pitch_speed.target=-vision_pitch.output;
	pan_tilt_pitch_speed.f_cal_pid(&pan_tilt_pitch_speed,-imu.gy);
	/**********使视觉和手动平稳过渡*******/
	pan_tilt_yaw.target=imu.yaw;//
	pan_tilt_pitch.target=imu.pit;
/**********视觉识别完成后进行自动打击*******/
	if(int_abs(vision_yaw.err)<YAW_AUTO_SHOT_THRESHOLD&&int_abs(vision_pitch.err)<PIT_AUTO_SHOT_THRESHOLD)//当目标对准装甲板时，自动射击，提高命中率
	{
		if (shoot_mode != SHOOT_DONE && trigger_motor.key == SWITCH_TRIGGER_ON)
     {
			 if(robot_status.fric_mode!=STOP)//防止未开摩擦轮的时候拨动拨轮
          shoot_mode = SHOOT_BULLET;
     }
	}
	else
	{
		 shoot_mode = SHOOT_READY;
	}
}
/**********************************************************************************************************
*函 数 名: get_armour_err
*功能说明: 获取装甲板相对于云台的误差
*形    参:需要视觉提供云台相对于装甲板的yaw和pitch的err
*返 回 值: 
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
			//限幅
			Armour_attack.pan_tilt_angel_err.Yaw_Err=int16_constrain(Armour_attack.pan_tilt_angel_err.Yaw_Err,MIN_ERR,MAX_ERR);
			Armour_attack.pan_tilt_angel_err.Pitch_Err=int16_constrain(Armour_attack.pan_tilt_angel_err.Pitch_Err,MIN_ERR,MAX_ERR);
			
			//滤波
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
			robot_status.vision_status=VISION_SUCCESS;//有视觉信息，正常
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);//测试
		}

	}



//串口6接收中断服务程序
//帧头0xfe

uint8_t counter=0;
uint8_t USART6_RX_BUF=0;
void USART6_IRQHandler(void)                	
{
	HAL_UART_IDLE_IRQHandler(&huart6);	
}                             


/**
  * @brief  路径拟合核心函数，根据输入进行一次拟合
  * @param  预判样本时间长度
  * @param  预判时间
  * @param  预判位置
  * @retval 0 成功        1 失败
  */
uint8_t ForcastCore(uint16_t SampleTime, uint16_t ForcastTime, Point_Struct *ForcastPoint)
{
    int RelativeTime;                       //相对时间，防止绝对时间超范围
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
    
    //寻找起点
    for(SampleNum = 0; SampleNum < ENEMYDATABUFFERLENGHT; SampleNum++)
    {
        if(EnemyDataBuffer[(EnemyDataBufferPoint + ENEMYDATABUFFERLENGHT - SampleNum - 1) % 
					ENEMYDATABUFFERLENGHT].Time + SampleTime < EnemyDataBuffer[EnemyDataBufferPoint].Time)//取过去300ms时间内的数据进行拟合
        {
            break;
        }
    }
    
    //拟合数据量过少
    if(SampleNum < 5)
    {
        return 1;
    }
    
    E =  -(1 + SampleNum);
    
    //数据拟合
    for(index = 0; index <= SampleNum; index++)
    {
        Currentindex = (EnemyDataBufferPoint + ENEMYDATABUFFERLENGHT - index) % ENEMYDATABUFFERLENGHT;
        
        RelativeTime = EnemyDataBuffer[Currentindex].Time - EnemyDataBuffer[(EnemyDataBufferPoint + ENEMYDATABUFFERLENGHT - SampleNum) % ENEMYDATABUFFERLENGHT].Time;//        
        //在循环过程中 RelativeTime不断减少
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
    
    //数据非法
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
