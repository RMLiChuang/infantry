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
#include "robomaster_common.h"
#define INFANTRY_MAX_SPEED 5000
#define chassis_speed_distribute 1.3 //底盘1，2号电机相对于3，4号电机速度分配
#define twist_straight_speed_coefficient 0.4f //扭腰走直线时的速度系数
const int THRESHOLD=5000;  //极限速度
const int TURNSPEED=5000;   //转弯极限速度
/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定是发送1024过来，
  * @author         RM
  * @param[in]      输入的遥控器值
  * @param[in]      输出的死区处理后遥控器值
  * @param[in]      死区值
  * @retval         返回空
  */




RC_Type remote_control;
uint32_t  Latest_Remote_Control_Pack_Time = 0;
uint32_t  LED_Flash_Timer_remote_control = 0;
uint32_t dbus_time=0;//用于检测遥控器是否离线
/*******************************************************************************************
  * @Func		void Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
  * @Brief  DR16接收机协议解码程序
  * @Param		RC_Type* rc　存储遥控器数据的结构体　　uint8_t* buff　用于解码的缓存
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
	
	if(Latest_Remote_Control_Pack_Time - LED_Flash_Timer_remote_control>500) //让LED_T每1秒闪烁一次
		{
			
			HAL_GPIO_TogglePin(LED_T_GPIO_Port,LED_T_Pin);
			
			LED_Flash_Timer_remote_control = Latest_Remote_Control_Pack_Time;
			
			
			
	}
	robot_status.anomaly=NORMAL;//判断遥控器接收正常
	dbus_time=Latest_Remote_Control_Pack_Time;//获取时间
	HAL_GPIO_WritePin(LED_USER_GPIO_PORT, LED_B_Pin,GPIO_PIN_SET);
}

extern uint16_t TIM_COUNT[];
int16_t HighTime;


/*******************************************************************************************
  * @Func		void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
  * @Brief  PWM接收机脉宽计算
  * @Param		TIM_HandleTypeDef *htim 用于测量PWM脉宽的定时器。
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
                      底盘电机控制2017(上一届)

1, 就算出四种姿态的输出
2，四种姿态的叠加
3，准备CAN的数据输出
4，开始电机控制
__________________________________________________________________
| 电机位置 |电机号| 输出\方向 | 前 | 后 | 左 | 右 | 顺 | 逆 | 停 | 													前			后
``````````````````````````````````````````````````````````````````
|   左前   |0X201 |moto_ctr[0]| >0 | <0 | <0 | >0 | >0 | <0 | =0 |      									>0	|		<0
``````````````````````````````````````````````````````````````````	
|   右前   |0X202 |moto_ctr[1]| <0 | >0 | <0 | >0 | >0 | <0 | =0 |												<0	|		>0
``````````````````````````````````````````````````````````````````   //2018.10.6 修改 												2018.10.7（经测试，原版没问题）
|   左后   |0X203 |moto_ctr[2]| <0 | >0 | >0 | <0 | >0 | <0 | =0 |												>0  |		<0
``````````````````````````````````````````````````````````````````	
|   右后   |0X204 |moto_ctr[3]| >0 | <0 | >0 | <0 | >0 | <0 | =0 |												<0	|		>0
``````````````````````````````````````````````````````````````````
|  备注:|moto_ctr[0]|=|moto_ctr[1]|=|moto_ctr[2]|=|moto_ctr[3]|  |
``````````````````````````````````````````````````````````````````											
*************************** ***************************/
float twist_distribute;//扭腰走直线
void cal_twist_distribute()
{
	twist_distribute=((arm_cos_f32(int_abs(chassis_move.chassis_relative_angle))-arm_sin_f32(int_abs(chassis_move.chassis_relative_angle)))
										/(arm_cos_f32(int_abs(chassis_move.chassis_relative_angle))+arm_sin_f32(int_abs(chassis_move.chassis_relative_angle))));
}


int16_t moto_ctr[6];
//遥控控制

int yaw_control=0;
int translation=0,straight=0;
void DBUS_Deal()
{
//	if(remote_control.switch_left==2)//将imu融合到底盘中
//	{
//		moto_ctr[0]=remote_control.ch2+remote_control.ch1;
//		moto_ctr[1]=-remote_control.ch2+remote_control.ch1;   //移除第三通道对底盘左右旋转的控制，将3通道用于控制yaw偏转  2018.11.21  12：16  修改 周恒
//		moto_ctr[2]=-remote_control.ch2-remote_control.ch1;
//		moto_ctr[3]=remote_control.ch2-remote_control.ch1;
////		/************设定中位死区，为20  ************/
////		if(remote_control.ch3<-40)
////			yaw_control=(remote_control.ch3+40)*150/620;
////		if(remote_control.ch3>40)
////			yaw_control=(remote_control.ch3-40)*150/620;
////		else
////			yaw_control=0;
//	}
//	if(remote_control.switch_left==1)
//	{
//		moto_ctr[0]=remote_control.ch2+remote_control.ch3+remote_control.ch1;
//		moto_ctr[1]=-remote_control.ch2+remote_control.ch3+remote_control.ch1;
//		moto_ctr[2]=-remote_control.ch2+remote_control.ch3-remote_control.ch1;   
//		moto_ctr[3]=remote_control.ch2+remote_control.ch3-remote_control.ch1;
//			
//	}
		rc_deadline_limit(remote_control.ch3,remote_control.ch3,CHASSIS_RC_DEADLINE);
		if(robot_status.mode==TWIST)//当车再扭腰模式时
		{
			cal_twist_distribute();
			if(pan_tilt_angle>0)//底盘相对于云台为正方向
			{
				moto_ctr[0]=straight+translation+yaw_velocity_target*chassis_speed_distribute*0.3;
				moto_ctr[1]=-straight*twist_distribute+translation+yaw_velocity_target*chassis_speed_distribute*0.3;   //移除第三通道对底盘左右旋转的控制，将3通道用于控制yaw偏转  2018.11.21  12：16  修改 周恒
				moto_ctr[2]=-straight-translation+yaw_velocity_target*0.3;
				moto_ctr[3]=straight*twist_distribute-translation+yaw_velocity_target*0.3;
			}
			else//底盘相对于云台为
			{
				moto_ctr[0]=straight*twist_distribute*twist_straight_speed_coefficient+translation+yaw_velocity_target*chassis_speed_distribute*0.4f;
				moto_ctr[1]=-straight*twist_straight_speed_coefficient+translation+yaw_velocity_target*chassis_speed_distribute*0.4f;   //移除第三通道对底盘左右旋转的控制，将3通道用于控制yaw偏转  2018.11.21  12：16  修改 周恒
				moto_ctr[2]=-straight*twist_distribute*twist_straight_speed_coefficient-translation+yaw_velocity_target*0.4f;
				moto_ctr[3]=straight*twist_straight_speed_coefficient-translation+yaw_velocity_target*0.4f;
			}
		}
		else//当车不在扭腰模式时
		{
			moto_ctr[0]=straight+yaw_velocity_target*chassis_speed_distribute*0.4f+translation;
			moto_ctr[1]=-straight+yaw_velocity_target*chassis_speed_distribute*0.4f+translation;
			moto_ctr[2]=-straight+yaw_velocity_target*chassis_speed_distribute*0.4f-translation;   
			moto_ctr[3]=straight+yaw_velocity_target*chassis_speed_distribute*0.4f-translation;
		}
	
			motor_pid[0].target=moto_ctr[0]*INFANTRY_MAX_SPEED/660;
			motor_pid[1].target=moto_ctr[1]*INFANTRY_MAX_SPEED/660;
			motor_pid[2].target=moto_ctr[2]*INFANTRY_MAX_SPEED/660;
			motor_pid[3].target=moto_ctr[3]*INFANTRY_MAX_SPEED/660;
	
		
		if((motor_pid[0].target>THRESHOLD)&&(motor_pid[1].target>THRESHOLD)&&(motor_pid[2].target>THRESHOLD)&&(motor_pid[3].target>THRESHOLD))			//顺转速度控制
		{
			motor_pid[0].target=TURNSPEED;
			motor_pid[1].target=TURNSPEED;
			motor_pid[2].target=TURNSPEED;
			motor_pid[3].target=TURNSPEED;
		}
		
		if((motor_pid[0].target<-THRESHOLD)&&(motor_pid[1].target<-THRESHOLD)&&(motor_pid[2].target<-THRESHOLD)&&(motor_pid[3].target<-THRESHOLD))			//顺转速度控制
		{
	    motor_pid[0].target=-TURNSPEED;
			motor_pid[1].target=-TURNSPEED;
			motor_pid[2].target=-TURNSPEED;
			motor_pid[3].target=-TURNSPEED;
		}	  
}

