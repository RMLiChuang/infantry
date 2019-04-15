#include "robomaster_vcan.h"
#include "usart.h"

//该.c文件主要是配合山外调试助手发送波形
short  wave_form_data[6] = {0};
void send_data(uint8_t date)
{
	HAL_UART_Transmit(&huart8,&date,1,10);
	//while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  
	
}
void shanwai_send_wave_form(void)
{
	uint8_t i;
	
	send_data(0x03);
	send_data(0xfc);
	for(i = 0;i<6;i++)
	{
	  send_data((wave_form_data[i]&0xff)); //先发送低位再发送高位
	  send_data((wave_form_data[i]>>8));
		
	  
	}
	send_data(0xfc);
	send_data(0x03);
}	

void debug_program()
{
	/*		wave_form_data[0] =motor_pid[6].output;
	  wave_form_data[1] =motor_pid[6].target;
	  wave_form_data[2] =moto_chassis[6].speed_rpm;
	  wave_form_data[3] =moto_chassis[6].angle;
	  wave_form_data[4] =moto_chassis[6].total_angle;//moto_chassis[0].real_current;
	  wave_form_data[5] =moto_chassis[6].real_current;//moto_chassis[0].hall;
	*/  
//	  wave_form_data[0] =(short)imu.yaw;
//	  wave_form_data[1] =(short)chassis_yaw.target;
//	  wave_form_data[2] =(short)((imu.mx/imu.my)*100);//chassis_yaw.output;
//	  wave_form_data[3] =(short)chassis_yaw_speed.output;
//	  wave_form_data[4] =(short)HAL_GetTick();//imu.gz;
//	  wave_form_data[5] =(short)chassis_yaw_speed.target;
		
//		wave_form_data[0] =(short)imu.yaw;
//	  wave_form_data[1] =(short)chassis_yaw.target;//MPU_Set_Accel_Fsr(2);
//	  wave_form_data[2] =(short)-imu.gz;//chassis_yaw.output;
//	  wave_form_data[3] =(short)chassis_yaw_speed.target;//mag_field_intensity;//磁场强度;
//	  wave_form_data[4] =(short)chassis_yaw.output;//imu.gz;
//	  wave_form_data[5] =(short)chassis_yaw_speed.output;

//底盘电机跟随云台闭环
//			wave_form_data[0] =(short)chassis_yaw_angle.target;    //YAW ID:206
//	    wave_form_data[1] =(short)chassis_yaw_angle.output;
//	    wave_form_data[2] =(short)moto_chassis[5].angle;
//	    wave_form_data[3] =(short)imu.gz;
//	    wave_form_data[4] =(short)pan_tilt_yaw_speed.output;      //PITCH ID:205
//	    wave_form_data[5] =(short)imu.pit;

//yaw轴imu角度反馈调试
//    wave_form_data[0] =(short)pan_tilt_yaw.target;    //YAW ID:206
//	  wave_form_data[1] =(short)motor_pid[5].output;
//	  wave_form_data[2] =(short)pan_tilt_yaw_speed.target;
//	  wave_form_data[3] =(short)-imu.gz;
//		wave_form_data[4] =(short)pan_tilt_yaw_speed.output;      //PITCH ID:205
//	  wave_form_data[5] =(short)imu.yaw;

////pitch轴imu角度反馈调试
//    wave_form_data[0] =(short)pan_tilt_pitch.target;      //YAW ID:206
//	  wave_form_data[1] =(short)pan_tilt_pitch.output;
//	  wave_form_data[2] =(short)pan_tilt_pitch_speed.target;
//	  wave_form_data[3] =(short)imu.gy;//pan_tilt_yaw_speed.output;
//		wave_form_data[4] =(short)pan_tilt_pitch_speed.output;//motor_pid[5].err;      //PITCH ID:205
//	  wave_form_data[5] =(short)imu.pit;//moto_chassis[4].angle;

////云台归中的yaw和pitch机械角度调试
//		wave_form_data[0] =(short)motor_pid[5].target;    //YAW ID:206
//	  wave_form_data[1] =(short)motor_pid[5].err;
//	  wave_form_data[2] =(short)moto_chassis[5].angle;
//	  wave_form_data[3] =(short)motor_pid[4].target; 
//		wave_form_data[4] =(short)motor_pid[4].output;     //PITCH ID:205
//	  wave_form_data[5] =(short)moto_chassis[4].angle;

//检测can通信的6个电机是否有数据
//		wave_form_data[0] =(short)refSysData.PowerHeatData_t.chassisPower;//chassis_yaw_speed.output;      //YAW ID:206
//		wave_form_data[1] =(short)refSysData.PowerHeatData_t.chassisPowerBuffer;
//			wave_form_data[0]=refSysData.PowerHeatData_t.chassisPower;//功率
//			wave_form_data[1]=refSysData.PowerHeatData_t.chassisPowerBuffer;//60J能量缓冲
//			wave_form_data[2]=(MY_ADC_GetValue()*(100.0/2.4));//电容电量
//			wave_form_data[3]=(Calc_Cap_Power(MY_ADC_GetValue(),Cap_In_flag));//电容实时充电功率
//			wave_form_data[4]=control_state*30;//控制状态标志
//	  wave_form_data[1] =(short)imu.gz;
//	  wave_form_data[2] =(short)chassis_yaw_angle.output;
//	  wave_form_data[3] =(short)chassis_yaw_angle.target;//motor_pid[3].target;//pan_tilt_yaw_speed.output;
//		wave_form_data[4] =(short)uart6_rx_buff[4];//moto_chassis[4].angle;//moto_chassis[6].round_cnt;//motor_pid[5].err;      //PITCH ID:205
//		wave_form_data[5] =(short)Armour_attack.pan_tilt_angel_err.Yaw_Err;//refSysData.PowerHeatData_t.chassisPower;

//键盘鼠标控制
//		wave_form_data[0] =(short)remote_control.mouse.x;
//	  wave_form_data[1] =(short)remote_control.mouse.y;//MPU_Set_Accel_Fsr(2);
//	  wave_form_data[2] =(short)yaw_velocity_target;//remote_control.mouse.press_left;//chassis_yaw.output;
//	  wave_form_data[3] =(short)pitch_velocity_target;//remote_control.mouse.press_right;//mag_field_intensity;//磁场强度;
//	  wave_form_data[4] =(short)imu.pit;//remote_control.keyBoard.key_code;//;
//	  wave_form_data[5] =(short)pan_tilt_pitch.target;//chassis_yaw_speed.output;

//视觉信息
		wave_form_data[0] =(short)pan_tilt_pitch.output;
	  wave_form_data[1] =(short)imu.gy;//chassis_yaw_angle.initial;
	  wave_form_data[2] =(short)Armour_attack.pan_tilt_angel_err.origin_yaw;
	  wave_form_data[3] =(short)Armour_attack.pan_tilt_angel_err.origin_pitch;
		wave_form_data[4] =(short)Armour_attack.pan_tilt_angel_err.Yaw_Err;//pan_tilt_yaw_motor.angle;//vision_yaw.output;;//moto_chassis[4].angle;//moto_chassis[6].round_cnt;//motor_pid[5].err;      //PITCH ID:205
		wave_form_data[5] =(short)Armour_attack.pan_tilt_angel_err.Pitch_Err;//mpu_data.gy;//refSysData.PowerHeatData_t.chassisPower;
//底盘相关数据
//		wave_form_data[0] =(short)(chassis_move.vx*100);
//	  wave_form_data[1] =(short)(chassis_move.vy*100);
//	  wave_form_data[2] =(short)pan_tilt_yaw_motor.angle;//(pan_tilt_yaw_angle*100);//(chassis_move.wz*100);
//	  wave_form_data[3] =(short)(chassis_move.vx_set*100);
//	  wave_form_data[4] =(short)(chassis_move.vy_set*100);//pan_tilt_pit_angle;//moto_chassis[4].angle;//chassis_move.chassis_pitch;//moto_chassis[0].real_current;
//	  wave_form_data[5] =(short)(chassis_move.wz_set*100);//moto_chassis[0].hall;


		shanwai_send_wave_form();   //将数据传输到三外上位机，可以看到实时波形
}
