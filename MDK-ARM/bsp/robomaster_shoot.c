/**
  ******************************************************************************
  * File Name          : robomaster_shoot.c
	* author						 : 周恒
	* Date							 :2019.1.23
  * Description        :拨弹轮驱动
	
  ******************************************************************************
  ******************************************************************************
  */

#include "robomaster_shoot.h"

Shoot_Motor_t trigger_motor;          //射击数据
shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机
//微动开关IO
#define Butten_Trig_Pin HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)
//拨弹速度
#define TRIGGER_SPEED 11.0f
#define Ready_Trigger_Speed 7.0f

#define KEY_OFF_JUGUE_TIME 200
//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME 1
#define SWITCH_TRIGGER_ON 1
#define SWITCH_TRIGGER_OFF 0
/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void);
/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);
/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
/**
  * @brief          射击完成控制，判断微动开关一段时间无子弹来判断一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void);
/**
  * @brief          射击准备控制，将子弹送到微动开关处，
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void);
/**
  * @brief          射击初始化，初始化PID，遥控器指针，电机指针
  * @author         RM
  * @param[in]      void
  * @retval         返回空
  */
void shoot_init(void)
{
    //更新数据
    Shoot_Feedback_Update();
		ramp_init(&fric_ramp,SHOOT_CONTROL_TIME * 0.001f, Close_Fric_ON, Fric_OFF);
	
    trigger_motor.ecd_count = 0;
    trigger_motor.angle = moto_chassis[6].angle * Motor_ECD_TO_ANGLE;
    trigger_motor.given_current = 0;
    trigger_motor.move_flag = 0;
    trigger_motor.set_angle = trigger_motor.angle;
    trigger_motor.speed = 0.0f;
    trigger_motor.speed_set = 0.0f;
    trigger_motor.BulletShootCnt = 0;
}
/**
  * @brief          射击循环
  * @author         RM
  * @param[in]      void
  * @retval         返回can控制值
  */
int16_t shoot_control_loop(void)
{
    int16_t shoot_CAN_Set_Current; //返回的can值
//		oled_shownum(3,4, trigger_motor.BulletShootCnt,0,3);
    Shoot_Set_Mode();        //设置状态机
    Shoot_Feedback_Update(); //更新数据

    //发射状态控制
    if (shoot_mode == SHOOT_BULLET)
    {
//			trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
//			trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    //发射完成状态控制
    else if (shoot_mode == SHOOT_DONE)
    {
        shoot_done_control();
    }
    //发射准备状态控制
    else if (shoot_mode == SHOOT_READY)
    {
//        trigger_motor_pid.max_out = TRIGGER_READY_PID_MAX_OUT;
//        trigger_motor_pid.max_iout = TRIGGER_READY_PID_MAX_IOUT;
        shoot_ready_control();
    }

    if (shoot_mode == SHOOT_STOP)
    {
        shoot_CAN_Set_Current = 0;
				
    }
    else
    {
//        //摩擦轮pwm
//        static uint16_t fric_pwm1 = Fric_OFF;
//        static uint16_t fric_pwm2 = Fric_OFF;


//        //shoot_laser_on();       //激光开启


//        //摩擦轮需要一个个斜波开启，不能同时直接开启，否则可能电机不转
//        ramp_calc(&trigger_motor.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

//        if(trigger_motor.fric1_ramp.out == trigger_motor.fric1_ramp.max_value)
//        {
//            ramp_calc(&trigger_motor.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
//        }

//        if( trigger_motor.fric2_ramp.out != trigger_motor.fric2_ramp.max_value)
//        {
//            trigger_motor.speed_set = 0.0f;
//        }


////鼠标右键按下加速摩擦轮，使得左键低速射击， 右键高速射击
//        static uint16_t up_time = 0;
//        if (trigger_motor.press_r)
//        {
//            up_time = UP_ADD_TIME;
//        }

//        if (up_time > 0)
//        {
//            trigger_motor.fric1_ramp.max_value = Fric_UP;
//            trigger_motor.fric2_ramp.max_value = Fric_UP;
//            up_time--;
//        }
//        else
//        {
//            trigger_motor.fric1_ramp.max_value = Fric_DOWN;
//            trigger_motor.fric2_ramp.max_value = Fric_DOWN;
//        }

//        fric_pwm1 = (uint16_t)(trigger_motor.fric1_ramp.out);
//        fric_pwm2 = (uint16_t)(trigger_motor.fric2_ramp.out);

//        shoot_fric1_on(fric_pwm1);
//        shoot_fric2_on(fric_pwm2);

        //计算拨弹轮电机PID
				motor_pid[6].target=trigger_motor.speed_set;
				motor_pid[6].f_cal_pid(&motor_pid[6],trigger_motor.speed);
        //PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);

        trigger_motor.given_current = (int16_t)(motor_pid[6].output);
        shoot_CAN_Set_Current = trigger_motor.given_current;
    }

    return shoot_CAN_Set_Current;
}
/**
  * @brief          射击数据更新
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //拨弹轮电机速度滤波一下
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //二阶低通滤波
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (moto_chassis[6].speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;

    //电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
//    if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
//    {
//        trigger_motor.ecd_count--;
//    }
//    else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
//    {
//        trigger_motor.ecd_count++;
//    }
			trigger_motor.ecd_count=moto_chassis[6].round_cnt;
    if (trigger_motor.ecd_count == FULL_COUNT)
    {
        trigger_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (trigger_motor.ecd_count == -FULL_COUNT)
    {
        trigger_motor.ecd_count = FULL_COUNT - 1;
    }

    //计算输出轴角度
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + moto_chassis[6].angle ) * Motor_ECD_TO_ANGLE;
    //微动开关
    trigger_motor.key = Butten_Trig_Pin;
    //鼠标按键
    trigger_motor.last_press_l = trigger_motor.press_l;
    //trigger_motor.last_press_r = trigger_motor.press_r;
    trigger_motor.press_l = remote_control.mouse.press_left;
    //trigger_motor.press_r = remote_control.mouse.press_right;
    //长按计时
    if (trigger_motor.press_l)
    {
        if (trigger_motor.press_l_time < PRESS_LONG_TIME)
        {
            trigger_motor.press_l_time++;
        }
    }
    else
    {
        trigger_motor.press_l_time = 0;
    }

    if (trigger_motor.press_r)
    {
        if (trigger_motor.press_r_time < PRESS_LONG_TIME)
        {
            trigger_motor.press_r_time++;
        }
    }
    else
    {
        trigger_motor.press_r_time = 0;
    }

    //射击开关下档时间计时
    if (shoot_mode != SHOOT_STOP && remote_control.switch_right==2)
    {

        if (trigger_motor.rc_s_time < RC_S_LONG_TIME)
        {
            trigger_motor.rc_s_time++;
        }
    }
    else
    {
        trigger_motor.rc_s_time = 0;
    }
}
/**
  * @brief          射击状态机设置，遥控器上拨一次开启，再上拨关闭，下拨1次发射1颗，一直处在下，则持续发射，用于3min准备时间清理子弹
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //上拨判断， 一次开启，再次关闭
    if ((switch_is_up(remote_control.switch_right) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
				robot_status.fric_mode=MID_FIRE;//默认摩擦轮速度为中速 15m/s左右
        shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(remote_control.switch_right) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP) || (remote_control.keyBoard.key_code & SHOOT_OFF_KEYBOARD))
    {
        shoot_mode = SHOOT_STOP;
			#ifdef DEBUG_MODE//如果在调试模式，则可以停止摩擦轮，否则不允许关闭摩擦轮，避免卡弹导致摩擦轮无法启动
			robot_status.fric_mode=STOP;
			#endif
    }

    //处于中档， 可以使用键盘开启拨弹轮 摩擦轮
    if (switch_is_mid(remote_control.switch_right) && (remote_control.keyBoard.key_code & SHOOT_ON_KEYBOARD) && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY;
    }
    //处于中档， 可以使用键盘关闭拨弹轮 摩擦轮
    else if (switch_is_mid(remote_control.switch_right) && (remote_control.keyBoard.key_code & SHOOT_OFF_KEYBOARD) && shoot_mode == SHOOT_READY)
    {
        shoot_mode = SHOOT_STOP;
    }

    //如果云台状态是 无力状态，就关闭射击
//    if (gimbal_cmd_to_shoot_stop())
//    {
//        shoot_mode = SHOOT_STOP;
//    }

    if (shoot_mode == SHOOT_READY)
    {
        //下拨一次或者鼠标按下一次，进入射击状态
        if ((switch_is_down(remote_control.switch_right) && !switch_is_down(last_s)) || 
						(trigger_motor.press_l && trigger_motor.last_press_l == 0) || 
						(trigger_motor.press_r && trigger_motor.last_press_r == 0))
        {
            shoot_mode = SHOOT_BULLET;
            trigger_motor.last_butter_count = trigger_motor.BulletShootCnt;
        }
        //鼠标长按一直进入射击状态 保持连发
        if ((trigger_motor.press_l_time == PRESS_LONG_TIME) || 
						(trigger_motor.press_r_time == PRESS_LONG_TIME) || 
						(trigger_motor.rc_s_time == RC_S_LONG_TIME))
        {
            if (shoot_mode != SHOOT_DONE && trigger_motor.key == SWITCH_TRIGGER_ON)
            {
                shoot_mode = SHOOT_BULLET;
            }
        }
    }

    last_s = remote_control.switch_right;
}
/**
  * @brief          射击控制，控制拨弹电机角度，完成一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)//进入此程序的状态应该是子弹压在限位开关上，即将发射状态，即trigger_motor.key=1
{
    //子弹射出判断
    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
    {
        trigger_motor.shoot_done = 1;
        trigger_motor.shoot_done_time = 0;

        shoot_mode = SHOOT_DONE;//发射完成状态
        trigger_motor.set_angle = trigger_motor.angle;
    }

    //每次拨动 1/10PI的角度
    if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_BULLET)
    {
        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Ten);
        trigger_motor.cmd_time = HAL_GetTick();
        trigger_motor.move_flag = 1;
    }

    //到达角度判断
    if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
    {
        //没到达一直设置旋转速度
        trigger_motor.speed_set = TRIGGER_SPEED;
        trigger_motor.run_time = HAL_GetTick();

        //堵转判断,堵转原因是还没转到目标角度就停下来了才会触发，
        if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && 
						trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && 
						fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
        {
            trigger_motor.speed_set = -TRIGGER_SPEED;
        }
        else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || 
								fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)//
        {
            trigger_motor.cmd_time = HAL_GetTick();
        }
    }
    else//到达角度后再进入设置目标角度，让其一直旋转
    {
        trigger_motor.move_flag = 0;
    }
}
/**
  * @brief          射击完成控制，判断微动开关一段时间无子弹来判断一次发射
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void)
{
    trigger_motor.speed_set = 0.0f;
    //射击完成判断，判断微动开关一段时间无子弹
    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
    {
        if (trigger_motor.shoot_done_time < SHOOT_DONE_KEY_OFF_TIME)
        {
            trigger_motor.shoot_done_time++;
        }
        else if (trigger_motor.shoot_done_time == SHOOT_DONE_KEY_OFF_TIME)
        {
            trigger_motor.BulletShootCnt++;//完成一次发射，子弹加一
					shoot_mode = SHOOT_READY;//发射完成后需要再次将子弹送入限位开关上，以供下次快速发射
        }
    }
    else
    {
        shoot_mode = SHOOT_BULLET;
    }
}
/**
  * @brief          射击准备控制，将子弹送到微动开关处，
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void)
{

    if (trigger_motor.shoot_done)
    {
        trigger_motor.shoot_done = 0;
    }

    if (trigger_motor.key == SWITCH_TRIGGER_ON)//子弹到达微动开关，电机停止输出
    {
        //判断子弹到达微动开关处
        trigger_motor.set_angle = trigger_motor.angle;
        motor_pid[6].output = 0;
        //trigger_motor_pid.Iout = 0.0f;

        trigger_motor.speed_set = 0.0f;
        trigger_motor.move_flag = 0;
        trigger_motor.key_time = 0;
    }
    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time < KEY_OFF_JUGUE_TIME)
    {
        //判断无子弹一段时间
        trigger_motor.key_time++;
    }
    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time == KEY_OFF_JUGUE_TIME)
    {
        //微动开关一段时间没有子弹，进入拨弹，一次旋转 1/10PI的角度
        if (trigger_motor.move_flag == 0)
        {
            trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Ten);
            trigger_motor.cmd_time = HAL_GetTick();
            trigger_motor.move_flag = 1;
        }

        if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)//未到达角度
        {
            //角度达到判断
            trigger_motor.speed_set = Ready_Trigger_Speed;
            trigger_motor.run_time = HAL_GetTick();
            //堵转判断
            if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
            {
                trigger_motor.speed_set = -Ready_Trigger_Speed;
            }
            else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
            {
                trigger_motor.cmd_time = HAL_GetTick();
            }
        }
        else//到达角度，进入目标角度设置程序
        {
            trigger_motor.move_flag = 0;
        }
    }
}


///**********************************************************************************************************
//*函 数 名: single_shot
//*功能说明: 拨弹轮单发
//*形    参: 
//*返 回 值: 电流输出
//**********************************************************************************************************/
//int pwm_output;

//void single_shoot()
//{
//	if(remote_control.switch_left!=3)
//	{
//		if(remote_control.switch_right==1)
//		{
//			HeadTxData[4]=0;
//			HeadTxData[5]=0;
//			init_Fric_PWM();
//			fric_ramp.out=1000;
//		}
//		if(remote_control.switch_right==2)
//		{
//			moto_chassis[6].round_cnt=0;
//			HeadTxData[4]=0;
//			HeadTxData[5]=0;
//			close_fire();
//		}
//		if(remote_control.switch_right==3)
//		{
//			if(moto_chassis[6].round_cnt<1)
//			{
//					motor_pid[6].target=2500;
//					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
//					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
//					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
//			}
//			else
//			{
//					HeadTxData[4]=0;
//					HeadTxData[5]=0;
//			}
//			close_fire();
//		}
//	}
//}



//char BD_zhuangtai=0;
//char BD_tuidan=0;
//int i,keep;
//int pwm_output;
//void shoot_control()
//{
//	if(remote_control.switch_left!=3)
//		{
//				
//				
////					motor_pid[6].target=3000;
////					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
////					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
////					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
//					single_shoot();
//					
//					if(motor_pid[6].err<500&&moto_chassis[6].speed_rpm>600)		//拨弹轮工作状态稳定
//					{
//						BD_zhuangtai=1;
//					}
//					if((BD_zhuangtai==1)&&(motor_pid[6].err>400)&&(moto_chassis[6].speed_rpm<600))//拨弹轮由工作稳定到卡弹
//					{
//						BD_zhuangtai=2;
//						BD_tuidan=1;
//					}
//					if(BD_tuidan==1)
//					{
//						BD_tuidan=2;
//						moto_chassis[6].round_cnt=0;
//					}
//					if(BD_tuidan==2)
//					{
//						if(moto_chassis[6].round_cnt>-1)
//						{
//							motor_pid[6].target=-2500;
//							motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
//							HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
//					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
//						}
//						else
//						{
//							BD_tuidan=0;
//							BD_zhuangtai=3;
//						}
//					}
//					if(BD_zhuangtai==3)
//					{
//						keep++;
//						if(keep>500)
//							{BD_zhuangtai=0;keep=0;}
//					}
//					
//					
//					PWM_SetDuty(&htim5,TIM_CHANNEL_1,1800);
//					PWM_SetDuty(&htim5,TIM_CHANNEL_2,1800);
//					PWM_SetDuty(&htim5,TIM_CHANNEL_3,1800);
//					PWM_SetDuty(&htim5,TIM_CHANNEL_4,1800);
//				}
//				if(remote_control.switch_right==1)
//				{
//					HeadTxData[4]=0;
//					HeadTxData[5]=0;//拨弹轮电流值
//					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
//					//set_rammer_current(&hcan1,0);
//					init_Fric_PWM();
//				}

//		if(remote_control.switch_left==3)
//		{
//			HeadTxData[4]=0;
//			HeadTxData[5]=0;//拨弹轮电流值
//			//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
//			//set_rammer_current(&hcan1,0);
//			init_Fric_PWM();
//		}
//}


////单发程序
//void single_shoot1(void)
//{
//	int s=0;
//	if(remote_control.switch_left!=1)
//	{
//		ramp_calc(&fric_ramp,1200);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_1,fric_ramp.out);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_2,fric_ramp.out);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_3,fric_ramp.out);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_4,fric_ramp.out);
////			PWM_SetDuty(&htim5,TIM_CHANNEL_1,1.2);
////			PWM_SetDuty(&htim5,TIM_CHANNEL_2,1.2);
////			PWM_SetDuty(&htim5,TIM_CHANNEL_3,1.2);
////			PWM_SetDuty(&htim5,TIM_CHANNEL_4,1.2);
//				
//		if(remote_control.switch_right==2)//
//		{	
//			motor_pid[6].target=2200;
//			s=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
//			if(k&&(!s))
//				motor_pid[6].target=0;
//			j=0;
//		}
//		
//		else 
//			if(remote_control.switch_right==1)//
//			{
//				motor_pid[6].target=2200;
//				s=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
//				if(j&&!s)
//				motor_pid[6].target=0;
//			}
//		
//		if(remote_control.switch_right==3)//
//		{	
//				motor_pid[6].target=0;
//				moto_chassis[6].round_cnt=0;
//				j=0;
//				k=0;
//		}
//	}
//	
//	else
//	{
//		ramp_calc(&fric_ramp,900);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_1,fric_ramp.out);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_2,fric_ramp.out);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_3,fric_ramp.out);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_4,fric_ramp.out);
////		PWM_SetDuty(&htim5,TIM_CHANNEL_1,1.0);
////		PWM_SetDuty(&htim5,TIM_CHANNEL_2,1.0);
////		PWM_SetDuty(&htim5,TIM_CHANNEL_3,1.0);
////		PWM_SetDuty(&htim5,TIM_CHANNEL_4,1.0);
//	}
//	
//		motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
//		//motor_pid[7].f_cal_pid(&motor_pid[7],moto_chassis[7].speed_rpm);
//		HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
//		HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF);
//		//HeadTxData[6]=(uint8_t)((motor_pid[7].output>>8)&0xFF);//拨弹电机电流值
//		//HeadTxData[7]=(uint8_t)(motor_pid[7].output&0xFF);
//		CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
//}

//void shoot(char num)
//{
//	
//}

