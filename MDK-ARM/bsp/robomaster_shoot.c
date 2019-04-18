/**
  ******************************************************************************
  * File Name          : robomaster_shoot.c
	* author						 : �ܺ�
	* Date							 :2019.1.23
  * Description        :����������
	
  ******************************************************************************
  ******************************************************************************
  */

#include "robomaster_shoot.h"

Shoot_Motor_t trigger_motor;          //�������
shoot_mode_e shoot_mode = SHOOT_STOP; //���״̬��
//΢������IO
#define Butten_Trig_Pin HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)
//�����ٶ�
#define TRIGGER_SPEED 11.0f
#define Ready_Trigger_Speed 7.0f

#define KEY_OFF_JUGUE_TIME 200
//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME 1
#define SWITCH_TRIGGER_ON 1
#define SWITCH_TRIGGER_OFF 0
/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void);
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void);
/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);
/**
  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void);
/**
  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_ready_control(void);
/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{
    //��������
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
  * @brief          ���ѭ��
  * @author         RM
  * @param[in]      void
  * @retval         ����can����ֵ
  */
int16_t shoot_control_loop(void)
{
    int16_t shoot_CAN_Set_Current; //���ص�canֵ
//		oled_shownum(3,4, trigger_motor.BulletShootCnt,0,3);
    Shoot_Set_Mode();        //����״̬��
    Shoot_Feedback_Update(); //��������

    //����״̬����
    if (shoot_mode == SHOOT_BULLET)
    {
//			trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
//			trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    //�������״̬����
    else if (shoot_mode == SHOOT_DONE)
    {
        shoot_done_control();
    }
    //����׼��״̬����
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
//        //Ħ����pwm
//        static uint16_t fric_pwm1 = Fric_OFF;
//        static uint16_t fric_pwm2 = Fric_OFF;


//        //shoot_laser_on();       //���⿪��


//        //Ħ������Ҫһ����б������������ͬʱֱ�ӿ�����������ܵ����ת
//        ramp_calc(&trigger_motor.fric1_ramp, SHOOT_FRIC_PWM_ADD_VALUE);

//        if(trigger_motor.fric1_ramp.out == trigger_motor.fric1_ramp.max_value)
//        {
//            ramp_calc(&trigger_motor.fric2_ramp, SHOOT_FRIC_PWM_ADD_VALUE);
//        }

//        if( trigger_motor.fric2_ramp.out != trigger_motor.fric2_ramp.max_value)
//        {
//            trigger_motor.speed_set = 0.0f;
//        }


////����Ҽ����¼���Ħ���֣�ʹ�������������� �Ҽ��������
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

        //���㲦���ֵ��PID
				motor_pid[6].target=trigger_motor.speed_set;
				motor_pid[6].f_cal_pid(&motor_pid[6],trigger_motor.speed);
        //PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);

        trigger_motor.given_current = (int16_t)(motor_pid[6].output);
        shoot_CAN_Set_Current = trigger_motor.given_current;
    }

    return shoot_CAN_Set_Current;
}
/**
  * @brief          ������ݸ���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Feedback_Update(void)
{

    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;

    //�����ֵ���ٶ��˲�һ��
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //���׵�ͨ�˲�
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (moto_chassis[6].speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;

    //���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
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

    //���������Ƕ�
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + moto_chassis[6].angle ) * Motor_ECD_TO_ANGLE;
    //΢������
    trigger_motor.key = Butten_Trig_Pin;
    //��갴��
    trigger_motor.last_press_l = trigger_motor.press_l;
    //trigger_motor.last_press_r = trigger_motor.press_r;
    trigger_motor.press_l = remote_control.mouse.press_left;
    //trigger_motor.press_r = remote_control.mouse.press_right;
    //������ʱ
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

    //��������µ�ʱ���ʱ
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
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void Shoot_Set_Mode(void)
{
    static int8_t last_s = RC_SW_UP;

    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(remote_control.switch_right) && !switch_is_up(last_s) && shoot_mode == SHOOT_STOP))
    {
				robot_status.fric_mode=MID_FIRE;//Ĭ��Ħ�����ٶ�Ϊ���� 15m/s����
        shoot_mode = SHOOT_READY;
    }
    else if ((switch_is_up(remote_control.switch_right) && !switch_is_up(last_s) && shoot_mode != SHOOT_STOP) || (remote_control.keyBoard.key_code & SHOOT_OFF_KEYBOARD))
    {
        shoot_mode = SHOOT_STOP;
			#ifdef DEBUG_MODE//����ڵ���ģʽ�������ֹͣĦ���֣���������ر�Ħ���֣����⿨������Ħ�����޷�����
			robot_status.fric_mode=STOP;
			#endif
    }

    //�����е��� ����ʹ�ü��̿��������� Ħ����
    if (switch_is_mid(remote_control.switch_right) && (remote_control.keyBoard.key_code & SHOOT_ON_KEYBOARD) && shoot_mode == SHOOT_STOP)
    {
        shoot_mode = SHOOT_READY;
    }
    //�����е��� ����ʹ�ü��̹رղ����� Ħ����
    else if (switch_is_mid(remote_control.switch_right) && (remote_control.keyBoard.key_code & SHOOT_OFF_KEYBOARD) && shoot_mode == SHOOT_READY)
    {
        shoot_mode = SHOOT_STOP;
    }

    //�����̨״̬�� ����״̬���͹ر����
//    if (gimbal_cmd_to_shoot_stop())
//    {
//        shoot_mode = SHOOT_STOP;
//    }

    if (shoot_mode == SHOOT_READY)
    {
        //�²�һ�λ�����갴��һ�Σ��������״̬
        if ((switch_is_down(remote_control.switch_right) && !switch_is_down(last_s)) || 
						(trigger_motor.press_l && trigger_motor.last_press_l == 0) || 
						(trigger_motor.press_r && trigger_motor.last_press_r == 0))
        {
            shoot_mode = SHOOT_BULLET;
            trigger_motor.last_butter_count = trigger_motor.BulletShootCnt;
        }
        //��곤��һֱ�������״̬ ��������
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
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)//����˳����״̬Ӧ�����ӵ�ѹ����λ�����ϣ���������״̬����trigger_motor.key=1
{
    //�ӵ�����ж�
    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
    {
        trigger_motor.shoot_done = 1;
        trigger_motor.shoot_done_time = 0;

        shoot_mode = SHOOT_DONE;//�������״̬
        trigger_motor.set_angle = trigger_motor.angle;
    }

    //ÿ�β��� 1/10PI�ĽǶ�
    if (trigger_motor.move_flag == 0 && shoot_mode == SHOOT_BULLET)
    {
        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Ten);
        trigger_motor.cmd_time = HAL_GetTick();
        trigger_motor.move_flag = 1;
    }

    //����Ƕ��ж�
    if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
    {
        //û����һֱ������ת�ٶ�
        trigger_motor.speed_set = TRIGGER_SPEED;
        trigger_motor.run_time = HAL_GetTick();

        //��ת�ж�,��תԭ���ǻ�ûת��Ŀ��ǶȾ�ͣ�����˲Żᴥ����
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
    else//����ǶȺ��ٽ�������Ŀ��Ƕȣ�����һֱ��ת
    {
        trigger_motor.move_flag = 0;
    }
}
/**
  * @brief          �����ɿ��ƣ��ж�΢������һ��ʱ�����ӵ����ж�һ�η���
  * @author         RM
  * @param[in]      void
  * @retval         void
  */
static void shoot_done_control(void)
{
    trigger_motor.speed_set = 0.0f;
    //�������жϣ��ж�΢������һ��ʱ�����ӵ�
    if (trigger_motor.key == SWITCH_TRIGGER_OFF)
    {
        if (trigger_motor.shoot_done_time < SHOOT_DONE_KEY_OFF_TIME)
        {
            trigger_motor.shoot_done_time++;
        }
        else if (trigger_motor.shoot_done_time == SHOOT_DONE_KEY_OFF_TIME)
        {
            trigger_motor.BulletShootCnt++;//���һ�η��䣬�ӵ���һ
					shoot_mode = SHOOT_READY;//������ɺ���Ҫ�ٴν��ӵ�������λ�����ϣ��Թ��´ο��ٷ���
        }
    }
    else
    {
        shoot_mode = SHOOT_BULLET;
    }
}
/**
  * @brief          ���׼�����ƣ����ӵ��͵�΢�����ش���
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

    if (trigger_motor.key == SWITCH_TRIGGER_ON)//�ӵ�����΢�����أ����ֹͣ���
    {
        //�ж��ӵ�����΢�����ش�
        trigger_motor.set_angle = trigger_motor.angle;
        motor_pid[6].output = 0;
        //trigger_motor_pid.Iout = 0.0f;

        trigger_motor.speed_set = 0.0f;
        trigger_motor.move_flag = 0;
        trigger_motor.key_time = 0;
    }
    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time < KEY_OFF_JUGUE_TIME)
    {
        //�ж����ӵ�һ��ʱ��
        trigger_motor.key_time++;
    }
    else if (trigger_motor.key == SWITCH_TRIGGER_OFF && trigger_motor.key_time == KEY_OFF_JUGUE_TIME)
    {
        //΢������һ��ʱ��û���ӵ������벦����һ����ת 1/10PI�ĽǶ�
        if (trigger_motor.move_flag == 0)
        {
            trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Ten);
            trigger_motor.cmd_time = HAL_GetTick();
            trigger_motor.move_flag = 1;
        }

        if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)//δ����Ƕ�
        {
            //�Ƕȴﵽ�ж�
            trigger_motor.speed_set = Ready_Trigger_Speed;
            trigger_motor.run_time = HAL_GetTick();
            //��ת�ж�
            if (trigger_motor.run_time - trigger_motor.cmd_time > BLOCK_TIME && trigger_motor.run_time - trigger_motor.cmd_time < REVERSE_TIME + BLOCK_TIME && fabs(trigger_motor.speed) < REVERSE_SPEED_LIMIT)
            {
                trigger_motor.speed_set = -Ready_Trigger_Speed;
            }
            else if (trigger_motor.run_time - trigger_motor.cmd_time > REVERSE_TIME + BLOCK_TIME || fabs(trigger_motor.speed) > REVERSE_SPEED_LIMIT)
            {
                trigger_motor.cmd_time = HAL_GetTick();
            }
        }
        else//����Ƕȣ�����Ŀ��Ƕ����ó���
        {
            trigger_motor.move_flag = 0;
        }
    }
}


///**********************************************************************************************************
//*�� �� ��: single_shot
//*����˵��: �����ֵ���
//*��    ��: 
//*�� �� ֵ: �������
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
//					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//�����������ֵ
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
////					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//�����������ֵ
////					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
//					single_shoot();
//					
//					if(motor_pid[6].err<500&&moto_chassis[6].speed_rpm>600)		//�����ֹ���״̬�ȶ�
//					{
//						BD_zhuangtai=1;
//					}
//					if((BD_zhuangtai==1)&&(motor_pid[6].err>400)&&(moto_chassis[6].speed_rpm<600))//�������ɹ����ȶ�������
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
//							HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//�����������ֵ
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
//					HeadTxData[5]=0;//�����ֵ���ֵ
//					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
//					//set_rammer_current(&hcan1,0);
//					init_Fric_PWM();
//				}

//		if(remote_control.switch_left==3)
//		{
//			HeadTxData[4]=0;
//			HeadTxData[5]=0;//�����ֵ���ֵ
//			//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
//			//set_rammer_current(&hcan1,0);
//			init_Fric_PWM();
//		}
//}


////��������
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
//		HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//�����������ֵ
//		HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF);
//		//HeadTxData[6]=(uint8_t)((motor_pid[7].output>>8)&0xFF);//�����������ֵ
//		//HeadTxData[7]=(uint8_t)(motor_pid[7].output&0xFF);
//		CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
//}

//void shoot(char num)
//{
//	
//}

