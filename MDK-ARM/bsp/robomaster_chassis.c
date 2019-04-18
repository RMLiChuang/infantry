#include "robomaster_chassis.h"
#include "arm_math.h"


#define CHASSIS_ACCEL_X_NUM 0.6f
#define CHASSIS_ACCEL_Y_NUM 0.6f
#define CHASSIS_KEYBOARD_SPEED 1.6f
#define CHASSIS_KEYBOARD_FAST_SPEED 2.8f
//������Ϊ״̬��
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
//�����˶�����
chassis_move_t chassis_move;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //ң����������Ϊģʽ
	if(robot_status.control_mode==REMOTE_CONTROL)//����ģʽ
	{
    if (remote_control.switch_left==2)//����ģʽ
    {
        chassis_behaviour_mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
    }
    else if (remote_control.switch_left==3)//ɱ������
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
    else if (remote_control.switch_left==1)//���̸�����̨
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
	}
	else if(robot_status.control_mode==KEYBOARD_CONTROL)//����ģʽ
	{
		if(key_board_mode==0)//���̸�����̨
		{
			chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
		}
		else if((key_board_mode==1))////����ģʽ
		{
			chassis_behaviour_mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
		}
	}

    //��̨����ĳЩ״̬��ʱ�򣬵��̱��ֲ���
//    if (gimbal_cmd_to_chassis_stop())
//    {
//        chassis_behaviour_mode = CHASSIS_NO_MOVE;
//    }

    //������Ϊ״̬��ѡ�����״̬��
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //����Ϊ�ǵ��������������õ���״̬��Ϊ raw��ԭ��״̬����
    }
   else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)//3
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //����Ϊ�ǵ��̲��ƶ��������õ���״̬��Ϊ ���̲�����Ƕ� ״̬����
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)//1
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //����Ϊ����������������̨�������õ���״̬��Ϊ ���̸�����̨�Ƕ� ״̬����
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)//2
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW; //����Ϊ�ǹ��̸�����̽Ƕȣ������õ���״̬��Ϊ ���̸�����̽Ƕ� ״̬����
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //����Ϊ�ǵ��̲�����Ƕȣ������õ���״̬��Ϊ ���̲�����Ƕ� ״̬����
    }
//    else if (chassis_behaviour_mode == CHASSIS_OPEN)
//    {

//        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //����Ϊ�ǵ��̿����������õ���״̬��Ϊ ����ԭ��raw ״̬����
//    }
}
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)//3
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)//1
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)//2
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
//    else if (chassis_behaviour_mode == CHASSIS_OPEN)
//    {
//        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//    }
}

/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}
/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}
void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //���µ���ٶȣ����ٶ����ٶȵ�PID΢��
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * moto_chassis[i].speed_rpm;
        chassis_move_update->motor_chassis[i].accel = (motor_pid[i].err-motor_pid[i].last_err) * CHASSIS_CONTROL_FREQUENCE;
    }
		chassis_move.chassis_relative_pit_angle=(CHASSIS_PIT_MID_VALUE-pan_tilt_pitch_motor.angle)/8192.0f*360.0f*angle_to_radian;//�����������̨pit�ĽǶ�
		chassis_move.chassis_relative_angle=(CHASSIS_YAW_MID_VALUE-pan_tilt_yaw_motor.angle)/8192.0f*360.0f*angle_to_radian;//�����������̨yaw�ĽǶ�
    //���µ���ǰ���ٶ� x�� ƽ���ٶ�y����ת�ٶ�wz������ϵΪ����ϵ  ǰ��Ϊ�� ��ƽ��Ϊ�� ��ʱ����תΪ��
    chassis_move_update->vx = (chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //���������̬�Ƕ�, �����������������������ⲿ�ִ���  ���̽Ƕȷ�ΧΪ(-pi,pi)
    chassis_move_update->chassis_yaw = rad_format(((imu.yaw-180.0f)*angle_to_radian-chassis_move.chassis_relative_angle));			//������̨�ϵ������ǻ�ȡ��������ڵ����yaw,pitch,roll
    chassis_move_update->chassis_pitch = rad_format(((imu.pit-180.0f)*angle_to_radian - chassis_move.chassis_relative_pit_angle));
    chassis_move_update->chassis_roll = (imu.rol-180.0f)*angle_to_radian;
}
void chassis_init(chassis_move_t *chassis_move_init)
{
    //������ת��pidֵ
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    //��һ���˲�����б����������
    first_order_filter_init(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //��� ��С�ٶ�
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //����һ������
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //ҡ�ڽǶ�������sin�������ɣ�swing_time ��sin����������ֵ
    static fp32 swing_time = 0.0f;
    //swing_angle �Ǽ�������ĽǶ�
    static fp32 swing_angle = 0.0f;
    //max_angle ��sin�����ķ�ֵ
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //add_time ��ҡ�ڽǶȸı�Ŀ��������Խ��
     fp32  add_time = PI / Twist_speed;
    //ʹ��ҡ�ڱ�־λ
    static uint8_t swing_flag = 0;

    //����ң������ԭʼ�����ź�

    //�ж��Ƿ�Ҫҡ��
    if (remote_control.keyBoard.key_code & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }

    //�жϼ��������ǲ����ڿ��Ƶ����˶����������˶���Сҡ�ڽǶ�
    if (remote_control.keyBoard.key_code & CHASSIS_FRONT_KEY || remote_control.keyBoard.key_code & CHASSIS_BACK_KEY ||
        remote_control.keyBoard.key_code & CHASSIS_LEFT_KEY || remote_control.keyBoard.key_code & CHASSIS_RIGHT_KEY || 
				remote_control.ch1!=0 || remote_control.ch2!=0)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    //sin�������ɿ��ƽǶ�
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //sin����������2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }

    *angle_set = swing_angle;
}


//ң���������ݴ���ɵ��̵�ǰ��vx�ٶȣ�vy�ٶ�
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //ң����ԭʼͨ��ֵ
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //�������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
    rc_deadline_limit(remote_control.ch2, vx_channel, CHASSIS_RC_DEADLINE);//ֱ��
    rc_deadline_limit(remote_control.ch1, vy_channel, CHASSIS_RC_DEADLINE);//ƽ��

    vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;

      if (remote_control.keyBoard.key_code & CHASSIS_FRONT_KEY)
    {
        vx_set_channel = CHASSIS_KEYBOARD_SPEED;
    }
    else if (remote_control.keyBoard.key_code & CHASSIS_BACK_KEY)
    {
        vx_set_channel = -CHASSIS_KEYBOARD_SPEED;
    }

    if (remote_control.keyBoard.key_code & CHASSIS_LEFT_KEY)
    {
        vy_set_channel = CHASSIS_KEYBOARD_SPEED;
    }
    else if (remote_control.keyBoard.key_code & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = -CHASSIS_KEYBOARD_SPEED;
    }
		else if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_SHIFT)
		{
			if(vx_set_channel>0)
				vx_set_channel=CHASSIS_KEYBOARD_FAST_SPEED;
			else if(vx_set_channel<0)
				vx_set_channel=-CHASSIS_KEYBOARD_FAST_SPEED;
		}
    //һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_cmd_slow_set_vy, vy_set_channel);
		
    //ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_cmd_slow_set_vy.out = 0.0f;
    }

    *vx_set = chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_cmd_slow_set_vy.out;
}
/**
  * @brief          ���̸������yaw����Ϊ״̬���£�����ģʽ�Ǹ�����̽Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      angle_set�������õ�yaw����Χ -PI��PI
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set + CHASSIS_ANGLE_Z_RC_SEN * remote_control.ch3);//��0��2pi��(-0.001,0.01)
}
/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  * @param[in]      vy_set���ҵ��ٶ�
  * @param[in]      wz_set�������õ���ת�ٶ�
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * remote_control.ch3;//ң����ͨ�������Ƶ�����ת�ٶ�(-66,66)
}

//����ң�������������
void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    //�����ٶ�
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //������̨ģʽ
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
        sin_yaw = arm_sin_f32(chassis_move.chassis_relative_angle);//�����������̨�ĽǶ� rad/s ������
        cos_yaw = arm_cos_f32(chassis_move.chassis_relative_angle);
			
//				oled_shownum(0,0,int_abs(chassis_move.chassis_relative_angle)*100,0,3);
//				oled_shownum(1,0,int_abs(chassis_move_control->wz_set)*100,0,3);
			
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;//ǰ������ʼ���������̨
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //���ÿ��������̨�Ƕ�
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
				chassis_angle_pid.target=chassis_move_control->chassis_relative_angle_set;
        //������תPID���ٶ�
        chassis_move_control->wz_set = -chassis_angle_pid.f_cal_pid(&chassis_angle_pid, chassis_move.chassis_relative_angle);//����ֵΪ���������̨yaw�ĽǶ�
        //�ٶ��޷�
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //����������̨
        //���õ��̿��ƵĽǶ�
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
				chassis_angle_pid.target=delat_angle;
        //������ת�Ľ��ٶ�  ֻ�е�Ŀ��ֵΪ0ʱ�����ٶȲ�Ϊ0
        chassis_move_control->wz_set = chassis_angle_pid.f_cal_pid(&chassis_angle_pid, 0.0f);
        //���õ����˶����ٶ�
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

        //����������̨
        //���ģʽ�£��Ƕ����õ�Ϊ ���ٶ�
        fp32 chassis_wz = angle_set;
        chassis_move_control->wz_set = chassis_wz;
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
    {
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_cmd_slow_set_vy.out = 0.0f;
    }
}
void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
{
		//cal_twist_distribute();
    //��ת��ʱ�� ������̨����������ǰ������ 0 ��1 ��ת���ٶȱ�죬 �������� 2,3 ��ת���ٶȱ���
    wheel_speed[0] = vx_set - vy_set - (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = -vx_set - vy_set - (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = -vx_set + vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = +vx_set + vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}


void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
		int16_t output[4]={0};//���ڳ��������ٶ�����
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //�����˶��ֽ�
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
//		oled_shownum(1,4,int_abs(chassis_move_control_loop->vx_set)*100,0,3);
//		oled_shownum(1,8,int_abs(chassis_move_control_loop->vy_set)*100,0,3);


    //�������ӿ�������ٶȣ�������������ٶ�
    for (i = 0; i < 4; i++)
    {
        motor_pid[i].target = wheel_speed[i];
        temp = fabs(motor_pid[i].target);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            motor_pid[i].target *= vector_rate;
        }
    }

    //����pid

    for (i = 0; i < 4; i++)
    {
        motor_pid[i].f_cal_pid(&motor_pid[i], chassis_move_control_loop->motor_chassis[i].speed);
    }
		//oled_shownum(4,0,int_abs(motor_pid[1].output),0,5);
    //��ֵ����ֵ
//    for (i = 0; i < 4; i++)
//    {
//        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(motor_pid[i].output);
//    }
		
		
			output[0]=(motor_pid[0].output);
			output[1]=(motor_pid[1].output);
			output[2]=(motor_pid[2].output);
			output[3]=(motor_pid[3].output);
			Super_Cap_control(output);//���ݱջ�
			
			chassis_current_mix(output);
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //����̵�����͸����ĵ���ֵ
}

void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //���������̨ģʽ
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)//1
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //���������̽Ƕ�ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)//3
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //���벻������̨ģʽ
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)//2
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
}
/**********************************************************************************************************
*�� �� ��: chassis_task
*����˵��: ���̿�������
*��    ��: 
*�� �� ֵ: �������
**********************************************************************************************************/
void chassis_task()
{
	//ң��������״̬
  chassis_set_mode(&chassis_move);
  //ң����״̬�л����ݱ���
  chassis_mode_change_control_transit(&chassis_move);
  //�������ݸ���
  chassis_feedback_update(&chassis_move);
  //���̿���������
  chassis_set_contorl(&chassis_move);
  //���̿���PID����
  chassis_control_loop(&chassis_move);
}
/**********************************************************************************************************
*�� �� ��: chassis_current_mix
*����˵��: ���̵�������ں�
*��    ��: ��Ҫ�ٶȻ�������λ�û����������ʻ�����
*�� �� ֵ: �������
**********************************************************************************************************/
void chassis_current_mix(int16_t *output)
{
	MotorTxData[0] = output[0]>>8&0xFF;
	MotorTxData[1] = output[0]&0xFF;
	MotorTxData[2] = output[1]>>8&0xFF;
	MotorTxData[3] = output[1]&0xFF;
	MotorTxData[4] = output[2]>>8&0xFF;
	MotorTxData[5] = output[2]&0xFF;
	MotorTxData[6] = output[3]>>8&0xFF;
	MotorTxData[7] = output[3]&0xFF;
}





