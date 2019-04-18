#include "robomaster_chassis.h"
#include "arm_math.h"


#define CHASSIS_ACCEL_X_NUM 0.6f
#define CHASSIS_ACCEL_Y_NUM 0.6f
#define CHASSIS_KEYBOARD_SPEED 1.6f
#define CHASSIS_KEYBOARD_FAST_SPEED 2.8f
//底盘行为状态机
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
//底盘运动数据
chassis_move_t chassis_move;

void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    //遥控器设置行为模式
	if(robot_status.control_mode==REMOTE_CONTROL)//键盘模式
	{
    if (remote_control.switch_left==2)//底盘模式
    {
        chassis_behaviour_mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
    }
    else if (remote_control.switch_left==3)//杀死底盘
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
    else if (remote_control.switch_left==1)//底盘跟随云台
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
	}
	else if(robot_status.control_mode==KEYBOARD_CONTROL)//键盘模式
	{
		if(key_board_mode==0)//底盘跟随云台
		{
			chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
		}
		else if((key_board_mode==1))////底盘模式
		{
			chassis_behaviour_mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
		}
	}

    //云台进入某些状态的时候，底盘保持不动
//    if (gimbal_cmd_to_chassis_stop())
//    {
//        chassis_behaviour_mode = CHASSIS_NO_MOVE;
//    }

    //根据行为状态机选择底盘状态机
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //当行为是底盘无力，则设置底盘状态机为 raw，原生状态机。
    }
   else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)//3
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不移动，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)//1
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //当行为是正常步兵跟随云台，则设置底盘状态机为 底盘跟随云台角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)//2
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW; //当行为是工程跟随底盘角度，则设置底盘状态机为 底盘跟随底盘角度 状态机。
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {

        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不跟随角度，则设置底盘状态机为 底盘不跟随角度 状态机。
    }
//    else if (chassis_behaviour_mode == CHASSIS_OPEN)
//    {

//        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //当行为是底盘开环，则设置底盘状态机为 底盘原生raw 状态机。
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
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
        //更新电机速度，加速度是速度的PID微分
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * moto_chassis[i].speed_rpm;
        chassis_move_update->motor_chassis[i].accel = (motor_pid[i].err-motor_pid[i].last_err) * CHASSIS_CONTROL_FREQUENCE;
    }
		chassis_move.chassis_relative_pit_angle=(CHASSIS_PIT_MID_VALUE-pan_tilt_pitch_motor.angle)/8192.0f*360.0f*angle_to_radian;//底盘相对于云台pit的角度
		chassis_move.chassis_relative_angle=(CHASSIS_YAW_MID_VALUE-pan_tilt_yaw_motor.angle)/8192.0f*360.0f*angle_to_radian;//底盘相对于云台yaw的角度
    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系  前进为正 左平移为正 逆时针旋转为正
    chassis_move_update->vx = (chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;

    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码  底盘角度范围为(-pi,pi)
    chassis_move_update->chassis_yaw = rad_format(((imu.yaw-180.0f)*angle_to_radian-chassis_move.chassis_relative_angle));			//根据云台上的陀螺仪获取底盘相对于地面的yaw,pitch,roll
    chassis_move_update->chassis_pitch = rad_format(((imu.pit-180.0f)*angle_to_radian - chassis_move.chassis_relative_pit_angle));
    chassis_move_update->chassis_roll = (imu.rol-180.0f)*angle_to_radian;
}
void chassis_init(chassis_move_t *chassis_move_init)
{
    //底盘旋转环pid值
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //最大 最小速度
    chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}

/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //摇摆角度是利用sin函数生成，swing_time 是sin函数的输入值
    static fp32 swing_time = 0.0f;
    //swing_angle 是计算出来的角度
    static fp32 swing_angle = 0.0f;
    //max_angle 是sin函数的幅值
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //add_time 是摇摆角度改变的快慢，最大越快
     fp32  add_time = PI / Twist_speed;
    //使能摇摆标志位
    static uint8_t swing_flag = 0;

    //计算遥控器的原始输入信号

    //判断是否要摇摆
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

    //判断键盘输入是不是在控制底盘运动，底盘在运动减小摇摆角度
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
    //sin函数生成控制角度
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //sin函数不超过2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }

    *angle_set = swing_angle;
}


//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(remote_control.ch2, vx_channel, CHASSIS_RC_DEADLINE);//直行
    rc_deadline_limit(remote_control.ch1, vy_channel, CHASSIS_RC_DEADLINE);//平移

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
    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_cmd_slow_set_vy, vy_set_channel);
		
    //停止信号，不需要缓慢加速，直接减速到零
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
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set + CHASSIS_ANGLE_Z_RC_SEN * remote_control.ch3);//（0，2pi）(-0.001,0.01)
}
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度
  * @param[in]      vy_set左右的速度
  * @param[in]      wz_set底盘设置的旋转速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * remote_control.ch3;//遥控器通道三控制底盘旋转速度(-66,66)
}

//设置遥控器输入控制量
void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

    if (chassis_move_control == NULL)
    {
        return;
    }

    //设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
    chassis_behaviour_control_set(&vx_set, &vy_set, &angle_set, chassis_move_control);

    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
        //旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
        sin_yaw = arm_sin_f32(chassis_move.chassis_relative_angle);//底盘相对于云台的角度 rad/s 弧度制
        cos_yaw = arm_cos_f32(chassis_move.chassis_relative_angle);
			
//				oled_shownum(0,0,int_abs(chassis_move.chassis_relative_angle)*100,0,3);
//				oled_shownum(1,0,int_abs(chassis_move_control->wz_set)*100,0,3);
			
        chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;//前进方向始终相对于云台
        chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
        //设置控制相对云台角度
        chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
				chassis_angle_pid.target=chassis_move_control->chassis_relative_angle_set;
        //计算旋转PID角速度
        chassis_move_control->wz_set = -chassis_angle_pid.f_cal_pid(&chassis_angle_pid, chassis_move.chassis_relative_angle);//反馈值为底盘相对云台yaw的角度
        //速度限幅
        chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        fp32 delat_angle = 0.0f;
        //放弃跟随云台
        //设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(angle_set);
        delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
				chassis_angle_pid.target=delat_angle;
        //计算旋转的角速度  只有当目标值为0时，角速度才为0
        chassis_move_control->wz_set = chassis_angle_pid.f_cal_pid(&chassis_angle_pid, 0.0f);
        //设置底盘运动的速度
        chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {

        //放弃跟随云台
        //这个模式下，角度设置的为 角速度
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
    //旋转的时候， 由于云台靠后，所以是前面两轮 0 ，1 旋转的速度变快， 后面两轮 2,3 旋转的速度变慢
    wheel_speed[0] = vx_set - vy_set - (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = -vx_set - vy_set - (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = -vx_set + vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = +vx_set + vy_set - (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}


void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
		int16_t output[4]={0};//用于超级电容速度限制
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t i = 0;
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);
//		oled_shownum(1,4,int_abs(chassis_move_control_loop->vx_set)*100,0,3);
//		oled_shownum(1,8,int_abs(chassis_move_control_loop->vy_set)*100,0,3);


    //计算轮子控制最大速度，并限制其最大速度
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

    //计算pid

    for (i = 0; i < 4; i++)
    {
        motor_pid[i].f_cal_pid(&motor_pid[i], chassis_move_control_loop->motor_chassis[i].speed);
    }
		//oled_shownum(4,0,int_abs(motor_pid[1].output),0,5);
    //赋值电流值
//    for (i = 0; i < 4; i++)
//    {
//        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(motor_pid[i].output);
//    }
		
		
			output[0]=(motor_pid[0].output);
			output[1]=(motor_pid[1].output);
			output[2]=(motor_pid[2].output);
			output[3]=(motor_pid[3].output);
			Super_Cap_control(output);//电容闭环
			
			chassis_current_mix(output);
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
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

    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)//1
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)//3
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //切入不跟随云台模式
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
*函 数 名: chassis_task
*功能说明: 底盘控制任务
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void chassis_task()
{
	//遥控器设置状态
  chassis_set_mode(&chassis_move);
  //遥控器状态切换数据保存
  chassis_mode_change_control_transit(&chassis_move);
  //底盘数据更新
  chassis_feedback_update(&chassis_move);
  //底盘控制量设置
  chassis_set_contorl(&chassis_move);
  //底盘控制PID计算
  chassis_control_loop(&chassis_move);
}
/**********************************************************************************************************
*函 数 名: chassis_current_mix
*功能说明: 底盘电流输出融合
*形    参: 需要速度环电流，位置换电流，功率环电流
*返 回 值: 电流输出
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





