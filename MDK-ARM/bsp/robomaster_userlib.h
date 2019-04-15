#ifndef ROBOMASTER_USERLIB_H
#define ROBOMASTER_USERLIB_H
#include "robomaster_common.h"
#define N                  4         //滑动平均滤波参数
typedef __packed struct
{
    fp32 input;        //输入数据
    fp32 out;          //滤波输出的数据
    fp32 num[1];       //滤波参数
    fp32 frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;
typedef struct
{
    fp32 input;        //输入数据
    fp32 out;          //输出数据
    fp32 min_value;    //限幅最小值
    fp32 max_value;    //限幅最大值
    fp32 frame_period; //时间间隔
} ramp_function_source_t;
//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
extern float Data[N];
extern first_order_filter_type_t chassis_cmd_slow_set_vx;
extern first_order_filter_type_t chassis_cmd_slow_set_vy;
extern first_order_filter_type_t pan_tilt_cmd_slow_set_yaw;
extern first_order_filter_type_t pan_tilt_cmd_slow_set_pit;
extern ramp_function_source_t fric_ramp;
//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//循环限幅函数
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
float GildeAverageValueFilter(float NewValue,float *Data);
//弧度格式化为-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
