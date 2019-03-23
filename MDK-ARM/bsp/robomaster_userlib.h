#ifndef __ROBOMASTER_USERLIB
#define __ROBOMASTER_USERLIB
#include "robomaster_common.h"


typedef __packed struct
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

extern ramp_function_source_t fric_ramp;

#endif
