#ifndef __ROBOMASTER_USERLIB
#define __ROBOMASTER_USERLIB
#include "robomaster_common.h"


typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�������
    fp32 min_value;    //�޷���Сֵ
    fp32 max_value;    //�޷����ֵ
    fp32 frame_period; //ʱ����
} ramp_function_source_t;
//б��������ʼ��
void ramp_init(ramp_function_source_t *ramp_source_type, fp32 frame_period, fp32 max, fp32 min);

//б����������
void ramp_calc(ramp_function_source_t *ramp_source_type, fp32 input);

int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

extern ramp_function_source_t fric_ramp;

#endif
