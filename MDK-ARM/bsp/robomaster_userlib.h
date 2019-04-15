#ifndef ROBOMASTER_USERLIB_H
#define ROBOMASTER_USERLIB_H
#include "robomaster_common.h"
#define N                  4         //����ƽ���˲�����
typedef __packed struct
{
    fp32 input;        //��������
    fp32 out;          //�˲����������
    fp32 num[1];       //�˲�����
    fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;
typedef struct
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
extern float Data[N];
extern first_order_filter_type_t chassis_cmd_slow_set_vx;
extern first_order_filter_type_t chassis_cmd_slow_set_vy;
extern first_order_filter_type_t pan_tilt_cmd_slow_set_yaw;
extern first_order_filter_type_t pan_tilt_cmd_slow_set_pit;
extern ramp_function_source_t fric_ramp;
//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num[1]);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//ѭ���޷�����
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
float GildeAverageValueFilter(float NewValue,float *Data);
//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
