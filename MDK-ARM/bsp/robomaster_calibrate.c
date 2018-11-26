/**
  *@file robomaster_calibrate.c
  *@date 2018-11-24
  *@author izh20
  *@brief ���ļ���ȫ�Ǵ������Ľ�׼����
  */



#include "robomaster_calibrate.h"


/**********************************************************************************************************
*�� �� ��: 
*����˵��: 
*��    ��: 
*�� �� ֵ: 
**********************************************************************************************************/
//�������ϵ��Զ�У׼
void Calibrate_mag()
{
//	char Cal_num=0;
//	if(Cal_num<100)
//	{
//		if(hx>Hx_Max)Hx_Max=hx;//��¼��ֵ
//		if(hx<Hx_Min)Hx_Min=hx;
//		if(hy>Hy_Max)Hy_Max=hy;
//		if(hy<Hy_Min)Hy_Min=hy;
//		Cal_num++;
//	}
//	else
//	{
//		dX=(float)(Hx_Max+Hx_Min)/2;//��ƫ
//		dY=(float)(Hy_Max+Hy_Min);
//		AX=(float)1000/(Hx_Max-Hx_Min);//��һ������
//		AY=(float)1000/(Hy_Max-Hy_Min);
//	}
	
}





/**********************************************************************************************************
*�� �� ��: AccCalibration
*����˵��: ���ٶ�У׼
*��    ��: ���ٶ�ԭʼ����
*�� �� ֵ: ��
**********************************************************************************************************/

//void AccCalibration(Vector3f_t accRaw)
//{
//    static uint16_t samples_count = 0;
//    static uint8_t orientationCaliFlag[6];
//    static uint8_t currentOrientation;
//    static Vector3f_t new_offset; 
//    static Vector3f_t new_scale;
//    static Vector3f_t samples[6];
//    static uint8_t caliFlag = 0;
//    static uint32_t caliCnt = 0;

//    if(!acc.cali.should_cali)
//        return;

//    /*********************************���IMU���÷���************************************/
//    if(GetImuOrientation() == ORIENTATION_UP && !orientationCaliFlag[ORIENTATION_UP])
//    {
//        //�ж�IMU�Ƿ��ھ�ֹ״̬
//        if(GetPlaceStatus() == STATIC)
//            caliCnt++;
//        else
//            caliCnt = 0;

//        if(caliCnt > 1000)
//        {
//            caliFlag = 1;
//            orientationCaliFlag[ORIENTATION_UP] = 1;
//            samples_count = 0;
//            acc.cali.step++;
//            currentOrientation = ORIENTATION_UP;
//            //mavlink���ͼ����ʾ
//            MavlinkSendNoticeEnable(CAL_DOWN_DETECTED);
//        }
//    }

//    if(GetImuOrientation() == ORIENTATION_DOWN && !orientationCaliFlag[ORIENTATION_DOWN])
//    {
//        //�ж�IMU�Ƿ��ھ�ֹ״̬
//        if(GetPlaceStatus() == STATIC)
//            caliCnt++;
//        else
//            caliCnt = 0;

//        if(caliCnt > 1000)
//        {
//            caliFlag = 1;
//            orientationCaliFlag[ORIENTATION_DOWN] = 1;
//            samples_count = 0;
//            acc.cali.step++;
//            currentOrientation = ORIENTATION_DOWN;
//            //mavlink���ͼ����ʾ
//            MavlinkSendNoticeEnable(CAL_UP_DETECTED);
//        }
//    }

//    if(GetImuOrientation() == ORIENTATION_FRONT && !orientationCaliFlag[ORIENTATION_FRONT])
//    {
//        //�ж�IMU�Ƿ��ھ�ֹ״̬
//        if(GetPlaceStatus() == STATIC)
//            caliCnt++;
//        else
//            caliCnt = 0;

//        if(caliCnt > 1000)
//        {
//            caliFlag = 1;
//            orientationCaliFlag[ORIENTATION_FRONT] = 1;
//            samples_count = 0;
//            acc.cali.step++;
//            currentOrientation = ORIENTATION_FRONT;
//            //mavlink���ͼ����ʾ
//            MavlinkSendNoticeEnable(CAL_FRONT_DETECTED);
//        }
//    }

//    if(GetImuOrientation() == ORIENTATION_BACK && !orientationCaliFlag[ORIENTATION_BACK])
//    {
//        //�ж�IMU�Ƿ��ھ�ֹ״̬
//        if(GetPlaceStatus() == STATIC)
//            caliCnt++;
//        else
//            caliCnt = 0;

//        if(caliCnt > 1000)
//        {
//            caliFlag = 1;
//            orientationCaliFlag[ORIENTATION_BACK] = 1;
//            samples_count = 0;
//            acc.cali.step++;
//            currentOrientation = ORIENTATION_BACK;
//            //mavlink���ͼ����ʾ
//            MavlinkSendNoticeEnable(CAL_BACK_DETECTED);
//        }
//    }

//    if(GetImuOrientation() == ORIENTATION_LEFT && !orientationCaliFlag[ORIENTATION_LEFT])
//    {
//        //�ж�IMU�Ƿ��ھ�ֹ״̬
//        if(GetPlaceStatus() == STATIC)
//            caliCnt++;
//        else
//            caliCnt = 0;

//        if(caliCnt > 1000)
//        {
//            caliFlag = 1;
//            orientationCaliFlag[ORIENTATION_LEFT] = 1;
//            samples_count = 0;
//            acc.cali.step++;
//            currentOrientation = ORIENTATION_LEFT;
//            //mavlink���ͼ����ʾ
//            MavlinkSendNoticeEnable(CAL_LEFT_DETECTED);
//        }
//    }

//    if(GetImuOrientation() == ORIENTATION_RIGHT && !orientationCaliFlag[ORIENTATION_RIGHT])
//    {
//        //�ж�IMU�Ƿ��ھ�ֹ״̬
//        if(GetPlaceStatus() == STATIC)
//            caliCnt++;
//        else
//            caliCnt = 0;

//        if(caliCnt > 1000)
//        {
//            caliFlag = 1;
//            orientationCaliFlag[ORIENTATION_RIGHT] = 1;
//            samples_count = 0;
//            acc.cali.step++;
//            currentOrientation = ORIENTATION_RIGHT;
//            //mavlink���ͼ����ʾ
//            MavlinkSendNoticeEnable(CAL_RIGHT_DETECTED);
//        }
//    }
//    /****************************************************************************************/

//    //�ֱ�ɼ����ٶȼ�������������ݣ�˳�����⣬ÿ������ȡ500��������ƽ��ֵ
//    if(caliFlag)
//    {
//        if(samples_count < 500)
//        {
//            samples[acc.cali.step - 1].x += accRaw.x;
//            samples[acc.cali.step - 1].y += accRaw.y;
//            samples[acc.cali.step - 1].z += accRaw.z;
//            samples_count++;
//        }
//        else if(samples_count == 500)
//        {
//            samples[acc.cali.step - 1].x /= 500;
//            samples[acc.cali.step - 1].y /= 500;
//            samples[acc.cali.step - 1].z /= 500;
//            samples_count++;

//            caliFlag = 0;
//            caliCnt  = 0;

//            //bsklink���͵�ǰУ׼����
//            MessageSensorCaliFeedbackEnable(ACC, acc.cali.step, acc.cali.success);

//            //mavlink���͵�ǰУ׼����
//            switch(currentOrientation)
//            {
//            case ORIENTATION_UP:
//                MavlinkSendNoticeEnable(CAL_DOWN_DONE);
//                MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
//                break;
//            case ORIENTATION_DOWN:
//                MavlinkSendNoticeEnable(CAL_UP_DONE);
//                MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
//                break;
//            case ORIENTATION_FRONT:
//                MavlinkSendNoticeEnable(CAL_FRONT_DONE);
//                MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
//                break;
//            case ORIENTATION_BACK:
//                MavlinkSendNoticeEnable(CAL_BACK_DONE);
//                MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
//                break;
//            case ORIENTATION_LEFT:
//                MavlinkSendNoticeEnable(CAL_LEFT_DONE);
//                MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
//                break;
//            case ORIENTATION_RIGHT:
//                MavlinkSendNoticeEnable(CAL_RIGHT_DONE);
//                MavlinkSendNoticeProgress(acc.cali.step * 1.6f);
//                break;
//            default:
//                break;
//            }
//        }
//    }

//    if(acc.cali.step == 6 && samples_count == 501)
//    {
//        //���㷽�̽��ֵ
//        float initBeta[6];
//        initBeta[0] = 0;
//        initBeta[1] = 0;
//        initBeta[2] = 0;
//        initBeta[3] = 1;
//        initBeta[4] = 1;
//        initBeta[5] = 1;

//        //LM����⴫�����������Ž�
//        LevenbergMarquardt(samples, &new_offset, &new_scale, initBeta, 1);

//        //�ж�У׼�����Ƿ�����
//        if(fabsf(new_scale.x-1.0f) > 0.1f || fabsf(new_scale.y-1.0f) > 0.1f || fabsf(new_scale.z-1.0f) > 0.1f)
//        {
//            acc.cali.success = false;
//        }
//        else if(fabsf(new_offset.x) > (1 * 0.35f) || fabsf(new_offset.y) > (1 * 0.35f) || fabsf(new_offset.z) > (1 * 0.6f))
//        {
//            acc.cali.success = false;
//        }
//        else
//        {
//            acc.cali.success = true;
//        }

//        for(u8 i=0; i<6; i++)
//        {
//            samples[i].x = 0;
//            samples[i].y = 0;
//            samples[i].z = 0;
//        }

//        if(acc.cali.success)
//        {
//            acc.cali.offset = new_offset;
//            acc.cali.scale = new_scale;

//            //������ٶ�У׼����
//            ParamUpdateData(PARAM_ACC_OFFSET_X, &acc.cali.offset.x);
//            ParamUpdateData(PARAM_ACC_OFFSET_Y, &acc.cali.offset.y);
//            ParamUpdateData(PARAM_ACC_OFFSET_Z, &acc.cali.offset.z);
//            ParamUpdateData(PARAM_ACC_SCALE_X, &acc.cali.scale.x);
//            ParamUpdateData(PARAM_ACC_SCALE_Y, &acc.cali.scale.y);
//            ParamUpdateData(PARAM_ACC_SCALE_Z, &acc.cali.scale.z);
//            //����mavlink����
//            MavParamSetValue(CAL_ACC0_XOFF, acc.cali.offset.x);
//            MavParamSetValue(CAL_ACC0_YOFF, acc.cali.offset.y);
//            MavParamSetValue(CAL_ACC0_ZOFF, acc.cali.offset.z);
//            MavParamSetValue(CAL_ACC0_XSCALE, acc.cali.scale.x);
//            MavParamSetValue(CAL_ACC0_YSCALE, acc.cali.scale.y);
//            MavParamSetValue(CAL_ACC0_XSCALE, acc.cali.scale.z);

//            //mavlink����У׼���
//            MavlinkSendNoticeEnable(CAL_DONE);
//        }
//        else
//        {
//            //mavlink����У׼���
//            MavlinkSendNoticeEnable(CAL_FAILED);
//        }

//        //����У׼���
//        MessageSensorCaliFeedbackEnable(ACC, acc.cali.step, acc.cali.success);
//        acc.cali.should_cali = 0;
//        acc.cali.step = 0;
//        for(uint8_t i=0; i<6; i++)
//            orientationCaliFlag[i] = 0;
//    }
//}  





