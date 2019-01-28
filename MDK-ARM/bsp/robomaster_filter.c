#include "robomaster_filter.h"

//-----Butterworth变量 Matlab fdatool设计-----//
//设计方法见博客：http://blog.csdn.net/u011992534/article/details/73743955
/*
Butter_Parameter Butter_80HZ_Parameter_Acce={
  //200hz---80hz
1,     1.14298050254,   0.4128015980962,
0.638945525159,    1.277891050318,    0.638945525159
};

Butter_Parameter Butter_60HZ_Parameter_Acce={
  //200hz---60hz
1,   0.3695273773512,   0.1958157126558,
0.3913357725018,   0.7826715450035,   0.3913357725018
};

Butter_Parameter Butter_51HZ_Parameter_Acce={
  //200hz---51hz
1,  0.03680751639284,   0.1718123812701,
0.3021549744157,   0.6043099488315,   0.3021549744157,
};

Butter_Parameter Butter_30HZ_Parameter_Acce={
  //200hz---30hz
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter Butter_20HZ_Parameter_Acce={
  //200hz---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};
Butter_Parameter Butter_15HZ_Parameter_Acce={
  //200hz---15hz
  1,   -1.348967745253,   0.5139818942197,
  0.04125353724172,  0.08250707448344,  0.04125353724172
};

Butter_Parameter Butter_10HZ_Parameter_Acce={
  //200hz---10hz
  1,   -1.561018075801,   0.6413515380576,
  0.02008336556421,  0.04016673112842,  0.02008336556421};
Butter_Parameter Butter_5HZ_Parameter_Acce={
  //200hz---5hz
  1,   -1.778631777825,   0.8008026466657,
  0.005542717210281,  0.01108543442056, 0.005542717210281
};

Butter_Parameter Butter_2HZ_Parameter_Acce={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};
*/


Butter_Parameter Accel_Parameter={
  1,  -0.7477891782585,    0.272214937925,
  0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter Gyro_Parameter={
  //200hz---51hz
  1,  0.03680751639284,   0.1718123812701,
  0.3021549744157,   0.6043099488315,   0.3021549744157
};
Butter_Parameter Butter_1HZ_Parameter_Acce={
  //200hz---1hz
  1,   -1.955578240315,   0.9565436765112,
  0.000241359049042, 0.000482718098084, 0.000241359049042
};

Butter_Parameter Bandstop_Filter_Parameter_30_94={
  //200hz---30hz-94hz  采样-阻带
  1,   0.5334540355829,  -0.2235264828971,
  0.3882367585514,   0.5334540355829,   0.3882367585514
};

Butter_Parameter Bandstop_Filter_Parameter_30_98={
  //200hz---30hz-98hz  采样-阻带
  1,   0.6270403082828,  -0.2905268567319,
  0.354736571634,   0.6270403082828,    0.354736571634
};









//-----Butterworth变量-----//
Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
                 Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
                 Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;
Butter_BufferData Butter_Buffer[3];
Butter_BufferData Butter_Buffer_Feedback[3];
Butter_BufferData Butter_Buffer_SINS[3];
/****************************************
     Butterworth低通滤波器参数初始化：http://blog.csdn.net/u011992534/article/details/73743955
***************************************/
float Butterworth_Filter(float curr_inputer,
                         Butter_BufferData *Buffer,
                         Butter_Parameter *Parameter)
{
  /* 加速度计Butterworth滤波 */
  /* 获取最新x(n) */
  Buffer->Input_Butter[2]=curr_inputer;
  /* Butterworth滤波 */
  Buffer->Output_Butter[2]=
    Parameter->b[0] * Buffer->Input_Butter[2]
      +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
          -Parameter->a[1] * Buffer->Output_Butter[1]
            -Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) 序列保存 */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) 序列保存 */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return (Buffer->Output_Butter[2]);
}


void Butterworth_Parameter_Init(void)
{
  Set_Cutoff_Frequency(Sampling_Freq, 80,&Butter_80HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 60,&Butter_60HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 51,&Butter_51HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 30,&Butter_30HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 20,&Butter_20HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 15,&Butter_15HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 10,&Butter_10HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 5 ,&Butter_5HZ_Parameter_Acce);
  Set_Cutoff_Frequency(Sampling_Freq, 2 ,&Butter_2HZ_Parameter_Acce);
}

/*************************************************
函数名:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
说明:	加速度计低通滤波器
入口:	float curr_input 当前输入加速度计,滤波器参数，滤波器缓存
出口:	无
备注:	2阶Butterworth低通滤波器
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
        /* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
        static int LPF_Cnt=0;
        Buffer->Input_Butter[2]=curr_input;
        if(LPF_Cnt>=100)
        {
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
        }
        else
        {
          Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
          LPF_Cnt++;
        }
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return Buffer->Output_Butter[2];
}

Butter_BufferData Accel9250_BufferData[3];
Butter_BufferData Accel9250_BufferData_BPF[3];
Butter_BufferData Gyro9250_BufferData_BPF[3];
Butter_BufferData Gyro9250_BufferData[3];

Butter_BufferData Accel_BufferData[3];
Butter_BufferData Accel_BufferData_BPF[3];
Butter_BufferData Gyro_BufferData_BPF[3];
Butter_BufferData Gyro_BufferData[3];
Butter_BufferData Vision_BufferData[2];
float MPU_LPF(float curr_inputer,
               Butter_BufferData *Buffer,
               Butter_Parameter *Parameter)
{
        /* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
        Buffer->Input_Butter[2]=curr_inputer;
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0]
        -Parameter->a[1] * Buffer->Output_Butter[1]
        -Parameter->a[2] * Buffer->Output_Butter[0];
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        return (Buffer->Output_Butter[2]);
}


float Acce_Control[3]={0};
float Acce_Control_Feedback[3]={0};
float Acce_SINS[3]={0};
//void Acce_Control_Filter()
//{
///**********************惯导加速度LPF_60hz**************************/
//   Vector3f Body_Frame,Earth_Frame;

//   Acce_SINS[0]=LPButterworth(X_Origion,
//                    &Butter_Buffer_SINS[0],&Butter_60HZ_Parameter_Acce);
//   Acce_SINS[1]=LPButterworth(Y_Origion
//                    ,&Butter_Buffer_SINS[1],&Butter_60HZ_Parameter_Acce);
//   Acce_SINS[2]=LPButterworth(Z_Origion
//                    ,&Butter_Buffer_SINS[2],&Butter_60HZ_Parameter_Acce);//用作惯导融合的加速度计量

//   Acce_Control[0]=LPButterworth(X_Origion,
//                    &Butter_Buffer[0],&Butter_51HZ_Parameter_Acce);
//   Acce_Control[1]=LPButterworth(Y_Origion
//                    ,&Butter_Buffer[1],&Butter_51HZ_Parameter_Acce);
//   Acce_Control[2]=LPButterworth(Z_Origion
//                    ,&Butter_Buffer[2],&Butter_51HZ_Parameter_Acce);

//   Body_Frame.x=Acce_Control[0];
//   Body_Frame.y=Acce_Control[1];
//   Body_Frame.z=Acce_Control[2];
//   Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
//   FilterAfter_NamelessQuad.Acceleration[_YAW]=Earth_Frame.z;
//   FilterAfter_NamelessQuad.Acceleration[_PITCH]=Earth_Frame.x;
//   FilterAfter_NamelessQuad.Acceleration[_ROLL]=Earth_Frame.y;
//   /*
//   FilterAfter_NamelessQuad.Acceleration[_YAW]=
//                      -Sin_Roll* Acce_Control[0]
//                        + Sin_Pitch *Cos_Roll * Acce_Control[1]
//                           + Cos_Pitch * Cos_Roll * Acce_Control[2];
//   FilterAfter_NamelessQuad.Acceleration[_PITCH]=
//                      Cos_Yaw* Cos_Roll * Acce_Control[0]
//                        +(Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw) *Acce_Control[1]
//                          +(Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw) * Acce_Control[2];

//   FilterAfter_NamelessQuad.Acceleration[_ROLL]=
//                      Sin_Yaw* Cos_Roll * Acce_Control[0]
//                        +(Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw) * Acce_Control[1]
//                          + (Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw) * Acce_Control[2];
//   */
//   FilterAfter_NamelessQuad.Acceleration[_YAW]*=AcceGravity/AcceMax;
//   FilterAfter_NamelessQuad.Acceleration[_YAW]-=AcceGravity;
//   FilterAfter_NamelessQuad.Acceleration[_YAW]*=100;//加速度cm/s^2
//   FilterAfter_NamelessQuad.Acceleration[_PITCH]*=AcceGravity/AcceMax;
//   FilterAfter_NamelessQuad.Acceleration[_PITCH]*=100;//加速度cm/s^2
//   FilterAfter_NamelessQuad.Acceleration[_ROLL]*=AcceGravity/AcceMax;
//   FilterAfter_NamelessQuad.Acceleration[_ROLL]*=100;//加速度cm/s^2

///**********************加速度反馈量LPF_5hz*****************************************************************************/
//   Acce_Control_Feedback[0]=LPButterworth(X_Origion,
//                    &Butter_Buffer_Feedback[0],&Butter_5HZ_Parameter_Acce);//用作加速度反馈的加速度计量
//   Acce_Control_Feedback[1]=LPButterworth(Y_Origion
//                    ,&Butter_Buffer_Feedback[1],&Butter_5HZ_Parameter_Acce);
//   Acce_Control_Feedback[2]=LPButterworth(Z_Origion
//                    ,&Butter_Buffer_Feedback[2],&Butter_5HZ_Parameter_Acce);
///******************如果定高时，发现飞机有时莫名抽、抖，可以把加速度反馈滤波截止频率降低，APM飞控用的Fc=2Hz******************/

//   Body_Frame.x=Acce_Control_Feedback[0];
//   Body_Frame.y=Acce_Control_Feedback[1];
//   Body_Frame.z=Acce_Control_Feedback[2];
//   Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame);
//   Filter_Feedback_NamelessQuad.Acceleration[_YAW]=Earth_Frame.z;
//   Filter_Feedback_NamelessQuad.Acceleration[_PITCH]=Earth_Frame.x;
//   Filter_Feedback_NamelessQuad.Acceleration[_ROLL]=Earth_Frame.y;
///*
//   Filter_Feedback_NamelessQuad.Acceleration[_YAW]=
//                      -Sin_Roll* Acce_Control_Feedback[0]
//                        + Sin_Pitch *Cos_Roll * Acce_Control_Feedback[1]
//                           + Cos_Pitch * Cos_Roll * Acce_Control_Feedback[2];
//   Filter_Feedback_NamelessQuad.Acceleration[_PITCH]=
//                      Cos_Yaw* Cos_Roll * Acce_Control_Feedback[0]
//                        +(Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw) *Acce_Control_Feedback[1]
//                          +(Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw) * Acce_Control_Feedback[2];
//   Filter_Feedback_NamelessQuad.Acceleration[_ROLL]=
//                      Sin_Yaw* Cos_Roll * Acce_Control_Feedback[0]
//                        +(Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw) * Acce_Control_Feedback[1]
//                          + (Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw) * Acce_Control_Feedback[2];
//*/
//   Filter_Feedback_NamelessQuad.Acceleration[_YAW]*=AcceGravity/AcceMax;
//   Filter_Feedback_NamelessQuad.Acceleration[_YAW]-=AcceGravity;
//   Filter_Feedback_NamelessQuad.Acceleration[_YAW]*=100;//加速度cm/s^2
//   Filter_Feedback_NamelessQuad.Acceleration[_PITCH]*=AcceGravity/AcceMax;
//   Filter_Feedback_NamelessQuad.Acceleration[_PITCH]*=100;//加速度cm/s^2
//   Filter_Feedback_NamelessQuad.Acceleration[_ROLL]*=AcceGravity/AcceMax;
//   Filter_Feedback_NamelessQuad.Acceleration[_ROLL]*=100;//加速度cm/s^2
//}

// discrete low pass filter, cuts out the
// high frequency noise that can drive the controller crazy
//derivative = _last_derivative + _d_lpf_alpha * (derivative - _last_derivative);
float set_lpf_alpha(int16_t cutoff_frequency, float time_step)
{
    // calculate alpha
    float lpf_alpha;
    float rc = 1/(2*PI*cutoff_frequency);
    lpf_alpha = time_step / (time_step + rc);
    return lpf_alpha;
}

/************************************************************************/
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
	float fr = sample_frequent / cutoff_frequent;
	float ohm = tanf(M_PI_F / fr);
	float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
	if (cutoff_frequent <= 0.0f) {
		// no filtering
		return;
	}
	LPF->b[0] = ohm * ohm / c;
	LPF->b[1] = 2.0f * LPF->b[0];
	LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
	LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
	LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}


//磁力计滤波
float Data_X_MAG[N2];
float Data_Y_MAG[N2];
float Data_Z_MAG[N2];
float GildeAverageValueFilter_MAG(float NewValue,float *Data)
{
  float max,min;
  float sum;
  unsigned char i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=N2-1;i!=0;i--)
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];
  }
  i=N2-2;
  sum=sum-max-min;
  sum=sum/i;
  return(sum);
}

