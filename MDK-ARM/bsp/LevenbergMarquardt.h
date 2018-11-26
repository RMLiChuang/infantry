#ifndef _LEVENBERGMARQUARDT_H
#define _LEVENBERGMARQUARDT_H


#include "robomaster_common.h"
//typedef struct {
//    float x;
//    float y;
//    float z;
//} Vector3f_t;
void LevenbergMarquardt(Vector3f_t inputData[6], Vector3f_t* offset, Vector3f_t* scale, float initBeta[6], float length);
float ConstrainFloat(float amt, float low, float high);
#endif
