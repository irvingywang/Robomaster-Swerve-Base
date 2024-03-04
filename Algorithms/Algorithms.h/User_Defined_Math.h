/**
 * @file User_Defined_Math.h
 * @author Leo Liu
 * @brief Basic Math/Logic Operations
 * @version 1.0
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __USER_DEFINED_MATH_H
#define __USER_DEFINED_MATH_H

#include "GM6020_Motor.h"
#include "Chassis_Control.h"
#include <stdio.h>

#define PI 3.1415927f
#define DEG_TO_RAD(DEG) (DEG * 0.0174532925199432957692369076848f)
#define RAD_TO_DEG(RAD)	(RAD * 57.295779513082320876798154814105f)

#define VAL_MIN(a, b) ((a) < (b) ? (b) : (a))
#define VAL_MAX(a, b) ((a) > (b) ? (b) : (a))

extern float VAL_LIMIT(float Value, float Upper_Limit, float Lower_Limit);
extern float Find_Gimbal_Min_Angle(float Angle);
extern float Calculate_Wrapped_Angle(float radians);
float Calculate_Wrapped_Error(float input, float measured_val, float minimumInput, float maximumInput);
// extern float Calculate_Wrapped_Input(double input, double minimumInput, double maximumInput);
float Calculate_Wrapped_Input(float input, float measured_val, float minimumInput, float maximumInput);
float circulate(float target, float curr);

#endif
