/**
 * @file User_Defined_Math.c
 * @author Leo Liu
 * @brief Basic Math/Logic Operations
 * @version 1.0
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "User_Defined_Math.h"

float VAL_LIMIT(float Value, float Upper_Limit,float Lower_Limit);
float Find_Gimbal_Min_Angle(float Angle);

//Limit value through upper and lower bounds
float VAL_LIMIT(float Value, float Upper_Limit, float Lower_Limit)
{
	if(Value > Upper_Limit)
		Value = Upper_Limit;
	else if(Value < Lower_Limit)
		Value = Lower_Limit;
	
	return Value;
}

//Find the minimum rotation angle for gimbal
float Find_Gimbal_Min_Angle(float Angle)
{
	if(Angle > (GM6020_MECH_ANGLE_MAX/2))
		Angle -= GM6020_MECH_ANGLE_MAX;
	else if(Angle < -(GM6020_MECH_ANGLE_MAX/2))
		Angle += GM6020_MECH_ANGLE_MAX;
	
	return Angle;
}

/* Wrap the input of a circular system in radian */
float Calculate_Wrapped_Angle(float radians) {
	return fmodf(radians, 2*PI);
}

/* Get the difference of the current angle and the target angle in a circular system */
float Calculate_Wrapped_Error(float Current_Radians, float Target_Radians) {
    float error = Target_Radians - Current_Radians;
    error = (fmodf(error, PI));
    error = (error >= PI/2) ? (error - PI) : error;
    error = (error <= -PI/2) ? (error + PI) : error;
    return error;
}
