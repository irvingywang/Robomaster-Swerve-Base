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

float VAL_LIMIT(float Value, float Upper_Limit, float Lower_Limit);
float Find_Gimbal_Min_Angle(float Angle);

// Limit value through upper and lower bounds
float VAL_LIMIT(float Value, float Upper_Limit, float Lower_Limit)
{
    if (Value > Upper_Limit)
        Value = Upper_Limit;
    else if (Value < Lower_Limit)
        Value = Lower_Limit;

    return Value;
}

// Find the minimum rotation angle for gimbal
float Find_Gimbal_Min_Angle(float Angle)
{
    if (Angle > (GM6020_MECH_ANGLE_MAX / 2))
        Angle -= GM6020_MECH_ANGLE_MAX;
    else if (Angle < -(GM6020_MECH_ANGLE_MAX / 2))
        Angle += GM6020_MECH_ANGLE_MAX;

    return Angle;
}

/* Wrap the input of a circular system in radian */
float Calculate_Wrapped_Angle(float radians)
{
    return fmodf(radians, 2 * PI);
}

/* Get the difference of the measured_val angle and the target angle in a circular system */
float Calculate_Wrapped_Error(float input, float measured_val, float minimumInput, float maximumInput)
{
    // float error = Target_Radians - Current_Radians;
    // error = (fmodf(error, PI));
    // error = (error >= PI / 2) ? (error - PI) : error;
    // error = (error <= -PI / 2) ? (error + PI) : error;
    // return error;

    float mid_point = (minimumInput + maximumInput) / 2.0;  
    float range = maximumInput - minimumInput;
    float curr_normalized = fmod(measured_val, range);
    if (curr_normalized < 0)
    {
        curr_normalized += range;
    }
    float error = input - curr_normalized;
    
    if (error < -mid_point)
    {
        error = range + error;
    }
    else if (error > mid_point)
    {
        error = range - error;
    }

    return error;
}

float Calculate_Wrapped_Input(float input, float measured_val, float minimumInput, float maximumInput)
{
    // double modulus = maximumInput - minimumInput;
    // Wrap input if it's above the maximum input
    // int numMax = (int)((input - minimumInput) / modulus);
    // input -= numMax * modulus;
    // Wrap input if it's below the minimum input
    // int numMin = (int)((input - maximumInput) / modulus);
    // input -= numMin * modulus;

    float new_target = measured_val;
    float mid_point = (minimumInput + maximumInput) / 2.0;  
    float range = maximumInput - minimumInput;
    float curr_normalized = fmod(measured_val, range);
    if (curr_normalized < 0)
    {
        curr_normalized += range;
    }
    float diff = input - curr_normalized;
    
    if (diff < -mid_point)
    {
        new_target += range + diff;
    }
    else if (diff > mid_point)
    {
        new_target -= range - diff;
    }
    else
    {
        new_target += diff;
    }

    return new_target;
}

float circulate(float target, float curr)
{
    target = fmodf(target, 360.0);
    if (target < 0)
    {
        target += 360;
    }

    float curr_normalized = fmodf(curr, 360.0);
    float diff = target - curr_normalized;

    if (90.0 < fabsf(diff) && fabsf(diff) < 270.0)
    {
        float beta = 180.0 - fabsf(diff);
        beta *= signum(diff);
        target = curr - beta;
    }
    else if (fabs(diff) >= 270.0)
    {
        if (diff < 0)
        {
            target = curr + (360.0 + diff);
        }
        else
        {
            target = curr - (360.0 - diff);
        }
    }
    else
    {
        target = curr + diff;
    }

    return target;
}
