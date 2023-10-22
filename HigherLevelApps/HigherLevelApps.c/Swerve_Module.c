#include "Swerve_Module.h"
#include "User_Defined_Math.h"
#define PI 3.1415927f

float Measured_Wheel_Speed = 0.0f; //TODO get encoder feedback from motors
float	Measured_Wheel_Angle = 0.0f;

void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State);

/*Initialize Physical swerve constants*/
void Init_Swerve_Module(Swerve_Module_t *Swerve_Module, bool Azimuth_Encoder_Reversed, int Azimuth_CAN_ID) { //TODO add relavent constants
	Swerve_Module->Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed;
	Swerve_Module->Azimuth_CAN_ID = Azimuth_CAN_ID;
}

/*Optimize wheel angle so wheel doesn't have to rotate more than 90deg*/
Module_State_t Optimize_Module_Angle(Module_State_t Input_State) {
	Module_State_t Optimized_Module_State;
	float Wheel_Angle_Delta = Input_State.Module_Angle - Measured_Wheel_Angle;
	
	if (fabsf(Wheel_Angle_Delta) > 3.1415927f/2) {
		Optimized_Module_State.Module_Speed = -1 * Input_State.Module_Speed;
		Optimized_Module_State.Module_Angle = Input_State.Module_Angle + 3.1415927f;
	}
	
	return Optimized_Module_State;
}

float get_azimuth_error(float target, float current)
{
    // target = -2, current = 359
    // target = 359, current = 0
    // target = 0, current = 359
    float error = target - current;
//    error = (fmod(error, 2 * PI));
//    error = (error >= PI) ? (error - 2 * PI) : error;
//    error = (error <= -PI) ? (error + 2 * PI) : error;
    error = (fmod(error, PI));
    error = (error >= PI / 2) ? (error - PI) : error;
    error = (error <= -PI / 2) ? (error + PI) : error;
    return error;
}

/*Command motors to output calculated module state*/
void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State) {
	Desired_State.Module_Angle = Calculate_Wrapped_Angle(Desired_State.Module_Angle);
	//Desired_State = Optimize_Module_Angle(Desired_State);
	
    float Azimuth_Current_Angle = (Swerve_Module->Azimuth_Motor.Total_Angle - Swerve_Module->Azimuth_Encoder_Zero_Offset) / GM6020_MECH_ANGLE_MAX * 2 * 3.1415927f;
    
    float azimuth_error = get_azimuth_error(Desired_State.Module_Angle, Azimuth_Current_Angle);
	Swerve_Module->Azimuth_Motor.Output_Current = 
		PID_Func.Positional_PID(&Swerve_Module->Azimuth_PID, azimuth_error, 0);
	
	Swerve_Module->Drive_Motor.Output_Current = 
		PID_Func.Positional_PID(&Swerve_Module->Drive_PID, Desired_State.Module_Speed, Swerve_Module->Drive_Motor.Actual_Speed);
}
