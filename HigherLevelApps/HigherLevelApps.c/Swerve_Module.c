#include "Swerve_Module.h"
#include "User_Defined_Math.h"

float Measured_Wheel_Speed = 0.0f; //TODO get encoder feedback from motors
float	Measured_Wheel_Angle = 0.0f;

/*Initialize Physical swerve constants*/
void Init_Swerve_Module(Swerve_Module_t *Swerve_Module, bool Azimuth_Encoder_Reversed, int Azimuth_CAN_ID) { //TODO add relavent constants
	Swerve_Module->Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed;
	Swerve_Module->Azimuth_CAN_ID = Azimuth_CAN_ID;
}

/*Optimize wheel angle so wheel doesn't have to rotate more than 90deg*/
Module_State_t Optimize_Module_Angle(Module_State_t Input_State) {
	Module_State_t Optimized_Module_State;
	float Wheel_Angle_Delta = Input_State.Module_Angle - Measured_Wheel_Angle;
	
	if (fabsf(Wheel_Angle_Delta) > 90.0f) {
		Optimized_Module_State.Module_Speed = -1 * Input_State.Module_Speed;
		Optimized_Module_State.Module_Angle = Input_State.Module_Angle + 180.0f;
	}
	
	return Optimized_Module_State;
}

/*Command motors to output calculated module state*/
void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State) {
	Desired_State.Module_Angle = Calculate_Wrapped_Angle(Desired_State.Module_Angle);
	Desired_State = Optimize_Module_Angle(Desired_State);
	
	Swerve_Module->Azimuth_Motor.Output_Current = 
		PID_Func.Positional_PID(&Swerve_Module->Azimuth_PID, Desired_State.Module_Angle, Measured_Wheel_Angle);
	
	Swerve_Module->Drive_Motor.Output_Current = 
		PID_Func.Positional_PID(&Swerve_Module->Drive_PID, Desired_State.Module_Speed, Measured_Wheel_Speed);
}
