#include "Swerve_Module.h"
#include "User_Defined_Math.h"

void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State);

/* Initialize physical swerve constants */
void Init_Swerve_Module(Swerve_Module_t *Swerve_Module, bool Azimuth_Encoder_Reversed, int Azimuth_CAN_ID) { //TODO add relavent constants
	Swerve_Module->Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed;
	Swerve_Module->Azimuth_CAN_ID = Azimuth_CAN_ID;
}

/* Optimize wheel angle so wheel doesn't have to rotate more than 90deg */
Module_State_t Optimize_Module_Angle(Module_State_t Input_State, float Azimuth_Current_Angle) {
	float Wheel_Angle_Delta = Input_State.Module_Angle - Azimuth_Current_Angle;

    Module_State_t Optimized_Module_State;
	if (fabsf(Wheel_Angle_Delta) > PI/2) {
		Optimized_Module_State.Module_Speed = -1 * Input_State.Module_Speed;
		Optimized_Module_State.Module_Angle = Input_State.Module_Angle + PI;
	}
	
	return Optimized_Module_State;
}

/* Command motors to output calculated module state */
void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State) {
	Desired_State.Module_Angle = Calculate_Wrapped_Angle(Desired_State.Module_Angle);
    float Azimuth_Current_Angle = (Swerve_Module->Azimuth_Motor.Total_Angle - Swerve_Module->Azimuth_Encoder_Zero_Offset) / GM6020_MECH_ANGLE_MAX * (2*PI);
	Desired_State = Optimize_Module_Angle(Desired_State, Azimuth_Current_Angle);

    float Azimuth_Error = Calculate_Wrapped_Error(Desired_State.Module_Angle, Azimuth_Current_Angle);
	Swerve_Module->Azimuth_Motor.Output_Current = 
		PID_Func.Positional_PID(&Swerve_Module->Azimuth_PID, Azimuth_Error, 0);
	
	Swerve_Module->Drive_Motor.Output_Current = 
		PID_Func.Positional_PID(&Swerve_Module->Drive_PID, Desired_State.Module_Speed, Swerve_Module->Drive_Motor.Actual_Speed);
}
