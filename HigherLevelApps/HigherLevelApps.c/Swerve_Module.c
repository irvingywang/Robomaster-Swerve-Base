#include "Swerve_Module.h"
#include "User_Defined_Math.h"

void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State);

/* Initialize physical swerve constants */
void Init_Swerve_Module(Swerve_Module_t *Swerve_Module, bool Azimuth_Encoder_Reversed, int Azimuth_CAN_ID) { //TODO add relevant constants
	Swerve_Module->Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed;
	Swerve_Module->Azimuth_CAN_ID = Azimuth_CAN_ID;
}

/*Optimize wheel angle so wheel doesn't have to rotate more than 90deg*/
Module_State_t Optimize_Module_Angle(Module_State_t Input_State, float Measured_Angle) {
	Module_State_t Optimized_Module_State;
	float Wheel_Angle_Delta = Input_State.Module_Angle - Measured_Angle;
	
	if (fabsf(Wheel_Angle_Delta) > 3.1415927f/2) {
		Optimized_Module_State.Module_Speed = -1 * Input_State.Module_Speed;
		Optimized_Module_State.Module_Angle = fmodf(Input_State.Module_Angle + 3.1415927f, 2 * 3.1415927f);
	}
	
	return Optimized_Module_State;
}

Error_t get_azimuth_error(float target, float current)
{
	Error_t error;
	error.direction = 1;
	int counter = 0;
	error.error = target;
  float error_amount = error.error - current;
	while ((error.error - current) >= PI/2)	{
		error.error -= PI;
		counter++;
	}
	while((error.error - current) <= -PI/2) {
		error.error += PI;
		counter++;
	}
	if ((counter%2) == 1) {
		error.direction = -1;
	}
  return error;
}

/*Command motors to output calculated module state*/
void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State) {
	
	float Azimuth_Current_Angle = (Swerve_Module->Azimuth_Motor.Total_Angle - Swerve_Module->Azimuth_Encoder_Zero_Offset) / GM6020_MECH_ANGLE_MAX * 2 * 3.1415927f;
	
	//Desired_State.Module_Angle = Calculate_Wrapped_Angle(Desired_State.Module_Angle);
	//Desired_State = Optimize_Module_Angle(Desired_State, Swerve_Module->Azimuth_Motor.Angle_Rad);
	
    

    //Error_t azimuth_error = get_azimuth_error(Desired_State.Module_Angle, Swerve_Module->Azimuth_Motor.Angle_Rad);
	
    Swerve_Module->Azimuth_Motor.Output_Current =
		-PID_Func.Circular_PID(&Swerve_Module->Azimuth_PID, Desired_State.Module_Angle, Swerve_Module->Azimuth_Motor.Angle_Rad);
		
	float current_drive_speed = Swerve_Module->Drive_Motor.Actual_Speed / 16.8f * 2 * PI / 60.0f * 0.115f / 2.0f;
	if (fabs(Desired_State.Module_Speed) > 0.01f){
	Swerve_Module->Drive_Motor.Output_Current = //(Desired_State.Module_Speed / SWERVE_MAX_SPEED ) * 1000.0f;
		PID_Func.Positional_PID(&Swerve_Module->Drive_PID, Desired_State.Module_Speed, current_drive_speed);
	}
	else{
		Swerve_Module->Drive_Motor.Output_Current = 0.0f;
	}
}
