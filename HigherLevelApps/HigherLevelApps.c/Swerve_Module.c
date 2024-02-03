#include "Swerve_Module.h"
#include "User_Defined_Math.h"

void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State);

/* Initialize physical swerve constants */
void Init_Swerve_Module(Swerve_Module_t *Swerve_Module, bool Azimuth_Encoder_Reversed, int Azimuth_CAN_ID) { //TODO add relevant constants
	Swerve_Module->Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed;
	Swerve_Module->Azimuth_CAN_ID = Azimuth_CAN_ID;
}

/* Optimize wheel angle so wheel doesn't have to rotate more than 90deg*/
Module_State_t Optimize_Module_Angle(Module_State_t Input_State, float Measured_Angle) {
	Module_State_t Optimized_Module_State;
	float Wheel_Angle_Delta = Input_State.Module_Angle - Measured_Angle;
	
	if (fabsf(Wheel_Angle_Delta) > PI/2) { // if the delta is more than 90 degrees
		Optimized_Module_State.Module_Speed = -1 * Input_State.Module_Speed;
		Optimized_Module_State.Module_Angle = fmodf(Input_State.Module_Angle + PI, 2 * PI); //rotate the target by 180 degrees
	}
	
	return Optimized_Module_State;
}

/*Command motors to output calculated module state*/
void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State) {

}
