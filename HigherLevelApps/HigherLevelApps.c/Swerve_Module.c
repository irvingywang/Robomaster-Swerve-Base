#include "Swerve_Module.h"

float measured = 0.0f; //TODO get encoder feedback from motors

/*Initialize Physical swerve constants*/
void Init_Swerve_Module(Swerve_Module_t *Swerve_Module, bool Azimuth_Encoder_Reversed, int Azimuth_CAN_ID) { //TODO add relavent constants
	Swerve_Module->Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed;
	Swerve_Module->Azimuth_CAN_ID = Azimuth_CAN_ID;
}

/*Command motors to output calulated module state*/
void Set_Module_Output(Swerve_Module_t *Swerve_Module, Module_State_t Desired_State) {//todo
	Swerve_Module->Azimuth_Motor.Output_Current = 
		PID_Func.Positional_PID(&Swerve_Module->Azimuth_PID, Desired_State.Module_Angle, measured);
	
	Swerve_Module->Drive_Motor.Output_Current = Desired_State.Module_Speed*24.0f;
}

float Optimize_Module_Angle() {
		return 0.0;
}

/*void Set_Module_Angle(Swerve_Module_t *Swerve_Module, float angle) {
	Swerve_Module->Azimuth_Motor.Output_Current = PID_Func.Positional_PID(&Swerve_Module->Azimuth_PID, angle, measured);
}

void Set_Module_Power(Swerve_Module_t *Swerve_Module, float power) {
	//Swerve_Module->Drive_Motor.Output_Current = power*24.0f;
}*/
