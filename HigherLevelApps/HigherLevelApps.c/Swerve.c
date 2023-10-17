#include "Swerve.h"
#include "Swerve_Module.h"

/* Declare swerve struct */
Swerve_t Swerve;

/* Initailize physical constants of each module*/
void Init_Modules() {
	for (int i=0; i<4; i++) {
		Swerve.Modules[i].Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed_Array[i]; 
	}
}

/* Set the desired modules state of each module */
void Set_Desired_States(Module_State_Array_t Desired_States) {
	for (int i=0; i<4; i++) {
		Swerve.Modules[i].Module_State = Desired_States.States[i];
	}
}

/* Scale wheel speeds to max possible speed while preserving ratio between modules.*/
Module_State_Array_t Desaturate_Wheel_Speeds(Module_State_Array_t Module_State_Array) {
	float Highest_Speed;
	for (int i=0; i<4; i++) {
		if(Module_State_Array.States[i].Module_Speed > fabs(Highest_Speed)) {
			Highest_Speed = Module_State_Array.States[i].Module_Speed;
		}
	}
	float Desaturation_Coefficient = Swerve_Max_Speed / Highest_Speed;
	
	Module_State_Array_t Desaturated_Module_States;
	
	for (int i=0; i<4; i++) {
		Desaturated_Module_States.States[i].Module_Speed = Module_State_Array.States[i].Module_Speed / Desaturation_Coefficient;
	}
	
	return Desaturated_Module_States;
}
	

/* Inverse swerve kinematics*/
Module_State_Array_t Chassis_Speeds_To_Module_States(Chassis_Speeds_t Chassis_Speeds) { 
	Module_State_Array_t Calculated_Module_States;
	if (Chassis_Speeds.X_Speed == 0 && Chassis_Speeds.X_Speed == 0 && Chassis_Speeds.X_Speed == 0) {
		for (int i=0; i<4; i++) {
			Calculated_Module_States.States[i].Module_Speed = 0; 
			Calculated_Module_States.States[i].Module_Angle = 0;
		}
	} else {//TODO calculate inverse kinematics via matrix (multiply by chassis speed vector)
		
	}
	return Calculated_Module_States;
}

/* Takes driver input and sets respective wheel speeds without kinematics (drives like a car)*/
void drive() {
	Module_State_Array_t New_States;
	for (int i=0; i<4; i++) {
		New_States.States[i].Module_Speed = DR16_Export_Data.Remote_Control.Joystick_Left_Vx;;
		New_States.States[i].Module_Angle = DR16_Export_Data.Remote_Control.Joystick_Right_Vx;;
	}
	Set_Desired_States(Desaturate_Wheel_Speeds(New_States));
}

void Swerve_Processing(Swerve_t *Swerve) { // TODO
	switch (Swerve->Current_Mode) {
		case (Follow_Gimbal): {
			
		}
	}
	
	drive();
}

/* TODO drive with kinematics
void drive(float x, float y, float theta) {
	
}
*/


