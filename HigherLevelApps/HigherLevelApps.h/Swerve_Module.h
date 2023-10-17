#ifndef SWERVE_MODULE_h
#define SWERVE_MODULE_h

#include "PID.h"
#include "M3508_Motor.h"

typedef struct {
	float Module_Speed; // m/s
	float Module_Angle; // deg
} Module_State_t;

typedef struct {
	bool Azimuth_Encoder_Reversed;
	int Azimuth_CAN_ID;
	
	PID_t Azimuth_PID;
	Motor_Init_t Azimuth_Motor;
	
	PID_t Drive_PID;
	Motor_Init_t Drive_Motor;
	
	Module_State_t Module_State;
} Swerve_Module_t;

extern void Init_Swerve_Module(Swerve_Module_t *Swerve_Module, bool Azimuth_Encoder_Reversed, int Azimuth_CAN_ID);
extern void Set_Module_Angle(Swerve_Module_t *Swerve_Module, float angle);
extern void Set_Module_Power(Swerve_Module_t *Swerve_Module, float power);

#endif
