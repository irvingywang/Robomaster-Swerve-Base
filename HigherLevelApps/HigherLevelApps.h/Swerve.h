#ifndef SWERVE_h
#define SWERVE_h

#include "Swerve_Module.h"
#define ARM_MATH_CM4
#include "arm_math.h"

//PHYSICAL CONSTANTS
#define Swerve_Max_Speed 1.0f // m/s
#define Swerve_Width 1.0f // m, measured wheel to wheel
#define Swerve_Height 1.0f // m, measured wheel to wheel

//MODULE CONSTANTS 
bool Azimuth_Encoder_Reversed_Array[4] = {true, true, true, true}; 
int Azimuth_CAN_ID[4] = {1, 2, 3, 4};
float Animuth_Encoder_Zero_Offset[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // encoder ticks 

typedef struct {
	int Current_Mode;
	Swerve_Module_t Modules[4];
} Swerve_t;

typedef struct {
	float X_Speed, Y_Speed, Theta_Speed;
} Chassis_Speeds_t;

typedef struct {
	Module_State_t States[4];
} Module_State_Array_t;

//extern void drive();
#endif
