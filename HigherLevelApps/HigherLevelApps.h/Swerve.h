#ifndef SWERVE_h
#define SWERVE_h

#include "Swerve_Module.h"

//PHYSICAL CONSTANTS
#define Swerve_Max_Speed 1.0f // m/s
#define Swerve_Width 1.0f // m, measured wheel to wheel
#define Swerve_Height 1.0f // m, measured wheel to wheel

//MODULE CONSTANTS
bool Azimuth_Encoder_Reversed_Array[4] = {true, true, true, true}; //made up
int Azimuth_CAN_ID[4] = {1, 2, 3, 4}; //random numbers

typedef struct {
	Swerve_Module_t Modules[4];
} Swerve_t;

typedef struct {
	Module_State_t States[4];
} Module_State_Array_t;


#endif
