#ifndef SWERVE_h
#define SWERVE_h

#include "Swerve_Module.h"
#define ARM_MATH_CM4
#include "arm_math.h"

//PHYSICAL CONSTANTS
#define SWERVE_MAX_SPEED 1.0f // m/s
#define SWERVE_MAX_ANGLUAR_SPEED 90.0f // deg/s
#define TRACK_WIDTH 0.23f // m, measured wheel to wheel (side to side)
#define WHEEL_BASE 0.23f // m, measured wheel to wheel (up and down)

//MODULE CONSTANTS
//ORDER MATTERS - front left, front right, back left, back right
//bool Azimuth_Encoder_Reversed_Array[4] = {true, true, true, true};
//int Azimuth_CAN_ID[4] = {1, 2, 3, 4};
//float Azimuth_Encoder_Zero_Offset[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // encoder ticks

//float Swerve_Inverse_Kinematics[8][3] = {
//        { 1, 0, -(WHEEL_BASE/2)}, //front left 1
//        {0, 1, +(-TRACK_WIDTH/2)},
//        {1, 0 ,-(WHEEL_BASE/2)}, //front right 2
//        {0, 1, +(TRACK_WIDTH/2)},
//        {1, 0 ,-(-WHEEL_BASE/2)}, //back left 3
//        {0, 1, +(-TRACK_WIDTH/2)},
//        {1, 0 ,-(-WHEEL_BASE/2)}, //back right 4
//        {0, 1, +(TRACK_WIDTH/2)}};

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

extern Swerve_t Swerve;

extern void Init_Modules(void);
extern void Swerve_Processing(Swerve_t *Swerve);

//extern void drive();
#endif
