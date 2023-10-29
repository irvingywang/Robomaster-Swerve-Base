#include "Swerve.h"
#include "Swerve_Module.h"

/* Declare swerve struct */
Swerve_t Swerve;
float last_swerve_angle[4] = {.0f, .0f, .0f, .0f};
bool Azimuth_Encoder_Reversed_Array[4] = {true, true, true, true};
int Azimuth_CAN_ID[4] = {0x205, 0x206, 0x207, 0x208};
uint16_t Drive_CAN_ID[4] = {0x202, 0x204, 0x203, 0x201};
//float Azimuth_Encoder_Zero_Offset[4] = {5470.0f, 5483.0f, 6267.0f, 7444.0f}; // encoder ticks
float Azimuth_Encoder_Zero_Offset[4] = {5470.0f, 1390.0f, 6267.0f, 3410.0f}; // encoder ticks

// Inverse kinematics matrix for a 4 module swerve
float Swerve_Inverse_Kinematics[8][3] = {
        { 1, 0, -(WHEEL_BASE/2)}, //front left 0
        {0, 1, +(-TRACK_WIDTH/2)},
        {1, 0 ,-(WHEEL_BASE/2)}, //front right 1
        {0, 1, +(TRACK_WIDTH/2)},
        {1, 0 ,-(-WHEEL_BASE/2)}, //back left 2
        {0, 1, +(-TRACK_WIDTH/2)},
        {1, 0 ,-(-WHEEL_BASE/2)}, //back right 3
        {0, 1, +(TRACK_WIDTH/2)}};

void Init_Modules(void);
void Swerve_Processing(Swerve_t *Swerve);

/* Initialize physical constants of each module */
void Init_Modules() {
	for (int i=0; i<NUMBER_OF_MODULES; i++) {
		Swerve.Modules[i].Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed_Array[i];
        Swerve.Modules[i].Azimuth_CAN_ID = Azimuth_CAN_ID[i];
        Swerve.Modules[i].Azimuth_Encoder_Zero_Offset = Azimuth_Encoder_Zero_Offset[i];
				Swerve.Modules[i].Azimuth_Motor.Angle_Offset = Azimuth_Encoder_Zero_Offset[i];
				Swerve.Modules[i].Azimuth_PID.Kp = 10000.0f;
        Swerve.Modules[i].Azimuth_PID.Ki = 0.0f;
        Swerve.Modules[i].Azimuth_PID.Kd = 0.0f;
        Swerve.Modules[i].Azimuth_PID.Output_Max = 18000.0f;
				
				Swerve.Modules[i].Drive_PID.Kp = 2000.0f;
        Swerve.Modules[i].Drive_PID.Ki = 0.0f;
        Swerve.Modules[i].Drive_PID.Kd = 0.0f;
				Swerve.Modules[i].Drive_PID.I_Out_Max = 0.0f;
        Swerve.Modules[i].Drive_PID.Output_Max = 18000.0f;
				Swerve.Modules[i].Drive_Motor.Last_Total_Angle = 0;
        Swerve.Modules[i].Drive_CAN_ID = Drive_CAN_ID[i];
		PID_Func.Clear_PID_Data(&Swerve.Modules[i].Drive_PID);
		PID_Func.Clear_PID_Data(&Swerve.Modules[i].Azimuth_PID);
    }
	
}

/* Scale wheel speeds to max possible speed while preserving ratio between modules.*/
Module_State_Array_t Desaturate_Wheel_Speeds(Module_State_Array_t Module_State_Array) {
	float Highest_Speed = fabsf(Module_State_Array.States[0].Module_Speed);
	for (int i=0; i<NUMBER_OF_MODULES; i++) {
		if(fabsf(Module_State_Array.States[i].Module_Speed) > fabsf(Highest_Speed)) {
			Highest_Speed = Module_State_Array.States[i].Module_Speed;
		}
	}
	if (fabs(Highest_Speed) > 0.01f) {
		float Desaturation_Coefficient = fabs(SWERVE_MAX_SPEED / Highest_Speed);
		//1.4/(1/1.4)
		Module_State_Array_t Desaturated_Module_States;
	
		for (int i=0; i<4; i++) {
			Desaturated_Module_States.States[i].Module_Speed = Module_State_Array.States[i].Module_Speed * Desaturation_Coefficient;
					Desaturated_Module_States.States[i].Module_Angle = Module_State_Array.States[i].Module_Angle;
			}
			
		return Desaturated_Module_States;
	}
	return Module_State_Array;
}

/* Convert chassis speeds to module states using inverse kinematics */
Module_State_Array_t Chassis_Speeds_To_Module_States(Chassis_Speeds_t Chassis_Speeds) { 
	Module_State_Array_t Calculated_Module_States;
	if (Chassis_Speeds.X_Speed==0 && Chassis_Speeds.Y_Speed==0 && Chassis_Speeds.Omega==0) {
		for (int i=0; i<NUMBER_OF_MODULES; i++) {
			Calculated_Module_States.States[i].Module_Speed = 0; 
			Calculated_Module_States.States[i].Module_Angle = last_swerve_angle[i];
		}
	} else {
        float Chassis_Speeds_Vector[3][1] =
                {Chassis_Speeds.X_Speed, Chassis_Speeds.Y_Speed, Chassis_Speeds.Omega};

        float Module_States_Matrix[8][1];

        // Initialize matrix instances
        arm_matrix_instance_f32 Swerve_Inverse_Kinematics_instance, Chassis_Speeds_Vector_instance, Module_States_Matrix_instance;
        arm_mat_init_f32(&Swerve_Inverse_Kinematics_instance, 8, 3, &Swerve_Inverse_Kinematics[0][0]);
        arm_mat_init_f32(&Chassis_Speeds_Vector_instance, 3, 1, &Chassis_Speeds_Vector[0][0]);
        arm_mat_init_f32(&Module_States_Matrix_instance, 8, 1, &Module_States_Matrix[0][0]);
        
        // Multiply inverse kinematic matrix by chassis speed vector to get module x,y matrix
        if (arm_mat_mult_f32(&Swerve_Inverse_Kinematics_instance, &Chassis_Speeds_Vector_instance, &Module_States_Matrix_instance) == ARM_MATH_SUCCESS) {
            /* operation success */
            // Convert module x,y matrix to wheel speed and angle
            for (int i = 0; i < NUMBER_OF_MODULES; i++) {
                float x = Module_States_Matrix[i*2][0];
                float y = Module_States_Matrix[i*2 + 1][0];

                float speed = hypotf(x, y);
                if (speed > 1e-6f) {
                    y /= speed;
                    x /= speed;
                    float angle = atan2f(x, y);

                    Calculated_Module_States.States[i].Module_Angle = angle;
									last_swerve_angle[i] = angle;
                } else {
                    x = 0.0f;
                    y = 1.0f;
                }
//                float angle = atan2f(m_sin, m_cos);
		
								Calculated_Module_States.States[i].Module_Speed = speed;
								Calculated_Module_States.States[i].Module_Angle = last_swerve_angle[i];
								
//                Calculated_Module_States.States[i].Module_Angle = angle;
            }
        } else {
            /* operation failed */
            // Do nothing
        }
	}
	return Calculated_Module_States;
}

/* Set the desired modules state of each module */
void Set_Desired_States(Module_State_Array_t Desired_States) {
    Desired_States = Desaturate_Wheel_Speeds(Desired_States);
    for (int i=0; i<NUMBER_OF_MODULES; i++) {
        Swerve.Modules[i].Module_State = Desired_States.States[i];
    }
}

/* Takes driver input (-1 to 1) and sets respective module outputs */
void drive(Swerve_t *Swerve, float x, float y, float omega) {
    x *= SWERVE_MAX_SPEED; //convert to m/s
    y *= SWERVE_MAX_SPEED;
    //theta *= SWERVE_MAX_ANGLUAR_SPEED; //convert to rad/s
	Chassis_Speeds_t Desired_Chassis_Speeds = {.X_Speed = x, .Y_Speed = y, .Omega = omega};

    Set_Desired_States(Chassis_Speeds_To_Module_States(Desired_Chassis_Speeds));

    for (int i=0; i <NUMBER_OF_MODULES; i++)
    {
        Set_Module_Output(&(Swerve->Modules[i]), Swerve->Modules[i].Module_State);
    }
}

/* Commands modules to stop moving and reset angle to 0. Should be called on robot enable */
void Reset_Modules() {
    Module_State_t Reset_State = {.Module_Speed = 0, .Module_Angle = 0};
    Module_State_Array_t Desired_States = {Reset_State, Reset_State, Reset_State, Reset_State};

    Set_Desired_States(Desired_States);
}


void Swerve_Processing(Swerve_t *Swerve) { // TODO
    switch (Swerve->Current_Mode) {
        case (Follow_Gimbal): {

        }
    }

    drive(Swerve,
		DR16_Export_Data.Remote_Control.Joystick_Left_Vx / 660.0f,
            DR16_Export_Data.Remote_Control.Joystick_Left_Vy / 660.0f,
            -DR16_Export_Data.Remote_Control.Joystick_Right_Vx / 660.0f);
}
