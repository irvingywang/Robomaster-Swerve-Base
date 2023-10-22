#include "Swerve.h"
#include "Swerve_Module.h"

/* Declare swerve struct */
Swerve_t Swerve;

bool Azimuth_Encoder_Reversed_Array[4] = {true, true, true, true};
int Azimuth_CAN_ID[4] = {0x205, 0x206, 0x207, 0x208};
float Azimuth_Encoder_Zero_Offset[4] = {5470.0f, 1256.0f, 5465.0f, 7444.0f}; // encoder ticks

float Swerve_Inverse_Kinematics[8][3] = {
        { 1, 0, -(WHEEL_BASE/2)}, //front left 1
        {0, 1, +(-TRACK_WIDTH/2)},
        {1, 0 ,-(WHEEL_BASE/2)}, //front right 2
        {0, 1, +(TRACK_WIDTH/2)},
        {1, 0 ,-(-WHEEL_BASE/2)}, //back left 3
        {0, 1, +(-TRACK_WIDTH/2)},
        {1, 0 ,-(-WHEEL_BASE/2)}, //back right 4
        {0, 1, +(TRACK_WIDTH/2)}};


void Init_Modules(void);
void Swerve_Processing(Swerve_t *Swerve);

/* Initialize physical constants of each module*/
void Init_Modules() {
	for (int i=0; i<4; i++) {
		Swerve.Modules[i].Azimuth_Encoder_Reversed = Azimuth_Encoder_Reversed_Array[i];
        Swerve.Modules[i].Azimuth_CAN_ID = Azimuth_CAN_ID[i];
        Swerve.Modules[i].Azimuth_Encoder_Zero_Offset = Azimuth_Encoder_Zero_Offset[i];
        Swerve.Modules[i].Azimuth_PID.Kp = 10000.0f;
        Swerve.Modules[i].Azimuth_PID.Ki = 0.0f;
        Swerve.Modules[i].Azimuth_PID.Kd = 0.0f;
        Swerve.Modules[i].Azimuth_PID.Output_Max = 18000.0f;
    }
    
}

/* Scale wheel speeds to max possible speed while preserving ratio between modules.*/
Module_State_Array_t Desaturate_Wheel_Speeds(Module_State_Array_t Module_State_Array) {
	float Highest_Speed = fabsf(Module_State_Array.States[0].Module_Speed);
	for (int i=0; i<4; i++) {
		if(fabsf(Module_State_Array.States[i].Module_Speed) > fabsf(Highest_Speed)) {
			Highest_Speed = Module_State_Array.States[i].Module_Speed;
		}
        float num = fmaxf(1,2);
	}
	float Desaturation_Coefficient = fabsf(SWERVE_MAX_SPEED / Highest_Speed);
	//1.4/(1/1.4)
	Module_State_Array_t Desaturated_Module_States;
	
	for (int i=0; i<4; i++) {
		Desaturated_Module_States.States[i].Module_Speed = Module_State_Array.States[i].Module_Speed * Desaturation_Coefficient;
        Desaturated_Module_States.States[i].Module_Angle = Module_State_Array.States[i].Module_Angle;
    }
	
	return Desaturated_Module_States;
}

/* Inverse swerve kinematics*/
Module_State_Array_t Chassis_Speeds_To_Module_States(Chassis_Speeds_t Chassis_Speeds) { 
	Module_State_Array_t Calculated_Module_States;
	if (Chassis_Speeds.X_Speed == 0 && Chassis_Speeds.Y_Speed == 0 && Chassis_Speeds.Theta_Speed == 0) {
		for (int i=0; i<4; i++) {
			Calculated_Module_States.States[i].Module_Speed = 0; 
			Calculated_Module_States.States[i].Module_Angle = 0;
		}
	} else {//TODO calculate inverse kinematics via matrix (multiply by chassis speed vector)

        float Chassis_Speeds_Vector[3][1] =
                {Chassis_Speeds.X_Speed, Chassis_Speeds.Y_Speed, Chassis_Speeds.Theta_Speed};

        /* sample mat calc start */
        float Module_States_Matrix[8][1];
        // init arm math instance
        arm_matrix_instance_f32 Swerve_Inverse_Kinematics_instance, Chassis_Speeds_Vector_instance, Module_States_Matrix_instance;
        arm_mat_init_f32(&Swerve_Inverse_Kinematics_instance, 8, 3, &Swerve_Inverse_Kinematics[0][0]);
        arm_mat_init_f32(&Chassis_Speeds_Vector_instance, 3, 1, &Chassis_Speeds_Vector[0][0]);
        arm_mat_init_f32(&Module_States_Matrix_instance, 8, 1, &Module_States_Matrix[0][0]);
        
        // calc
        if (arm_mat_mult_f32(&Swerve_Inverse_Kinematics_instance, &Chassis_Speeds_Vector_instance, &Module_States_Matrix_instance) == ARM_MATH_SUCCESS) {
            float m_sin, m_cos;
            /* operation success */
            for (int i = 0; i < 4; i++) {
                float x = Module_States_Matrix[i*2][0];
                float y = Module_States_Matrix[i*2 + 1][0];

                float speed = hypotf(x, y);
                if (speed > 1e-6f) {
                    m_sin = y / speed;
                    m_cos = x / speed;
                    float angle = atan2f(m_sin, m_cos);

                    // Calculated_Module_States.States[i].Module_Speed = speed;
                    Calculated_Module_States.States[i].Module_Angle = angle;
                } 
                else {
                    m_sin = 0.0f;
                    m_cos = 1.0f;
                }
//                float angle = atan2f(m_sin, m_cos);

                Calculated_Module_States.States[i].Module_Speed = speed;
//                Calculated_Module_States.States[i].Module_Angle = angle;
            }
        } else {
            /* operation failed */
        }
        /* sample mat calc end */
	}
	return Calculated_Module_States;
}

/* Set the desired modules state of each module */
void Set_Desired_States(Module_State_Array_t Desired_States) {
    Desired_States = Desaturate_Wheel_Speeds(Desired_States);
    for (int i=0; i<4; i++) {
        Swerve.Modules[i].Module_State = Desired_States.States[i];
    }
}

/* Takes driver input (-1 to 1) and sets respective wheel speeds using inverse kinematics*/
void drive(float x, float y, float theta) {
    x *= SWERVE_MAX_SPEED; //convert to m/s
    y *= SWERVE_MAX_SPEED;
    theta *= SWERVE_MAX_ANGLUAR_SPEED; //convert to deg/s
	Chassis_Speeds_t Desired_Chassis_Speeds = {.X_Speed = x, .Y_Speed = y, .Theta_Speed = theta};

    Set_Desired_States(Chassis_Speeds_To_Module_States(Desired_Chassis_Speeds));
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

    drive(DR16_Export_Data.Remote_Control.Joystick_Left_Vy / 660.0f,
            DR16_Export_Data.Remote_Control.Joystick_Left_Vx / 660.0f,
            DR16_Export_Data.Remote_Control.Joystick_Right_Vx / 660.0f);
    
    for (int i = 0; i < 4; i++)
    {
        Set_Module_Output(&(Swerve->Modules[i]), Swerve->Modules[i].Module_State);
    }
}

/* Takes driver input and sets respective wheel speeds without kinematics (drives like a car)*/
/*
void drive() {
	Module_State_Array_t New_States;
	for (int i=0; i<4; i++) {
		New_States.States[i].Module_Speed = DR16_Export_Data.Remote_Control.Joystick_Left_Vx;;
		New_States.States[i].Module_Angle = DR16_Export_Data.Remote_Control.Joystick_Right_Vx;;
	}
	Set_Desired_States(Desaturate_Wheel_Speeds(New_States));
}*/

