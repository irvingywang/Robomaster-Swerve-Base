/**
 * @file Robot_Control.c
 * @author Leo Liu
 * @brief general robot control
 * @version 1.0
 * @date 2022-07-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "Robot_Control.h"
#include "Swerve.h"
#include "can.h"
void Robot_Control_Start(void); 
void Robot_Control_Send(void);
void Robot_Control_Disabled(void);

Robot_Control_Func_t Robot_Control_Func = Robot_Control_Func_GroundInit;
#undef Robot_Control_Func_GroundInit

Robot_Mode_t Robot_Mode;
int hello = 0;
//The function name tells you what it does
void Robot_Control_Start(void)
{
    if (hello == 0) {
        Init_Modules();
        hello++;
    }
    
//	State_Machine_Func.Remote_Control_Update(); 
//	Referee_System_Func.Referee_Set_Robot_State();
	
//	Tx2_Func.Jetson_Tx2_Get_Data();
//	Tx2_Func.Jetson_Tx2_Send_Data();
//	
//	Chassis_Func.Chassis_Speed_Get_Data(&Chassis);
//	Chassis_Func.Chassis_Processing(&Chassis);
//	
//	Gimbal_Func.Gimbal_Control_Get_Data(&Gimbal);
//	Gimbal_Func.Gimbal_Processing(&Gimbal);
//	
//	Shooting_Func.Trigger_Get_Data(&Shooting);
//	Shooting_Func.Shooting_Processing(&Shooting);
	Swerve_Processing(&Swerve);
	Robot_Control_Func.Robot_Control_Send();
}

void Robot_Control_Send(void)
{
    if (DR16_Export_Data.Remote_Control.Right_Switch == SWITCH_DOWN)
    {
        GM6020_Func.GM6020_Send_Current_Group1(&hcan1, 0, 0, 0, 0);
				M3508_Func.M3508_Send_Current_Group1(&hcan1, 0, 0, 0, 0);
    }
    else
    {
    GM6020_Func.GM6020_Send_Current_Group1(&hcan1, Swerve.Modules[0].Azimuth_Motor.Output_Current, Swerve.Modules[1].Azimuth_Motor.Output_Current,
                                            Swerve.Modules[2].Azimuth_Motor.Output_Current, Swerve.Modules[3].Azimuth_Motor.Output_Current);
    
			
			
		M3508_Func.M3508_Send_Current_Group1(&hcan1, Swerve.Modules[1].Drive_Motor.Output_Current, Swerve.Modules[3].Drive_Motor.Output_Current,
                                            Swerve.Modules[2].Drive_Motor.Output_Current, Swerve.Modules[0].Drive_Motor.Output_Current);

		}
    //	M3508_Func.M3508_Chassis_Send_Data(M3508_Chassis[0].Output_Current,M3508_Chassis[1].Output_Current,
//																			M3508_Chassis[2].Output_Current,M3508_Chassis[3].Output_Current);
//	M3508_Func.M3508_Fric_Wheel_Send_Data(M3508_Fric_Wheel[0].Output_Current,M3508_Fric_Wheel[1].Output_Current);
//	GM6020_Func.GM6020_Gimbal_Send_Data(GM6020_Pitch.Output_Current,GM6020_Yaw.Output_Current);
//	M2006_Func.M2006_Trigger_Send_Data(M2006_Trigger.Output_Current);
//	Super_Capacitor_Func.Super_Capacitor_Send_Data(Super_Capacitor.Target_Power);
}

void Robot_Control_Disabled(void)
{
//	Chassis.Current_Mode = Disabled;
//	Gimbal.Current_Mode = Disabled;
//	Shooting_Func.Shooting_Disabled();
}
