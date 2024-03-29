/**
 * @file GM6020_Motor.c
 * @author Leo Liu
 * @brief GM6020 communication
 * @version 1.0
 * @date 2022-07-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "GM6020_Motor.h"
#include <stdio.h>

Motor_Init_t GM6020_Yaw;
Motor_Init_t GM6020_Pitch;

void GM6020_Get_Data(Motor_Init_t *motor_init, CAN_Export_Data_t RxMessage);
void GM6020_Send_Current_Group1(CAN_HandleTypeDef *hcanx, uint16_t motor_1, uint16_t motor_2, uint16_t motor_3, uint16_t motor_4);
void GM6020_Yaw_Get_Data(CAN_Export_Data_t RxMessage);
void GM6020_Pitch_Get_Data(CAN_Export_Data_t RxMessage);
void GM6020_Gimbal_Send_Data(int16_t Pitch_Output, int16_t Yaw_Output);
void Check_GM6020_Yaw(void);
void Check_GM6020_Pitch(void);

GM6020_Func_t GM6020_Func = GM6020_Func_GroundInit;
#undef GM6020_Func_GroundInit

void GM6020_Get_Data(Motor_Init_t *motor_init, CAN_Export_Data_t RxMessage)
{
	motor_init->Prev_Angle = motor_init->Actual_Angle;
	motor_init->Actual_Angle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
	motor_init->Actual_Speed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
	motor_init->Actual_Current = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
	motor_init->Temperature = RxMessage.CANx_Export_RxMessage[6];

	if ((motor_init->Actual_Angle - motor_init->Prev_Angle) < -6500)
		motor_init->Turn_Count++;
	else if ((motor_init->Actual_Angle - motor_init->Prev_Angle) > 6500)
		motor_init->Turn_Count--;

    motor_init->Total_Angle = motor_init->Actual_Angle + (GM6020_MECH_ANGLE_MAX * motor_init->Turn_Count);
	
    //motor_init->Total_Angle += (motor_init->Actual_Angle - motor_init->Prev_Angle);//Total_Angle is the accumulated angle in motor ticks
	motor_init->Angle_Rad = -(motor_init->Total_Angle - motor_init->Angle_Offset)/GM6020_MECH_ANGLE_MAX * 2 * 3.1415927f + 3.1415927f/2;
    motor_init->Info_Update_Frame++;
}

/* CAN ID 1-4 */
void GM6020_Send_Current_Group1(CAN_HandleTypeDef *hcanx, uint16_t motor_1, uint16_t motor_2, uint16_t motor_3, uint16_t motor_4)
{
	CAN_Func.CAN_0x1FF_Send_Data(hcanx, motor_1, motor_2, motor_3, motor_4);
}

//Get yaw motor data from CAN
void GM6020_Yaw_Get_Data(CAN_Export_Data_t RxMessage)
{
	GM6020_Yaw.Prev_Angle = GM6020_Yaw.Actual_Angle;
	GM6020_Yaw.Actual_Angle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
  GM6020_Yaw.Actual_Speed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
  GM6020_Yaw.Actual_Current = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
  GM6020_Yaw.Temperature = RxMessage.CANx_Export_RxMessage[6];
	if((GM6020_Yaw.Actual_Angle - GM6020_Yaw.Prev_Angle) < -6500 )
		GM6020_Yaw.Turn_Count++;
	else if((GM6020_Yaw.Actual_Angle - GM6020_Yaw.Prev_Angle) > 6500)
		GM6020_Yaw.Turn_Count--;
	GM6020_Yaw.Total_Angle = GM6020_Yaw.Actual_Angle + (GM6020_MECH_ANGLE_MAX * GM6020_Yaw.Turn_Count);
	GM6020_Yaw.Info_Update_Frame++;
}

//Get pitch motor data from CAN
void GM6020_Pitch_Get_Data(CAN_Export_Data_t RxMessage)
{
	GM6020_Pitch.Prev_Angle = GM6020_Pitch.Actual_Angle;
	GM6020_Pitch.Actual_Angle = (int16_t)(RxMessage.CANx_Export_RxMessage[0] << 8 | RxMessage.CANx_Export_RxMessage[1]);
  GM6020_Pitch.Actual_Speed = (int16_t)(RxMessage.CANx_Export_RxMessage[2] << 8 | RxMessage.CANx_Export_RxMessage[3]);
  GM6020_Pitch.Actual_Current = (int16_t)(RxMessage.CANx_Export_RxMessage[4] << 8 | RxMessage.CANx_Export_RxMessage[5]);
  GM6020_Pitch.Temperature = RxMessage.CANx_Export_RxMessage[6];
	if((GM6020_Pitch.Actual_Angle - GM6020_Pitch.Prev_Angle) < -6500 )
		GM6020_Pitch.Turn_Count++;
	else if((GM6020_Pitch.Actual_Angle - GM6020_Pitch.Prev_Angle) > 6500)
		GM6020_Pitch.Turn_Count--;
	GM6020_Pitch.Total_Angle = GM6020_Pitch.Actual_Angle + (GM6020_MECH_ANGLE_MAX * GM6020_Pitch.Turn_Count);
	GM6020_Pitch.Info_Update_Frame++;
}

//Send gimbal data through specified identifier
void GM6020_Gimbal_Send_Data(int16_t Pitch_Output,int16_t Yaw_Output)
{
	CAN_Func.CAN_0x2FF_Send_Data(&hcan1,0,Pitch_Output,Yaw_Output,0);
}


void Check_GM6020_Yaw(void)
{
	if(GM6020_Yaw.Info_Update_Frame < 1)
		GM6020_Yaw.Offline_Flag = 1;
	else
		GM6020_Yaw.Offline_Flag = 0;
	GM6020_Yaw.Info_Update_Frame = 0;
}

void Check_GM6020_Pitch(void)
{
	if(GM6020_Pitch.Info_Update_Frame < 1)
		GM6020_Pitch.Offline_Flag = 1;
	else
		GM6020_Pitch.Offline_Flag = 0;
	GM6020_Pitch.Info_Update_Frame = 0;
}
