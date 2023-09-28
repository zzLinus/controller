//此文件代码主要用于计算NX发来的信息
#include "NX_Calculate.h"
#include "pid.h"
#include "bsp_can.h"
#include "com.h"
#include "usart.h"

uint8_t High_Speed_Level = 0;
uint8_t Low_Speed_Level = 1;
uint8_t NX_Type = 0;
uint8_t OPEN_MV_Type = 1;
uint8_t PI_Type = 2;
uint8_t Position_Flag = 1;
uint8_t CV_Flag = 2;
uint8_t Test_Flag = 3;
uint8_t PID_Flag = 2;
uint8_t Side_Rotation[4] = {0x5A,0xA5,0x5C,0x82};
uint8_t Tape_Rotation[4] = {0x5A,0XA5,0X5C,0X84};


float Demension_X = 0;
float Demension_Y = 0;

PID_TypeDef CV_TRACK_X;
PID_TypeDef CV_TRACK_Y;
PID_TypeDef Speed_Pid[4];
PID_TypeDef Position_Pid[4];


//void Chassis_Init()
//{
//    pid_init(&Speed_Pid[0]);
//    Speed_Pid[0].f_param_init(&Speed_Pid[0], PID_Speed, 16384, 10000, 0, 0, 8000, 0, 100, 10, 5);
//    pid_init(&Position_Pid[0]);
//    Position_Pid[0].f_param_init(&Position_Pid[0], PID_Position, 8000, 10000, 0, 0, 9000, 0, 0.5, 0, 0);
//
//    pid_init(&Speed_Pid[1]);
//    Speed_Pid[1].f_param_init(&Speed_Pid[1], PID_Speed, 16384, 10000, 0, 0, 8000, 0, 15, 5, 1);
//    pid_init(&Position_Pid[1]);
//    Position_Pid[1].f_param_init(&Position_Pid[1], PID_Position, 8000, 10000, 0, 0, 9000, 0, 1, 0, 0);
//}

void Demension_Ctrl(float X,float Y)
{
  Demension_X = X / 360 * 8192;
  Demension_Y = Y / 360 * 8192;
}

void Position_Init()
{
  pid_init(&Speed_Pid[0]);
  Speed_Pid[0].f_param_init(&Speed_Pid[0], PID_Speed, 2500, 10000, 0, 0, 8000, 0, 100, 100, 5);
  pid_init(&Position_Pid[0]);
  Position_Pid[0].f_param_init(&Position_Pid[0], PID_Position, 2500, 10000, 0, 0, 9000, 0, 0.5, 0, 0);

  pid_init(&Speed_Pid[1]);
  Speed_Pid[1].f_param_init(&Speed_Pid[1], PID_Speed, 2500, 10000, 3, 0, 8000, 0, 15, 5, 1);
  pid_init(&Position_Pid[1]);
  Position_Pid[1].f_param_init(&Position_Pid[1], PID_Position, 25000, 10000, 3, 0, 9000, 0, 1, 0, 0);
}

void CV_Init()
{
  pid_init(&CV_TRACK_X);
  CV_TRACK_X.f_param_init(&CV_TRACK_X,CV,5000, 10000, 0, 0, 8000, 0,0.05, 0, 0);
  pid_init(&CV_TRACK_Y);
  CV_TRACK_Y.f_param_init(&CV_TRACK_Y,CV,5000, 3000, 0, 0, 3000, 0,3, 0, 1);

  pid_init(&Speed_Pid[0]);
  Speed_Pid[0].f_param_init(&Speed_Pid[0], PID_Speed, 5000, 10000, 0, 0, 8000, 0, 100, 100, 5);
  pid_init(&Speed_Pid[1]);
  Speed_Pid[1].f_param_init(&Speed_Pid[1], PID_Speed, 5000, 10000, 0, 0, 8000, 0, 15,6, 5);


}

void Reset()
{
  PID_Flag = Position_Flag;
  Position_Init();
  Demension_Ctrl(288,210);
}


//  Position_Pid[0].target = (float)Demension_X;
//  Position_Pid[0].f_cal_pid(&Position_Pid[0],moto_chassis[0].angle);
//  Position_Pid[1].target = (float)Demension_Y;
//  Position_Pid[1].f_cal_pid(&Position_Pid[1],moto_chassis[1].angle);
//
//  Speed_Pid[0].target = Position_Pid[0].output;
//  Speed_Pid[0].f_cal_pid(&Speed_Pid[0], moto_chassis[0].speed_rpm);
//  Speed_Pid[1].target = Position_Pid[1].output;
//  Speed_Pid[1].f_cal_pid(&Speed_Pid[1], moto_chassis[1].speed_rpm);

//void Chassis_Behavior(int CV_Y)
//{
//    CV_TRACK.target = 40;
//    CV_TRACK.f_cal_pid(&CV_TRACK,CV_Y);
//}
//
//void Speed_Init(int CV_Type,int Speed_Type)
//{
//    if (CV_Type == NX_Type)//此处为NX模式
//    {
//        if (Speed_Type == Low_Speed_Level)
//        {
//            CV_TRACK.f_param_init(&CV_TRACK,CV,5000,4000,5,0,320,0,2.9,0,0);//35,0.05,10
//        }
//        else if(Speed_Type == High_Speed_Level)
//        {
//            CV_TRACK.f_param_init(&CV_TRACK,CV,5000,4000,5,0,320,0,7,0,8);//35,0.05,10
//        }
//    }
//    else if(CV_Type == OPEN_MV_Type)//此处为OPEN_MV模式
//    {
//        if (Speed_Type == Low_Speed_Level)
//        {
//            CV_TRACK.f_param_init(&CV_TRACK,CV,5000,4000,3,0,8000,0,35,0.05,10);//35,0.05,10
//        }
//        else if(Speed_Type == High_Speed_Level)
//        {
//            CV_TRACK.f_param_init(&CV_TRACK,CV,5000,4000,3,0,8000,0,35,0.05,10);//35,0.05,10
//        }
//    }
//    else if(CV_Type == PI_Type)//此处为树莓派模式
//    {
//        if (Speed_Type == Low_Speed_Level)
//        {
//            CV_TRACK.f_param_init(&CV_TRACK,CV,4400,3500,10,0,8000,0,35,0.05,10);//35,0.05,10
//        }
//        else if(Speed_Type == High_Speed_Level)
//        {
//            CV_TRACK.f_param_init(&CV_TRACK,CV,4400,3500,10,0,8000,0,35,0.05,10);//35,0.05,10
//        }
//    }
//}



