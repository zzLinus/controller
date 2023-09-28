#ifndef NX_CALCULATE_H
#define NX_CALCULATE_H
#include "stdio.h"
#include "main.h"
#include "pid.h"
extern uint8_t Position_Flag;
extern uint8_t CV_Flag;
extern uint8_t PID_Flag;
extern uint8_t Test_Flag;
extern uint8_t High_Speed_Level;
extern uint8_t Low_Speed_Level;
extern PID_TypeDef Speed_Pid[4];
extern PID_TypeDef Position_Pid[4];
extern PID_TypeDef CV_TRACK_X;
extern PID_TypeDef CV_TRACK_Y;
extern float Demension_X;
extern float Demension_Y;
extern uint8_t Cmd_Order;

extern uint8_t Side_Rotation[4];
extern uint8_t Tape_Rotation[4];

void Reset();
void Chassis_Init(void);
void Speed_Init(int CV_Type,int Speed_Type);
void Demension_Ctrl(float X,float Y);
void Chassis_Behavior(int CV_Y);
void CV_Init();
#endif

