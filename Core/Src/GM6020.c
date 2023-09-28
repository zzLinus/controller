#include "tim.h"
#include "main.h"

void Gimbal_Rotates(int Degree)
{
  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,Degree);
}