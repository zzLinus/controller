/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include <stdio.h>
#include "usart.h"
#include "com.h"
#include "NX_Calculate.h"
#include "stm32f4xx_it.h"
#include "oled.h"
#include "oledfont.h"
#include "GM6020.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId ChassisTaskHandle;
osThreadId PIDTaskHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void Chassis_Task(void const * argument);
void PID_Task(void const * argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

//  osThreadDef(PID, PID_Task,osPriorityNormal,0,512);
//  PIDTaskHandle = osThreadCreate(osThread(PID),NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//  osThreadDef(Chassis, Chassis_Task, osPriorityNormal, 0, 1024);
//  ChassisTaskHandle = osThreadCreate(osThread(Chassis),NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
   while(1)
   {
       osDelay(10);
   }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void Chassis_Task(void const * argument)
{
    //Chassis_Init();
    while(1)
    {
//      if(Cmd_Order == 0x81)//开始复位
//      {
//        Reset();
//      }
//      else if(Cmd_Order == 0x82)//开始边线旋转
//      {
//        HAL_UART_Transmit(&huart6,&Side_Rotation[0],sizeof(Side_Rotation),0xff);
//      }
//      else if(Cmd_Order == 0x84)//开始胶带旋转
//      {
//        HAL_UART_Transmit(&huart6,&Side_Rotation[0],sizeof(Tape_Rotation),0xff);
//      }

      osDelay(10);
    }
}

void PID_Task(void const * argument)
{
  CV_Init();
  while(1)
  {
    if (PID_Flag == Position_Flag)
    {
      Position_Pid[0].target = (float)Demension_X;
      Position_Pid[0].f_cal_pid(&Position_Pid[0],moto_chassis[0].angle);
      Position_Pid[1].target = (float)Demension_Y;
      Position_Pid[1].f_cal_pid(&Position_Pid[1],moto_chassis[1].angle);

      Speed_Pid[0].target = Position_Pid[0].output;
      Speed_Pid[0].f_cal_pid(&Speed_Pid[0], moto_chassis[0].speed_rpm);
      Speed_Pid[1].target = Position_Pid[1].output;
      Speed_Pid[1].f_cal_pid(&Speed_Pid[1], moto_chassis[1].speed_rpm);
    }
    else if (PID_Flag == CV_Flag)
    {
      CV_TRACK_X.target = 0;
      CV_TRACK_X.f_cal_pid(&CV_TRACK_X, (float) Receive_X);
      CV_TRACK_Y.target = 0;
      CV_TRACK_Y.f_cal_pid(&CV_TRACK_Y, (float) Receive_Y);
      if (Receive_X == 0)
      {
        CV_TRACK_X.output = 0;
      }
      if(Receive_Y == 0)
      {
        CV_TRACK_Y.output = 0;
      }
      Speed_Pid[0].target = CV_TRACK_X.output;
      //Speed_Pid[0].target = 1;
      Speed_Pid[0].f_cal_pid(&Speed_Pid[0], moto_chassis[0].speed_rpm);
      //Speed_Pid[1].target = 1;
      Speed_Pid[1].target = CV_TRACK_Y.output;
      Speed_Pid[1].f_cal_pid(&Speed_Pid[1], moto_chassis[1].speed_rpm);
      cprintf(&huart1,"CV_TRACK_X.output = %d\n",moto_chassis[1].speed_rpm);
      cprintf(&huart1,"CV_TRACK_Y.output = %f\n",CV_TRACK_Y.output);
    }
    else if(PID_Flag == Test_Flag)
    {
      Speed_Pid[0].target = 100;
      Speed_Pid[1].target = 0;
      Speed_Pid[0].f_cal_pid(&Speed_Pid[0], moto_chassis[0].speed_rpm);
      Speed_Pid[1].f_cal_pid(&Speed_Pid[1], moto_chassis[1].speed_rpm);
      cprintf(&huart6,"moto1.speed_rpm = %d\n",moto_chassis[1].speed_rpm);
    }
    set_moto_current(&hcan1, 0, Speed_Pid[1].output,Speed_Pid[2].output,Speed_Pid[3].output);
    osDelay(5);
  }
}
/* USER CODE END Application */
