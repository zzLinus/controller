/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#ifndef __BSP_CAN
#define __BSP_CAN

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#elif defined STM32F1
#include "stm32f1xx_hal.h"
#endif

#include "can.h"
typedef uint8_t 	u8;
typedef uint16_t 	u16;
typedef uint32_t 	u32;

typedef int8_t 		s8;
typedef int16_t 	s16;
typedef int32_t		s32;

typedef volatile uint8_t 	vu8;
typedef volatile uint16_t 	vu16;
typedef volatile uint32_t 	vu32;

typedef volatile int8_t 	vs8;
typedef volatile int16_t 	vs16;
typedef volatile int32_t	vs32;

#define HEARTBEAT_MESSAGE_CMDID 0x001 
#define GET_ENCODER_COUNT_CMDID 0X00A
#define GET_VBUS_VOLTAGE_CMDID  0x017

#define FILTER_BUF_LEN		5
/*?????????????????????*/
typedef struct{
	int16_t	 	speed_rpm;
  float  	real_current;
  int16_t  	given_current;
  uint8_t  	hall;
	uint16_t 	angle;				//abs angle range:[0,8191]
	uint16_t 	last_angle;	//abs angle range:[0,8191]
	uint16_t	offset_angle;
	int32_t		round_cnt;
	int32_t		total_angle;
	u8			buf_idx;
	u16			angle_buf[FILTER_BUF_LEN];
	u16			fited_angle;
	u32			msg_cnt;
}moto_measure_t;


typedef struct
{
	int16_t encoder;
	int16_t speed;
	float real_current;
	float vbus_vlotage;
	float traget_torque;
	/*×´Ì¬²âÊÔ*/
	int32_t axis_err;
	int8_t axis_current_stage;
	int8_t motor_err_flag;
	int8_t encoder_err_flag;
	int8_t state_resrt_count;
} Odrive_motor_measure;



extern moto_measure_t  moto_chassis[];
extern Odrive_motor_measure motor[];

#define LED_Pin GPIO_PIN_10
#define LED_GPIO_Port GPIOH

void my_can_filter_init(CAN_HandleTypeDef* hcan);
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);
void can_filter_recv_special(CAN_HandleTypeDef* hcan, uint8_t filter_number, uint16_t filtered_id);
void get_moto_measure(moto_measure_t *ptr, uint8_t Data[]);
void can_receive_onetime(CAN_HandleTypeDef* _hcan, int time);
void set_moto_current(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan);
uint8_t Odrv_CAN_Send_Msg(CAN_HandleTypeDef *hcan,uint16_t StdID,uint8_t *msg,uint8_t len,uint8_t Frame_type);
void Odrv_set_motor_torque(CAN_HandleTypeDef* hcan, int axis_id, float torque_set);
void Odrv_set_motor_position(CAN_HandleTypeDef* hcan, int axis_id, float position_set, int16_t vel_lim, int16_t tor_lim);
void Odrv_set_motor_ControlMode(CAN_HandleTypeDef* hcan, int axis_id, int32_t control_mode, int32_t input_mode);
void Odrv_Clear_err(CAN_HandleTypeDef* hcan, int axis_id);
void Odrv_set_axis_state(CAN_HandleTypeDef* hcan, int axis_id, int32_t axis_state);

#endif

