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

#include "can.h"
#include "bsp_can.h"
#include "com.h"
#include "usart.h"

#define printf(...) cprintf(&huart1,__VA_ARGS__)


Odrive_motor_measure motor[3] = {0};
int can_message_count = 0;


/*******************************************************************************************
  * @Func		my_can_filter_init
  * @Brief    CAN1和CAN2滤波器配置
  * @Param		CAN_HandleTypeDef* hcan
  * @Retval		None
  * @Date     2015/11/30
 *******************************************************************************************/
void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan)
{
	//can1 &can2 use same filter config
	CAN_FilterTypeDef		CAN_FilterConfigStructure;

//	CAN_FilterConfigStructure.FilterNumber = 0;
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
	CAN_FilterConfigStructure.FilterBank = 14;//can1(0-13)和can2(14-27)分别得到一半的filter
	CAN_FilterConfigStructure.FilterActivation = ENABLE;

	if(HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
	{
//		while(1); //show error!
	}
}

uint32_t FlashTimer;
/*******************************************************************************************
  * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
  * @Brief    HAL库中标准的CAN接收完成回调函数，需要在此处理通过CAN总线接收到的数据
  * @Param		
  * @Retval		None 
  * @Date     2015/11/24
 *******************************************************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
	CAN_RxHeaderTypeDef		Rx1Message;
	uint8_t Data[8];
	uint8_t MotorID;
	
	Rx1Message.StdId = 0x201;  
	Rx1Message.ExtId = 0;  
	Rx1Message.IDE = CAN_ID_STD;
	Rx1Message.RTR = CAN_RTR_DATA;
	Rx1Message.DLC = 0x08;

	if(HAL_GetTick() - FlashTimer>500)
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		FlashTimer = HAL_GetTick();
	}

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &Rx1Message, Data);

	//输出CAN接收到的所有信息
	// if(Rx1Message.StdId == (0x01<<5 | HEARTBEAT_MESSAGE_CMDID))
	// {
	// printf("ID:%x,count=%d \t\tData:%x, %x, %x, %x, %x, %x, %x, %x\n", Rx1Message.StdId, can_message_count, Data[0], 
	// 				Data[1], Data[2], Data[3], Data[4], Data[5], Data[6], Data[7]);
	// can_message_count++;
	// }


	/*接收CAN发送的数据 & 向电机发送目标扭矩*/
	switch (Rx1Message.StdId&0x1F)
	{
	case GET_ENCODER_COUNT_CMDID:
		motor[Rx1Message.StdId>>5].encoder = Data[5]<<8 | Data[4];
		Odrv_set_motor_torque(&hcan1, Rx1Message.StdId>>5, motor[Rx1Message.StdId>>5].traget_torque);
		break;
	case HEARTBEAT_MESSAGE_CMDID:
		motor[Rx1Message.StdId>>5].axis_err = Data[3]<<24 | Data[2]<<16 | Data[1]<<8 | Data[0];
		motor[Rx1Message.StdId>>5].axis_current_stage = Data[4];
		motor[Rx1Message.StdId>>5].motor_err_flag = Data[5];
		motor[Rx1Message.StdId>>5].encoder_err_flag = Data[6];

		/*在编码器错误的情况下尝试重新进闭环*/
		for(int i=0; i<3; i++)
		{
			if((motor[i].encoder_err_flag == 1))
			{
			//清除错误
			printf("encoder failed, id=%d!\n", i);
			Odrv_Clear_err(&hcan1, i);
			motor[i].state_resrt_count = 0;
			}
			else if((motor[i].axis_current_stage != 8) && (motor[i].axis_current_stage != 0) && motor[i].state_resrt_count <= 100)
			{
			//尝试重新进闭环
			printf("stage_err, id=%d, state=%d!\n", i, motor[i].axis_current_stage);
			Odrv_set_axis_state(&hcan1, i, 8);
			motor[i].state_resrt_count++;
			}
		}
		break;
	}
	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}


/*向Odrive发送CAN信息*/
uint8_t Odrv_CAN_Send_Msg(CAN_HandleTypeDef *hcan,uint16_t StdID,uint8_t *msg,uint8_t len,uint8_t Frame_type)
{
	CAN_TxHeaderTypeDef TxHeader;
    uint8_t index=0;
    uint32_t TxMailbox;   //邮箱
    uint8_t send_buf[8] = {0};
    TxHeader.StdId=StdID;        //标准标识符
    TxHeader.ExtId=0;        //扩展标识符(29位)
    TxHeader.IDE=CAN_ID_STD;    //使用标准帧
    TxHeader.RTR = Frame_type != CAN_RTR_REMOTE ? CAN_RTR_DATA : CAN_RTR_REMOTE;  //数据帧为发送数据，远程帧为请求返回数据
    //TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC=len;
    /*****填充消息******/
    for ( index = 0; index < len; index++) {
            send_buf[index] = msg[index];
    }
    /*****发送消息*****/
	// printf("SENT ID:%x, \t\tData:%x, %x, %x, %x, %x, %x, %x, %x\n", TxHeader.StdId, send_buf[0],
	// 		send_buf[1], send_buf[2], send_buf[3], send_buf[4], send_buf[5], send_buf[6], send_buf[7]);
    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, send_buf, &TxMailbox) != HAL_OK)//发送
    {
        return 1;
    }
    return 0;
}

/*设置电机扭矩*/
void Odrv_set_motor_torque(CAN_HandleTypeDef* hcan, int axis_id, float torque_set)
{
	uint8_t can_msg[8] = {0};
	unsigned int* float_bit_data = (unsigned int*)(&torque_set);
	can_msg[3] = (*float_bit_data>>24)&0xff;
	can_msg[2] = (*float_bit_data>>16)&0xff;
	can_msg[1] = (*float_bit_data>>8)&0xff;
	can_msg[0] = (*float_bit_data)&0xff;
	//printf("%d, %d, %d, %d\n", can_msg[0], can_msg[1], can_msg[2], can_msg[3]);
	Odrv_CAN_Send_Msg(&hcan, axis_id<<5 | 0x00E, can_msg, 8, CAN_RTR_DATA);
}

/*设置电机位置*/
void Odrv_set_motor_position(CAN_HandleTypeDef* hcan, int axis_id, float position_set, int16_t vel_lim, int16_t tor_lim)
{
	uint8_t can_msg[8] = {0};
	unsigned int* float_bit_data = (unsigned int*)(&position_set);
	can_msg[3] = (*float_bit_data>>24)&0xff;
	can_msg[2] = (*float_bit_data>>16)&0xff;
	can_msg[1] = (*float_bit_data>>8)&0xff;
	can_msg[0] = (*float_bit_data)&0xff;
	can_msg[5] = (vel_lim<<8)&0xff;
	can_msg[6] = (vel_lim)&0xff;
	can_msg[8] = (tor_lim<<8)&0xff;
	can_msg[7] = (tor_lim)&0xff;
	//printf("%d, %d, %d, %d\n", can_msg[0], can_msg[1], can_msg[2], can_msg[3]);
	Odrv_CAN_Send_Msg(&hcan, axis_id<<5 | 0x00C, can_msg, 8, CAN_RTR_DATA);
}

/*设置控制模式*/
void Odrv_set_motor_ControlMode(CAN_HandleTypeDef* hcan, int axis_id, int32_t control_mode, int32_t input_mode)
{
	uint8_t can_msg[8] = {0};

	can_msg[3] = (control_mode<<24)&0xff;
	can_msg[2] = (control_mode<<16)&0xff;
	can_msg[1] = (control_mode<<8)&0xff;
	can_msg[0] = (control_mode)&0xff;
	can_msg[7] = (input_mode<<24)&0xff;
	can_msg[6] = (input_mode<<16)&0xff;
	can_msg[5] = (input_mode<<8)&0xff;
	can_msg[4] = (input_mode)&0xff;
	//printf("%d, %d, %d, %d\n", can_msg[0], can_msg[1], can_msg[2], can_msg[3]);
	Odrv_CAN_Send_Msg(&hcan, axis_id<<5 | 0x00B, can_msg, 8, CAN_RTR_DATA);
}

/*清除错误*/
void Odrv_Clear_err(CAN_HandleTypeDef* hcan, int axis_id)
{
	uint8_t can_msg[8] = {0};
	Odrv_CAN_Send_Msg(&hcan, axis_id<<5 | 0x018, can_msg, 8, CAN_RTR_REMOTE);
}

/*设置控制状态*/
void Odrv_set_axis_state(CAN_HandleTypeDef* hcan, int axis_id, int32_t axis_state)
{
	uint8_t can_msg[8] = {0};

	can_msg[3] = (axis_state<<24)&0xff;
	can_msg[2] = (axis_state<<16)&0xff;
	can_msg[1] = (axis_state<<8)&0xff;
	can_msg[0] = (axis_state)&0xff;
	Odrv_CAN_Send_Msg(&hcan, axis_id<<5 | 0x007, can_msg, 8, CAN_RTR_DATA);
}
