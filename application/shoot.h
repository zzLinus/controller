/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      射击功能。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"

// 摩擦轮速度
#define FRIC_UP 4850  //?? 1400  加速状态
#define FRIC_DOWN 4850  //?? 1320  开启状态
#define FRIC_OFF 0  //?? 1000  停止状态

//射击发射开关通道数据
#define SHOOT_RC_MODE_CHANNEL       1
//云台模式使用的开关通道

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    2500.0f	//原始100.0f

//射击摩擦轮激光打开 关闭
//#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
//#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//开关摩擦轮
#define FRIC_MODE_SWITCH_KEYBOARD         KEY_PRESSED_OFFSET_F

//切换点射、连射按键
#define SHOOT_MODE_SWITCH_KEYBOARD				KEY_PRESSED_OFFSET_B

//射击完成后 子弹弹出去后，判断时间，以防误触发
#define SHOOT_DONE_KEY_OFF_TIME     15
//鼠标长按判断
#define PRESS_LONG_TIME             400  //原始400		鼠标长按时间
//遥控器射击开关打下档一段时间后 连续发射子弹 用于清单
#define RC_S_LONG_TIME              300  //原始2000		遥控器长按时间
//摩擦轮高速 加速 时间
#define UP_ADD_TIME                 80
//电机反馈码盘值范围
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//电机rmp 变化成 旋转速度的比例
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  18
//拨弹速度
#define TRIGGER_SPEED               8.0f  	//原始10
#define CONTINUE_TRIGGER_SPEED      13.0f  	//原始13
#define READY_TRIGGER_SPEED         5.0f 		//未使用

#define TRIGGER_LONG_TIME						RC_S_LONG_TIME-5			//新增: 单次点射拨弹电机持续时间

#define KEY_OFF_JUGUE_TIME          500
// 检测送弹是否完成: 微动开关已按下,送弹完成
#define SWITCH_TRIGGER_ON           0			// 送弹检测微动开关ON: 设成0即可禁用此功能，即引脚电压一直为0，但无法从根本上禁用
// 检测送弹是否完成: 微动开关未按下,送弹未完成
#define SWITCH_TRIGGER_OFF          1			// 送弹检测微动开关OFF

//卡单时间 以及反转时间
#define BLOCK_TRIGGER_SPEED         1.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//拨弹轮电机PID
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   10000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f


#define SHOOT_HEAT_REMAIN_VALUE     40

typedef enum
{
    SHOOT_STOP = 0,						// 状态<摩擦轮停止>
    SHOOT_READY_FRIC,  				// 状态<摩擦轮开启>
    SHOOT_READY_BULLET,				// 状态<开始送弹>
    SHOOT_READY,							// 状态<可以射击>
    SHOOT_BULLET,							// 状态<开始射击-点射>
    SHOOT_CONTINUE_BULLET,		// 状态<开始射击-连射>
    SHOOT_DONE,								// 状态<射击完成>
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
    const motor_measure_t *shoot_motor_measure;
    ramp_function_source_t fric1_ramp;
    uint16_t fric_pwm1;
    ramp_function_source_t fric2_ramp;
    uint16_t fric_pwm2;
    pid_type_def trigger_motor_pid;
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;

    /* 2021.1.15添加 用于接收CV数据*/
    const can_CV_t *can_CV;
    /********************************/
} shoot_control_t;

//由于射击和云台使用同一个can的id故也射击任务在云台任务中执行
extern void shoot_init(void);
extern int16_t shoot_control_loop(gimbal_control_t *gimbal_shoot);

#endif
