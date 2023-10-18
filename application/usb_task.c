/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb输出错误信息
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "usb_task.h"

#include "cmsis_os.h"

#include "usb_device.h"
/**#include "usbd_cdc_if.h"*/
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"
#include "usbd_customhid.h"



extern USBD_HandleTypeDef hUsbDeviceFS;
static void usb_printf(const char *fmt,...);

static uint8_t usb_buf[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;



void usb_task(void const * argument)  // !!!!!该任务已关闭!!!!!
{
    MX_USB_DEVICE_Init();


		uint8_t send_data = 0x00;
    while(1)
    {
        osDelay(10);
				send_data++;
				USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &send_data, 1);
    }

}

static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);


    /**CDC_Transmit_FS(usb_buf, len);*/
}


/* added by 片哥 */
/* 使用山外调试助手虚拟示波器: usb_debug("\x03\xFC%c%c%c\xFC\x03", data1, data2, data3); */
/*usb_debug("\x03\xFC%c%c%c%c\xFC\x03", (int8_t)(gimbal_control.fric1_motor.speed/20), (int8_t)(gimbal_control.fric1_motor.speed_set/20),
																							(int8_t)(gimbal_control.fric2_motor.speed/20), (int8_t)(gimbal_control.fric2_motor.speed_set/20));*/
#define USB_DEBUG_SEND_PERIOD 50
void usb_debug(const char *fmt,...)
{
    static uint32_t last_time = 0;
    uint32_t time = xTaskGetTickCount();
    if(time - last_time > USB_DEBUG_SEND_PERIOD)
    {
        static va_list ap;
        uint16_t len = 0;
        va_start(ap, fmt);
        len = vsprintf((char *)usb_buf, fmt, ap);
        va_end(ap);
        /**CDC_Transmit_FS(usb_buf, len);*/
        last_time = time;
    }
}

