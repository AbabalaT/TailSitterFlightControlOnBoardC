/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       usb_task.c/h
  * @brief      usb outputs the error message.usb ‰≥ˆ¥ÌŒÛ–≈œ¢
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
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

#include "detect_task.h"
#include "voltage_task.h"

#include "CRC8_CRC16.h"

static void usb_printf(const char *fmt,...);

uint8_t usb_buf1[256];
static const char status[2][7] = {"OK", "ERROR!"};
const error_t *error_list_usb_local;

extern fp32 angle_data[3];
extern fp32 ahrs_quaternion[4];

void usb_task(void const * argument)
{
    MX_USB_DEVICE_Init();
    error_list_usb_local = get_error_list_point();
		osDelay(1000);

    while(1)
    {
				usb_buf1[0] = 0xff;
				usb_buf1[1] = 17;
				usb_buf1[2] = 0x14;
			
				usb_buf1[3] = 0x00;//??flag
				*(fp32*)(&usb_buf1[4]) = angle_data[1] * -180.0 / 3.14159265359;
				*(fp32*)(&usb_buf1[8]) = angle_data[2] * -180.0 / 3.14159265359;
				*(fp32*)(&usb_buf1[12]) = angle_data[0] * 180.0 / 3.14159265359;
			
				usb_buf1[16] = get_CRC8_check_sum(usb_buf1, 16, 0xFF);
				CDC_Transmit_FS(usb_buf1, 17);
        osDelay(2);
				
				usb_buf1[0] = 0xff;
				usb_buf1[1] = 21;
				usb_buf1[2] = 0x13;
				usb_buf1[3] = 0x00;
				*(fp32*)(&usb_buf1[4]) = ahrs_quaternion[0];
				*(fp32*)(&usb_buf1[8]) = ahrs_quaternion[1];
				*(fp32*)(&usb_buf1[12]) = ahrs_quaternion[2];
				*(fp32*)(&usb_buf1[16]) = ahrs_quaternion[3];

				usb_buf1[20] = get_CRC8_check_sum(usb_buf1, 20, 0xFF);
				CDC_Transmit_FS(usb_buf1, 21);
				osDelay(3);
		}
}

static void usb_printf(const char *fmt,...)
{
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf1, fmt, ap);

    va_end(ap);


    CDC_Transmit_FS(usb_buf1, len);
}
