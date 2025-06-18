/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       servo_task.c/h
  * @brief      
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-21-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

#define SERVO_MIN_PWM   1000
#define SERVO_MAX_PWM   2000

#define PWM_DETAL_VALUE 10

#define SERVO1_ADD_PWM_KEY  KEY_PRESSED_OFFSET_Z
#define SERVO2_ADD_PWM_KEY  KEY_PRESSED_OFFSET_X
#define SERVO3_ADD_PWM_KEY  KEY_PRESSED_OFFSET_C
#define SERVO4_ADD_PWM_KEY  KEY_PRESSED_OFFSET_V

#define SERVO_MINUS_PWM_KEY KEY_PRESSED_OFFSET_SHIFT

const RC_ctrl_t *servo_rc;

const static uint16_t servo_key[4] = {SERVO1_ADD_PWM_KEY, SERVO2_ADD_PWM_KEY, SERVO3_ADD_PWM_KEY, SERVO4_ADD_PWM_KEY};

uint16_t servo_pwm[6] = {SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM, SERVO_MIN_PWM};
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          ¶æ»úÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */

void set_pwm(uint16_t s1, uint16_t s2, uint16_t s3, uint16_t s4){
	servo_pwm[0] = (uint16_t)s1;
	servo_pwm[1] = (uint16_t)s2;
	servo_pwm[2] = (uint16_t)s3;
	servo_pwm[3] = (uint16_t)s4;
}

extern fp32 throttle_out, roll_out, pitch_out, yaw_out;
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
	  servo_pwm[0] = 1000;
	  servo_pwm[1] = 1000;
	  servo_pwm[2] = 1000;
	  servo_pwm[3] = 1000;
	  servo_pwm[4] = 1500;
    while(1)
    {
				for(uint8_t i = 0; i < 6; i=i+1)
        {
						if(i==4){
							servo_pwm_set(servo_pwm[i], i);
							continue;
						}
            if(servo_pwm[i] < SERVO_MIN_PWM)
            {
                servo_pwm[i] = SERVO_MIN_PWM;
            }
            else if(servo_pwm[i] > SERVO_MAX_PWM)
            {
                servo_pwm[i] = SERVO_MAX_PWM;
            }
						servo_pwm_set(servo_pwm[i], i);
        }
        osDelay(5);
    }
}


