/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      detect error task, judged by receiving data time. provide detect
                hook function, error exist function.
  *             检测错误任务， 通过接收数据时间来判断.提供 检测钩子函数,错误存在函数.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *
  @verbatim
  ==============================================================================
    add a sensor 
    1. in detect_task.h, add the sensor name at the end of errorList,like
    enum errorList
    {
        ...
        XXX_TOE,    //new sensor
        ERROR_LIST_LENGHT,
    };
    2.in detect_init function, add the offlineTime, onlinetime, priority params,like
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3. if XXX_TOE has data_is_error_fun ,solve_lost_fun,solve_data_error_fun function, 
        please assign to function pointer.
    4. when XXX_TOE sensor data come, add the function detect_hook(XXX_TOE) function.
    如果要添加一个新设备
    1.第一步在detect_task.h，添加设备名字在errorList的最后，像
    enum errorList
    {
        ...
        XXX_TOE,    //新设备
        ERROR_LIST_LENGHT,
    };
    2.在detect_init函数,添加offlineTime, onlinetime, priority参数
        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };
    3.如果有data_is_error_fun ,solve_lost_fun,solve_data_error_fun函数，赋值到函数指针
    4.在XXX_TOE设备数据来的时候, 添加函数detect_hook(XXX_TOE).
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#include "detect_task.h"
#include "remote_control.h"
#include "cmsis_os.h"


/**
  * @brief          init error_list, assign  offline_time, online_time, priority.
  * @param[in]      time: system time
  * @retval         none
  */
/**
  * @brief          初始化error_list,赋值 offline_time, online_time, priority
  * @param[in]      time:系统时间
  * @retval         none
  */
static void detect_init(uint32_t time);




error_t error_list[ERROR_LIST_LENGHT + 1];

extern void buzzer_on(uint16_t psc, uint16_t pwm);
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif


/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          检测任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
	
uint16_t running_psc = 2000, running_pwm = 0;
uint16_t running_psc_factor = 1, running_pwm_factor = 1;
uint16_t buzzer_factor1 = 178, buzzer_factor2 = 1;

extern uint8_t door_open;
extern uint8_t arm_mode;
extern uint8_t ctrl_mode;
extern fp32 battery_voltage;

const uint16_t music[3][8]={    //低中高音对应频率所需的预分频数
    {0,381,340,303,286,255,227,202},
    {0,191,170,151,143,127,113,101},
    {0,95,85,75,71,63,56,50}
};



void play(uint8_t i,uint8_t j,uint16_t time, uint16_t idle){
	int16_t current_pwm = 485;
	while(time > 10){
		current_pwm = current_pwm - 4;
		if(current_pwm < 0){
			current_pwm = 0;
		}
		buzzer_on(music[i][j], current_pwm*current_pwm/485);
		time = time - 5;
		vTaskDelay(5);
	}
	buzzer_off();
	vTaskDelay(idle);
}

void play_fix(uint8_t i,uint8_t j,uint16_t time, uint16_t idle,int16_t pwm){
	int16_t current_pwm = 485;
	buzzer_on(music[i][j], pwm);
	vTaskDelay(time);
	buzzer_off();
	vTaskDelay(idle);
}

extern Sbus_ctrl_t Sbus_ctrl;
void detect_task(void const *pvParameters)
{
    static uint32_t system_time;
    system_time = xTaskGetTickCount();
    //init,初始化
    detect_init(system_time);
    //wait a time.空闲一段时间
    vTaskDelay(DETECT_TASK_INIT_TIME);
    
    vTaskDelay(2000);
    for(uint16_t warn_psc=230; warn_psc > 10; warn_psc = warn_psc - 2){
        buzzer_on(warn_psc, 5);
        vTaskDelay(5);
    }
    buzzer_off();
    vTaskDelay(200);
    play_fix(1, 1, 200, 10, 16);
    play_fix(1, 2, 170, 10, 16);
    play_fix(1, 5, 200, 10, 16);
    
    play_fix(1, 2, 170, 10, 8);
    play_fix(1, 5, 200, 10, 8);
    
    play_fix(1, 2, 170, 10, 4);
    play_fix(1, 5, 200, 10, 4);
    vTaskDelay(1000);
	
    while (1)
    {
        if(ctrl_mode != 2){
				//play(1, 7, 200, 10);
				//play(1, 1, 200, 10);
				//play(1, 7, 200, 10);
				//play(1, 1, 200, 200);
        }
        if(door_open){
            if(arm_mode == 0){
                for(uint16_t warn_psc=191; warn_psc > 101; warn_psc = warn_psc - 5){
                    //buzzer_on(warn_psc, 480);
                    vTaskDelay(20);
                    if(door_open == 0){
                        break;
                    }
                    if(arm_mode){
                        break;
                    }
                }
                buzzer_off();
                vTaskDelay(50);
                for(uint16_t warn_psc=191; warn_psc > 101; warn_psc = warn_psc - 5){
                    //buzzer_on(warn_psc, 480);
                    vTaskDelay(20);
                    if(door_open == 0){
                        break;
                    }
                    if(arm_mode){
                        break;
                    }
                }
                buzzer_off();
                for(uint8_t i =0; i< 10; i=i+1){
                    if(door_open == 0){
                        break;
                    }
                    if(arm_mode){
                        break;
                    }
                    vTaskDelay(100);
                }
            }
        }else{
            buzzer_off();
        }
        if(battery_voltage < 11.5f){
            //play(0, 1, 600, 200);
            //play(0, 1, 600, 200);
            //play(0, 1, 600, 200);
        }
        

        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
/**
  * @brief          获取设备对应的错误状态
  * @param[in]      toe:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}

/**
  * @brief          record the time
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();
    
    if (error_list[toe].is_lost)
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }
    
    if (error_list[toe].data_is_error_fun != NULL)
    {
        if (error_list[toe].data_is_error_fun())
        {
            error_list[toe].error_exist = 1;
            error_list[toe].data_is_error = 1;

            if (error_list[toe].solve_data_error_fun != NULL)
            {
                error_list[toe].solve_data_error_fun();
            }
        }
        else
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else
    {
        error_list[toe].data_is_error = 0;
    }
}

/**
  * @brief          get error list
  * @param[in]      none
  * @retval         the point of error_list
  */
/**
  * @brief          得到错误列表
  * @param[in]      none
  * @retval         error_list的指针
  */
const error_t *get_error_list_point(void)
{
    return error_list;
}

extern void OLED_com_reset(void);
static void detect_init(uint32_t time)
{
    //设置离线时间，上线稳定工作时间，优先级 offlineTime onlinetime priority
    uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            {30, 40, 15},   //SBUS
            {10, 10, 11},   //motor1
            {10, 10, 10},   //motor2
            {10, 10, 9},    //motor3
            {10, 10, 8},    //motor4
            {2, 3, 14},     //yaw
            {2, 3, 13},     //pitch
            {10, 10, 12},   //trigger
            {2, 3, 7},      //board gyro
            {5, 5, 7},      //board accel
            {40, 200, 7},   //board mag
            {100, 100, 5},  //referee
            {10, 10, 7},    //rm imu
            {100, 100, 1},  //oled
        };

    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].data_is_error_fun = NULL;
        error_list[i].solve_lost_fun = NULL;
        error_list[i].solve_data_error_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].error_exist = 1;
        error_list[i].is_lost = 1;
        error_list[i].data_is_error = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }

    error_list[OLED_TOE].data_is_error_fun = NULL;
    error_list[OLED_TOE].solve_lost_fun = OLED_com_reset;
    error_list[OLED_TOE].solve_data_error_fun = NULL;

//    error_list[DBUSTOE].dataIsErrorFun = RC_data_is_error;
//    error_list[DBUSTOE].solveLostFun = slove_RC_lost;
//    error_list[DBUSTOE].solveDataErrorFun = slove_data_error;

}
