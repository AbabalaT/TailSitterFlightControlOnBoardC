#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
#include "servo_task.h"
#include "chassis_power_control.h"

#define stick_heli 0x00
#define stick_3d 0xff

static void chassis_init(chassis_move_t *chassis_move_init);

typedef struct {
    float w, x, y, z;
} Quaternion;

fp32 ahrs_quaternion[4] = {1.0, 0.0, 0.0, 0.0};

int cali_cnt;
float cali_imu_num;

extern RC_ctrl_t rc_ctrl;
extern Sbus_ctrl_t Sbus_ctrl;
extern uint16_t servo_pwm[6];

fp32 gyro_data[3], angle_data[3];

uint8_t ctrl_mode = 0;
uint8_t ctrl_mode_stick = 0;
uint8_t ctrl_mode_allow_offboard = 0;

float fdata[16];

uint16_t motor_idle_speed = 1070;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

uint8_t arm_mode = 0;
uint8_t arm_mode_stick = 0;
uint8_t arm_mode_stick_pre = 0;

uint8_t system_mode = 0;
uint8_t door_open = 0;
uint8_t pre_door_open = 0;
int16_t door_open_idle = 0;

uint8_t stick_mode = 0x00;
float throttle_set = 0.0f;

float d_ch(uint8_t ch_required){
	return (Sbus_ctrl.ch[ch_required] - 1024.0f) * 1.49f;
}

extern float target_velocity[3];
extern void usart6_tx_dma_enable(uint8_t *data, uint16_t len);

float using_dp = 0.0f;
float hover_dp = 45.0f;

void limit_out(float* input){
	if(*input > 2200.0f){
		*input = 2200.0f;
	}
	if(*input < 800.0f){
		*input = 800.0f;
	}
}

float mat_pid[4][4];	//R-P-Y-throttle
float angle_pid_mat[3][3];

float safe_dp  = 25.0f;
float pid_safe_dp[3];

float u_real_roll = 0.0f;
float u_real_pitch = 0.0f;
float u_real_yaw = 0.0f;
float w_yaw_world[3];
float w_yaw_body[3];

uint16_t pwm_debugging = 0xFF;

// 定义滤波器结构体
typedef struct
{
    // 滤波器系数
    double b0, b1, b2;
    double a1, a2;

    // 滤波器状态
    double x1, x2; // 输入状态
    double y1, y2; // 输出状态
} ButterworthFilter;

void initButterworthFilter(ButterworthFilter* filter, double sampleRate, double cutoffFreq)
{
    double omega_c = 2.0 * PI * cutoffFreq / sampleRate;
    double alpha = sin(omega_c) / 2.0;

    // 计算滤波器系数
    double b0 = (1 - cos(omega_c)) / 2;
    double b1 = 1 - cos(omega_c);
    double b2 = (1 - cos(omega_c)) / 2;
    double a0 = 1 + alpha;
    double a1 = -2 * cos(omega_c);
    double a2 = 1 - alpha;

    // 归一化系数
    filter->b0 = b0 / a0;
    filter->b1 = b1 / a0;
    filter->b2 = b2 / a0;
    filter->a1 = a1 / a0;
    filter->a2 = a2 / a0;

    // 初始化状态
    filter->x1 = 0;
    filter->x2 = 0;
    filter->y1 = 0;
    filter->y2 = 0;
}

// 应用滤波器
double applyButterworthFilter(ButterworthFilter* filter, double input)
{
    // 计算输出
    double output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 -
        filter->a2 * filter->y2;
    // 更新状态
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    return output;
}

ButterworthFilter omega_x_filter, omega_y_filter, omega_z_filter;


//void pid_set_empty(void){
//	mat_pid[0][0] = 0.0;
//	mat_pid[0][1] = 120.0f;//232.55f;
//	mat_pid[0][2] = 0.02;
//	mat_pid[0][3] = 4.5;
//	
//	mat_pid[1][0] = 0.0;
//	mat_pid[1][1] = 57.0f;//697.6f;
//	mat_pid[1][2] = 0.078;
//	mat_pid[1][3] = 8.0;
//	
//	mat_pid[2][0] = 0.0;
//	mat_pid[2][1] = 1400.0f;//139.53f;
//	mat_pid[2][2] = 0.035f;//0.24f;
//	mat_pid[2][3] = 32.0;
//	
//	angle_pid_mat[0][0] = 1.3;
//	angle_pid_mat[0][1] = 0.0f;//0.00006;//232.55f;
//	angle_pid_mat[0][2] = 0.2f;
//	
//	angle_pid_mat[1][0] = 0.7;
//	angle_pid_mat[1][1] = 0.0f;//0.00002f;//697.6f;
//	angle_pid_mat[1][2] = 1.4f;
//	
//	angle_pid_mat[2][0] = 1.3;
//	angle_pid_mat[2][1] = 0.0f;//0.000045f;//139.53f;
//	angle_pid_mat[2][2] = 0.2f;
//	
//	hover_dp= 45.0f;
//}

//void pid_set_light(void){
//	mat_pid[0][0] = 0.0;
//	mat_pid[0][1] = 120.0f;//232.55f;
//	mat_pid[0][2] = 0.02;
//	mat_pid[0][3] = 4.5;
//	
//	mat_pid[1][0] = 0.0;
//	mat_pid[1][1] = 57.0f;//697.6f;
//	mat_pid[1][2] = 0.078;
//	mat_pid[1][3] = 8.0;
//	
//	mat_pid[2][0] = 0.0;
//	mat_pid[2][1] = 1400.0f;//139.53f;
//	mat_pid[2][2] = 0.035f;//0.24f;
//	mat_pid[2][3] = 32.0;
//	
//	angle_pid_mat[0][0] = 1.3;
//	angle_pid_mat[0][1] = 0.0f;//0.00006;//232.55f;
//	angle_pid_mat[0][2] = 0.2f;
//	
//	angle_pid_mat[1][0] = 0.7;
//	angle_pid_mat[1][1] = 0.0f;//0.00002f;//697.6f;
//	angle_pid_mat[1][2] = 1.4f;
//	
//	angle_pid_mat[2][0] = 1.3;
//	angle_pid_mat[2][1] = 0.0f;//0.000045f;//139.53f;
//	angle_pid_mat[2][2] = 0.2f;
//	
//	hover_dp= 45.0f;
//}

void pid_set_heavy(void){
	mat_pid[0][0] = 0.0;
	mat_pid[0][1] = 100.0f;
	mat_pid[0][2] = 0.0;
	mat_pid[0][3] = 0.0;
	
	mat_pid[1][0] = 0.0;
	mat_pid[1][1] = 100.0f;
	mat_pid[1][2] = 0.0;
	mat_pid[1][3] = 0.0;
	
	mat_pid[2][0] = 0.0;
	mat_pid[2][1] = 100.0f;
	mat_pid[2][2] = 0.0f;
	mat_pid[2][3] = 0.0f;
	
	angle_pid_mat[0][0] = 1.5;
	angle_pid_mat[0][1] = 0.0f;
	angle_pid_mat[0][2] = 0.2f;
	
	angle_pid_mat[1][0] = 1.5;
	angle_pid_mat[1][1] = 0.0f;
	angle_pid_mat[1][2] = 0.2f;
	
	angle_pid_mat[2][0] = 1.5;
	angle_pid_mat[2][1] = 0.0f;
	angle_pid_mat[2][2] = 0.2f;
}

void pid_init(void){
	pid_set_heavy();
}

uint8_t summing = 0;

float pid_N = 0.75f;

float pid_roll(float target, float real, float dt){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;
	
	error = target - real;
	sum = sum + error * dt;
	
	if(sum > 3.0f){
		sum = 3.0;
	}
	if(sum < -3.0f){
		sum = -3.0;
	}
	
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 100.0f){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	
	if(sum > 0.05 || sum < -0.05){
		summing = 0xff;
	}else{
		summing = 0x00;
	}
	
	d_error = 0.0f - real;
	error_rate = (d_error - pre_error) / dt;
	pre_error = d_error;
	
	d_out =  pid_N * error_rate + (1.0f - pid_N) * d_out_1; //D filter
	d_out_1 = d_out;

	result = mat_pid[0][0]*target + mat_pid[0][1]*(error + mat_pid[0][2]*sum + mat_pid[0][3]*d_out);
	return result;
}

float pid_pitch(float target, float real, float dt){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;
	
	error = target - real;
	sum = sum + error * dt;
	
	if(sum > 3.0f){
		sum = 3.0;
	}
	if(sum < -3.0f){
		sum = -3.0;
	}
	
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 100.0f){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	
	if(sum > 0.05 || sum < -0.05){
		summing = 0xff;
	}else{
		summing = 0x00;
	}
	
	d_error = 0.0f - real;
	error_rate = (d_error - pre_error) / dt;
	pre_error = d_error;
	
	d_out =  pid_N * error_rate + (1.0f - pid_N) * d_out_1; //D filter
	d_out_1 = d_out;

	result = mat_pid[1][0]*target + mat_pid[1][1]*(error + mat_pid[1][2]*sum + mat_pid[1][3]*d_out);
	return result;
}



float pid_yaw(float target, float real, float dt){
	static float error;
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	static float d_out_1;
	static float d_out;
	static float d_error;
	
	error = target - real;
	sum = sum + error * dt;
	
	if(sum > 3.0f){
		sum = 3.0;
	}
	if(sum < -3.0f){
		sum = -3.0;
	}
	
	if(error > 3.14f){
		sum = 0.0f;
	}
	if(error < -3.14f){
		sum = 0.0f;
	}
	if(throttle_set < 100.0f){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	
	if(sum > 0.05 || sum < -0.05){
		summing = 0xff;
	}else{
		summing = 0x00;
	}
	
	d_error = 0.0f - real;
	error_rate = (d_error - pre_error) / dt;
	pre_error = d_error;
	
	d_out =  pid_N * error_rate + (1.0f - pid_N) * d_out_1; //D filter
	d_out_1 = d_out;

	result = mat_pid[2][0]*target + mat_pid[2][1]*(error + mat_pid[2][2]*sum + mat_pid[2][3]*d_out);
	return result;
}

float pid_angle_roll(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0f;
	}
	if(sum < -25000.0f){
		sum = -25000.0f;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	if(ctrl_mode == 1){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[0][0]*error + angle_pid_mat[0][1]*sum + angle_pid_mat[0][2]*error_rate;
	return result;
}

float pid_angle_pitch(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0;
	}
	if(sum < -25000.0f){
		sum = -25000.0;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	if(ctrl_mode == 1){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[1][0]*error + angle_pid_mat[1][1]*sum + angle_pid_mat[1][2]*error_rate;
	return result;
}

float pid_angle_yaw(float error){
	static float sum;
	static float pre_error;
	static float result;
	static float error_rate;
	sum = sum + error;
	if(sum > 25000.0f){
		sum = 25000.0f;
	}
	if(sum < -25000.0f){
		sum = -25000.0f;
	}
	if(error > 45.0f){
		sum = 0.0f;
	}
	if(error < -45.0f){
		sum = 0.0f;
	}
	if(throttle_set < 100){
		sum = 0.0f;
	}
	if(arm_mode == 0){
		sum = 0.0f;
	}
	if(ctrl_mode == 1){
		sum = 0.0f;
	}
	error_rate = error - pre_error;
	pre_error = error;
	result = angle_pid_mat[2][0]*error + angle_pid_mat[2][1]*sum + angle_pid_mat[2][2]*error_rate;
	return result;
}

uint8_t tx6_buff[36];
	
float output_roll;
float output_pitch;
float output_yaw;
float imu_roll;
float imu_pitch;
float imu_yaw;
float target_velocity_roll = 0.0f;
float target_velocity_pitch = 0.0f;
float target_velocity_yaw = 0.0f;

Quaternion yaw_to_quaternion(double yaw) {
    Quaternion quaternion;
    quaternion.w = cos(yaw / 2);
    quaternion.x = 0;
    quaternion.y = 0;
    quaternion.z = sin(yaw / 2);
    return quaternion;
}

Quaternion pitch_to_quaternion(double pitch) {
    Quaternion quaternion;
    quaternion.w = cos(pitch / 2);
    quaternion.x = sin(pitch / 2);
    quaternion.y = 0;
    quaternion.z = 0;
    return quaternion;
}

Quaternion roll_to_quaternion(double roll) {
    Quaternion quaternion;
    quaternion.w = cos(roll / 2);
    quaternion.x = 0;
    quaternion.y = sin(roll / 2);
    quaternion.z = 0;
    return quaternion;
}

Quaternion multiply_quaternion(Quaternion *q1, Quaternion *q2) {
	
    Quaternion result;
	
		if(q1->w < 0.0f){
			q1->w = -1.0f * q1->w;
			q1->x = -1.0f * q1->x;
			q1->y = -1.0f * q1->y;
			q1->z = -1.0f * q1->z;
		}
		
		if(q2->w < 0.0f){
			q2->w = -1.0f * q2->w;
			q2->x = -1.0f * q2->x;
			q2->y = -1.0f * q2->y;
			q2->z = -1.0f * q2->z;
		}
		
    result.w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    result.x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    result.y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    result.z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
		
		if(result.w < 0.0f){
			result.w = -result.w;
			result.x = -result.x;
			result.y = -result.y;
			result.z = -result.z;
		}
		
    return result;
}

Quaternion quaternion_conjugate(Quaternion q) {
    Quaternion result = {q.w, -q.x, -q.y, -q.z};
    return result;
}

Quaternion quaternion_diff(Quaternion q1, Quaternion q2) {
		q1 = quaternion_conjugate(q1);
    Quaternion result = multiply_quaternion(&q2, &q1);
		if(result.w < 0.0f){
			result.w = -result.w;
			result.x = -result.x;
			result.y = -result.y;
			result.z = -result.z;
		}
    return result;
}

void quaternionToAngles(Quaternion q, float *roll, float *pitch, float *yaw) {
	float we = q.w;
	if(we > 0.999999f){
		we = 0.999999f;
	}	
	if(we < -0.999999f){
		we = -0.999999f;
	}
	float theta = 2.0f * acosf(we);
	float ne = sqrtf(1.0f - we * we);
	float nx = q.x / ne;
	float ny = q.y / ne;
	float nz = q.z / ne;
	*pitch = ny * theta;
	*roll = nx * theta;
	*yaw = nz * theta;
}

float euler_angle[3];
float error_angle[3];
float error_body[3];
Quaternion target_quaternion;
Quaternion measure_quaternion;
float target_yaw = 0.0f;

void World_to_Body(float *vector_e, float *vector_v,Quaternion Qin)
{
	float C11,C12,C13;
	float C21,C22,C23;
	float C31,C32,C33;
	
	float Q[4];
	
	Q[0] =  Qin.w;
	Q[1] = -Qin.x;
	Q[2] = -Qin.y;
	Q[3] = -Qin.z;
	
	
	C11 = Q[0]*Q[0] + Q[1]*Q[1] - Q[2]*Q[2] - Q[3]*Q[3];
	C12 = 2.0f*(Q[1]*Q[2] - Q[0]*Q[3]);
	C13 = 2.0f*(Q[1]*Q[3] + Q[0]*Q[2]);
	
	C21 = 2.0f*(Q[1]*Q[2] + Q[0]*Q[3]);
	C22 = Q[0]*Q[0] - Q[1]*Q[1] + Q[2]*Q[2] - Q[3]*Q[3];
	C23 = 2.0f*(Q[2]*Q[3] - Q[0]*Q[1]);
	
	C31 = 2.0f*(Q[1]*Q[3] - Q[0]*Q[2]);
	C32 = 2.0f*(Q[2]*Q[3] + Q[0]*Q[1]);
	C33 = Q[0]*Q[0] - Q[1]*Q[1] - Q[2]*Q[2] + Q[3]*Q[3];
	
	vector_v[0] = C11*vector_e[0] + C12*vector_e[1] + C13*vector_e[2];
	vector_v[1] = C21*vector_e[0] + C22*vector_e[1] + C23*vector_e[2];
	vector_v[2] = C31*vector_e[0] + C32*vector_e[1] + C33*vector_e[2];
	
}

void chassis_task(void const *pvParameters)
{
    vTaskDelay(1500);
		pid_init();
	
		tx6_buff[4] = 0x00;
		tx6_buff[5] = 0x00;
		tx6_buff[6] = 0x80;
		tx6_buff[7] = 0x7F;
	
		cali_cnt = 0;
		system_mode = 2;

		initButterworthFilter(&omega_x_filter, 1000.0, 1.0);
		initButterworthFilter(&omega_y_filter, 1000.0, 1.0);
		initButterworthFilter(&omega_z_filter, 1000.0, 1.0);
	
    while (1){
				memcpy(&gyro_data, get_gyro_data_point(), 12);
				memcpy(&angle_data, get_INS_angle_point(), 12);
			
//				if(cali_cnt < 100000){
//					cali_cnt = cali_cnt + 1;
//					cali_imu_num = cali_imu_num + 0.00001 * gyro_data[1];
//				}

				if(Sbus_ctrl.ch[5] > 1500){
					ctrl_mode = 2;
				}else{
					ctrl_mode = 1;
				}
				
				if(Sbus_ctrl.ch[6] > 1500){
					arm_mode_stick = 0xff;
				}else{
					arm_mode_stick = 0x00;
				}
				
				if(ctrl_mode != 3){//不在板外模式时，起停桨听从拨杆，都是上升或下降沿出发起停桨命令，不是拨杆位置，板外模式时不响应拨杆起停桨
					if(arm_mode_stick != arm_mode_stick_pre){
						arm_mode_stick_pre = arm_mode_stick;
						arm_mode = arm_mode_stick;
					}
				}else{
					arm_mode_stick_pre = arm_mode_stick;//假设突然退出板外，由板外起桨时拨杆没放在解锁位，退出时没有发生杆位变化，不会触发空中停桨
					//在板外起桨后进入板内也不会停桨
				}

				stick_mode = stick_heli;
				
				float throttle_in = d_ch(2) / 2.0f + 500.0f;
				float yaw_in = d_ch(3) / 2.0f;
				float roll_in = d_ch(0) / 2.0f;
				float pitch_in = d_ch(1) / -2.0f;
				throttle_set = throttle_in;

				if(system_mode == 2){
					float motor1;
					float motor2;
					float motor3;
					float motor4;
					
					memcpy(&measure_quaternion, &ahrs_quaternion, 16);
					Quaternion de_yaw_quaternion = yaw_to_quaternion(-angle_data[0]);
					Quaternion de_yaw_ahrs = multiply_quaternion(&de_yaw_quaternion, &measure_quaternion);
					
					target_quaternion.w = 1.0f;
					target_quaternion.x = 0.0f;
					target_quaternion.y = 0.0f;
					target_quaternion.z = 0.0f;
					
					Quaternion temp_quaternion;
					temp_quaternion = pitch_to_quaternion(d_ch(1) * 0.0020708f);
					target_quaternion = multiply_quaternion(&temp_quaternion, &target_quaternion);
					temp_quaternion = roll_to_quaternion(d_ch(0) * -9.85398e-4);
					target_quaternion = multiply_quaternion(&temp_quaternion, &target_quaternion);
					
					temp_quaternion = quaternion_diff(de_yaw_ahrs, target_quaternion);
					quaternionToAngles(temp_quaternion, &error_angle[0], &error_angle[1], &error_angle[2]);
					
					if(isnan(error_angle[0])){
						error_angle[0] = 0.0f;
					}
					if(isnan(error_angle[1])){
						error_angle[1] = 0.0f;
					}
					if(isnan(error_angle[2])){
						error_angle[2] = 0.0f;
					}
					World_to_Body(error_angle, error_body, de_yaw_ahrs);

					w_yaw_world[0] = 0.0f;
					w_yaw_world[1] = 0.0f;
					w_yaw_world[2] = d_ch(3) * -0.002341f;
					
					World_to_Body(w_yaw_world, w_yaw_body, measure_quaternion);

					if(ctrl_mode == 2){
						target_velocity_pitch = pid_angle_pitch(-error_body[0]) - w_yaw_body[0];
						target_velocity_roll = pid_angle_roll(-error_body[1]) - w_yaw_body[1];
						target_velocity_yaw = pid_angle_yaw(error_body[2]) + w_yaw_body[2];
						
						if(target_velocity_pitch > 3.0f){
							target_velocity_pitch = 3.0f;
						}
						if(target_velocity_pitch < -3.0f){
							target_velocity_pitch = -3.0f;
						}
						if(target_velocity_roll > 3.0f){
							target_velocity_roll = 3.0f;
						}
						if(target_velocity_roll < -3.0f){
							target_velocity_roll = -3.0f;
						}
						if(target_velocity_yaw > 3.0f){
							target_velocity_yaw = 3.0f;
						}
						if(target_velocity_yaw < -3.0f){
							target_velocity_yaw = -3.0f;
						}
					}
					
					if(ctrl_mode == 1){
						if(stick_mode == stick_heli){
							target_velocity_roll = d_ch(0) * 0.002341f;
							target_velocity_pitch = d_ch(1) * -0.002341f;
							target_velocity_yaw = d_ch(3) * -0.002341f;
						}else{
							target_velocity_roll = d_ch(3) * -0.002341f;
							target_velocity_pitch = d_ch(1) * -0.002341f;
							target_velocity_yaw = d_ch(0) * -0.002341f;
						}
					}
					
					imu_roll = -gyro_data[1];
					imu_pitch = -gyro_data[0];
					imu_yaw = gyro_data[2];
					
					float roll_in = applyButterworthFilter(&omega_x_filter, imu_roll);
					float pitch_in = applyButterworthFilter(&omega_y_filter, imu_pitch);
					float yaw_in = applyButterworthFilter(&omega_z_filter, imu_yaw);
					
					output_roll = pid_roll(target_velocity_roll, roll_in, 0.001);
					output_pitch = pid_pitch(target_velocity_pitch, pitch_in, 0.001);
					output_yaw = pid_yaw(target_velocity_yaw, yaw_in, 0.001);
					
					//memcpy(&tx6_buff[0], &throttle_in, 4);
					//usart6_tx_dma_enable(tx6_buff, 8);
					
					float f1 = 0.0 - output_roll + output_pitch - output_yaw + throttle_in;
					float f2 = 0.0 + output_roll + output_pitch + output_yaw + throttle_in;
					float f3 = 0.0 - output_roll - output_pitch + output_yaw + throttle_in;
					float f4 = 0.0 + output_roll - output_pitch - output_yaw + throttle_in;
					
					
					if(f1 > 1000.0f){
						f1 = 1000.0f;
					}
					if(f1 < 0.0f){
						f1 = 0.0f;
					}
					if(f2 > 1000.0f){
						f2 = 1000.0f;
					}
					if(f2 < 0.0f){
						f2 = 0.0f;
					}
					if(f3 > 1000.0f){
						f3 = 1000.0f;
					}
					if(f3 < 0.0f){
						f3 = 0.0f;
					}
					if(f4 > 1000.0f){
						f4 = 1000.0f;
					}
					if(f4 < 0.0f){
						f4 = 0.0f;
					}
					
					motor1 = f1 + 1000;
					motor2 = f2 + 1000;
					motor3 = f3 + 1000;
					motor4 = f4 + 1000;

					if(arm_mode == 0){
						motor1 = 1000;
						motor2 = 1000;
						motor3 = 1000;
						motor4 = 1000;
					}else{
						if(motor1 < motor_idle_speed){
							motor1 = motor_idle_speed;
						}
						if(motor2 < motor_idle_speed){
							motor2 = motor_idle_speed;
						}
						if(motor3 < motor_idle_speed){
							motor3 = motor_idle_speed;
						}
						if(motor4 < motor_idle_speed){
							motor4 = motor_idle_speed;
						}
					}					
					if(pwm_debugging){
						set_pwm(motor1, motor2, motor3, motor4);
					}
				}
				vTaskDelay(1);//PID频率:1000HZ
		}
}
