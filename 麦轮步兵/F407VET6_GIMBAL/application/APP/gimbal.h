#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "usart_rc_dbus.h"
#include "pid.h"
#include "can_receive.h"
#include "math.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "function.h"
#include "ins_task.h"
#include "USART_task.h"

#define Radius 1//�����е�뾶
#define wheel_rpm_ratio 36.0//3508���ٱ�
#define rotate_ratio_f 1
#define rotate_ratio_b 1

//��̨��е���ݽṹ��
typedef struct
{
	float pitch_H_angle;//pitch�߶���λ
	float pitch_L_angle;//
	
	float yaw_H_angle;//pitch�߶���λ
	float yaw_L_angle;//
}Mechanical_measure_t;
extern Mechanical_measure_t mechanical_data;

//���ֵ������ݽṹ��
typedef struct
{
	
}ChassisHandle_t;
extern ChassisHandle_t ChassisHandle;


//ӳ�亯��������������ֵ��0~8191��ת��Ϊ�����ƵĽǶ�ֵ��-pi~pi��
double msp(double x, double min_in, double max_in, double min_out, double max_out);
//���̲�����ʼ��
void gimbal_init(void);
//�����˶�����
void gimbal_task(void);
//���ֵ���ٶȽ��㺯��
void Steer_Calculate(ChassisHandle_t *chassishandle,int16_t vx,int16_t vy,int16_t vw);

extern float target_angle[2];

#endif



