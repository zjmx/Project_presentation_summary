#ifndef STEERING_WHEEL_CHASSIS_H
#define STEERING_WHEEL_CHASSIS_H

#include "main.h"
#include "usart_rc_dbus.h"
#include "pid.h"
#include "can_receive.h"
#include "math.h"
#include "usart.h"
#include "string.h"
#include "USART_task.h"

#define Radius 1//�����е�뾶
#define wheel_rpm_ratio 36.0//3508���ٱ�
#define rotate_ratio_f 1
#define rotate_ratio_b 1

//���ֵ��̻�е���ݽṹ��
typedef struct
{
	float wheel_perimeter;//����ܳ�
}MechanicalStructure_t;
extern MechanicalStructure_t structure;

//���ֵ��̻�е���ݽṹ��
typedef struct
{
	int8_t speed_direction;//�ٶȷ���
}Steer_Type;
extern Steer_Type steer_set[4];

//���ֵ������ݽṹ��
typedef struct
{
	motor_measure_t motor_chassis[4];//��챵�����ݷ��ؽṹ��
	
	float vx; //Vx                      
	float vy; //Vy                   
	float vw; //Vw	
	float wheel_rpm[4]; //���ڴ洢ת����Ϣ
	
	float steeringAngleTarget[4];//�����Ŀ��ת��              
	float lastSteeringAngleTarget[4]; //��һ�ε�Ŀ��ת��
	float steeringAngle[4]; //��ǰ�Ķ��ת��                     
	float last_steeringAngle[4]; //��һ�εĶ��ת��
	int32_t motor_circle[4]; //���ת����Ȧ��
	int32_t motor_target_count[4]; //�����ת�Ĵ���
	Steer_Type steer_set[4]; //���ڶ����ٶȽ���
	uint16_t turnFlag[4];  //���ֽǶȽ����־λ
	
}ChassisHandle_t;
extern ChassisHandle_t ChassisHandle;


//ӳ�亯��������������ֵ��0~8191��ת��Ϊ�����ƵĽǶ�ֵ��-pi~pi��
double msp(double x, double min_in, double max_in, double min_out, double max_out);
//���̲�����ʼ��
void chassis_init(void);
//�����˶�����
void chassis_task(void);
//���ֵ���ٶȽ��㺯��
void Steer_Calculate(ChassisHandle_t *chassishandle,int16_t vx,int16_t vy,int16_t vw);

#endif



