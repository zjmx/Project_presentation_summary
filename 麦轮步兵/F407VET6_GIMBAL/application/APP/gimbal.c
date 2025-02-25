#include "gimbal.h"

extern RC_ctrl_t rc_ctrl;//ң��������ָ��
extern motor_measure_t motor_gimbal[2];//��̨������ݷ��ؽṹ��
extern INS_t INS;//mcu���ݽṹ��
Mechanical_measure_t mechanical_data;;//��е���ݽṹ��

//���̲�����ʼ��
void gimbal_init(void)
{
	//PID����
	float gimbal_current_PID[2][3]={{0.8,0,0.6},{0.8,0,0.6}};
	float gimbal_angle_PID[2][3]={{0,0,0},{1000,0,1000}};
	
	//���ȶ���̨ģʽ
	float gimbal_mcu_current_PID[2][3]={{0.8,0,0.6},{0.8,0,0.6}};//pitch������װ��������ֵ���Է���
	float gimbal_mcu_angle_PID[2][3]={{20000,30,15000},{350000,0,1500000}};
	
	//��е���ݳ�ʼ��
	mechanical_data.pitch_H_angle=0.40f;
	mechanical_data.pitch_L_angle=-0.40f;
	mechanical_data.yaw_H_angle=PI;
	mechanical_data.yaw_L_angle=-PI;
	
	//pid��ʼ��
	for(int i=0;i<2;i++)
	{
		PID_init(&motor_gimbal[i].gimbal_current_pid,gimbal_current_PID[i],10000,30000);//��챵��������PID��ʼ��
		PID_init(&motor_gimbal[i].gimbal_angle_pid,gimbal_angle_PID[i],10000,30000);//��̨����ǶȻ�PID��ʼ��
		//mcu_pid��ʼ��
		PID_init(&motor_gimbal[i].gimbal_mcu_current_pid,gimbal_mcu_current_PID[i],10000,30000);//��챵��������PID��ʼ��
		PID_init(&motor_gimbal[i].gimbal_mcu_angle_pid,gimbal_mcu_angle_PID[i],10000,30000);//��̨����ǶȻ�PID��ʼ��
	}
}

float target_angle[2]={1.55f,-1.604f};
float target_mcu_angle[2]={0,0};
float yaw_k=-1.7508,yaw_b=1.9053;
float angle=0;
//�����˶�����
void gimbal_task(void)
{	
	switch(INS.ins_flag)
	{
		case 0:break;
		case 1:
		{
			angle=(target_mcu_angle[1]*yaw_k+yaw_b)-(motor_gimbal[1].rad_ecd-(-1.61049294));
			while ( angle > PI )
				angle -= 2 * PI;
			while ( angle < - PI )
				angle += 2 * PI;
			if(rc_ctrl.rc.s[0]==2||rc_ctrl.rc.s[1]==2)
			{//ʧ��
				for(int i=0;i<2;i++)
				{
					motor_gimbal[i].gimbal_mcu_current_pid.out=0;
				}
			}
			else if(rc_ctrl.rc.s[0]==1)
			{//ʹ��ģʽ
				//̧����ҡͷ����
				if(rc_ctrl.rc.ch[1]>0){target_mcu_angle[0]+=PI/1000.0f;}
				else if(rc_ctrl.rc.ch[1]<0){target_mcu_angle[0]-=PI/1000.0f;}
				
				if(rc_ctrl.rc.ch[0]>0){target_mcu_angle[1]-=PI/800.0f;}
				else if(rc_ctrl.rc.ch[0]<0){target_mcu_angle[1]+=PI/800.0f;}
				
				//pitch��ߵ�λ����
				if(target_mcu_angle[0]>mechanical_data.pitch_H_angle){target_mcu_angle[0]=mechanical_data.pitch_H_angle;}
				else if(target_mcu_angle[0]<mechanical_data.pitch_L_angle){target_mcu_angle[0]=mechanical_data.pitch_L_angle;}
				
				if(rc_ctrl.rc.s[1]==1)
				{//���̸�����̨ģʽ
					//yaw
					PID_calc_rad_format(&motor_gimbal[1].gimbal_angle_pid,motor_gimbal[1].rad_ecd,target_angle[1]);
					Send_data.vw_tx=motor_gimbal[1].gimbal_angle_pid.out;
				}
				else if(rc_ctrl.rc.s[1]==3)
				{//С����ģʽ
					//�����ٶȸ���
					Send_data.vw_tx=1000;
					//����̨Ϊ�����򣬽����ƶ�
					Send_data.vy_tx=((rc_ctrl.rc.ch[3]*cosf((angle)))-(rc_ctrl.rc.ch[2]*sinf((angle))));
					Send_data.vx_tx=((rc_ctrl.rc.ch[3]*sinf((angle)))+(rc_ctrl.rc.ch[2]*cosf((angle))));
				}
				
				//pitch
				PID_calc_rad_format(&motor_gimbal[0].gimbal_mcu_angle_pid,INS.Pitch,target_mcu_angle[0]);
				PID_calc(&motor_gimbal[0].gimbal_mcu_current_pid,motor_gimbal[0].given_current,motor_gimbal[0].gimbal_mcu_angle_pid.out);
				//yaw
				PID_calc_rad_format(&motor_gimbal[1].gimbal_mcu_angle_pid,INS.Yaw,target_mcu_angle[1]);
				PID_calc(&motor_gimbal[1].gimbal_mcu_current_pid,motor_gimbal[1].given_current,motor_gimbal[1].gimbal_mcu_angle_pid.out);
				CAN_gimbal_send(&hcan1,-motor_gimbal[0].gimbal_mcu_current_pid.out,motor_gimbal[1].gimbal_mcu_current_pid.out,0,0);
			}
		}break;
		default:break;
	}
//	MyPrintf(&huart2,"%.2lf,%.2lf\r\n",motor_gimbal[1].rad_ecd,target_angle[1]);
	osDelay(1);
}
