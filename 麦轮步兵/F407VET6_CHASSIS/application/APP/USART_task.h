#ifndef USART_TRAIN_TASK_H
#define USART_TRAIN_TASK_H

#include "main.h"
#include "usart_rc_dbus.h"
#include "usart.h"

#define Commu_dobule_FRAME_LENGTH 16
#define Commu_FRAME_LENGTH 8

typedef struct
{
	//�°����
	uint8_t frame_rx_header1;//֡ͷ1
	int16_t vx_rx;//x�����ٶ�
	int16_t vy_rx;//y�����ٶ�
	int16_t vw_rx;//���ٶ�
	char s_rx[2];//ģʽ�л�
	uint8_t frame_rx_end;//֡β
	
	//�ϰ巢��
//	uint8_t frame_tx_header1;//֡ͷ1
//	int16_t vx_tx;//x�����ٶ�
//	int16_t vy_tx;//y�����ٶ�
//	int16_t vw_tx;//���ٶ�
//	uint8_t frame_tx_end;//֡β
}Communicate_measure_t;
extern Communicate_measure_t Send_data;

void USART_485_init(void);
void send_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void data_Rx_analysis(uint8_t *rx_data,Communicate_measure_t *send_data);
void USART_485_rx_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void USART_task(void);

#endif



