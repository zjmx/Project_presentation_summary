#include "USART_task.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
Communicate_measure_t Send_data;//双板通讯数据打包
uint8_t uart4_tx_data[18];//18个字节，给了36个字节长度，防止DMA传输越界
//uint8_t uart4_rx_data[2][Commu_FRAME_LENGTH];//18个字节，给了36个字节长度，防止DMA传输越界
uint8_t uart4_rx_data[20];
uint8_t uart4_rx_data_revise[10];

//双板通讯函数
void USART_task(void)
{
	
}

//双板通讯数据接收
void data_Rx_analysis(uint8_t *rx_data,Communicate_measure_t *send_data)
{//接收8位数据
	send_data->frame_rx_header1=rx_data[0];
	send_data->vx_rx=((rx_data[1]<<8)|(rx_data[2]));
	send_data->vy_rx=((rx_data[3]<<8)|(rx_data[4]));
	send_data->vw_rx=((rx_data[5]<<8)|(rx_data[6]));
	send_data->s_rx[0]=rx_data[7];
	send_data->s_rx[1]=rx_data[8];
	send_data->frame_rx_end=rx_data[9];
}

//遥控器初始化
void USART_485_init(void)
{
	HAL_UART_Receive_DMA(&huart4,uart4_rx_data,sizeof(uart4_rx_data));//重新开启DMA接收
//	USART_RxDMA_DoubleBuffer_Init(&huart4,uart4_rx_data[0],uart4_rx_data[1],Commu_dobule_FRAME_LENGTH);
//	send_init(uart4_rx_data[0],uart4_rx_data[1],Commu_dobule_FRAME_LENGTH);
}



//串口中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==UART4)
	{
		//利用数据标志位进行数据包校准
		for(int i=0;i<20;i++)
		{
			if((uart4_rx_data[i]==0xFF)&&(uart4_rx_data[i+9]==0xFE))
			{
				for(int j=0;j<10;j++)
				{
					uart4_rx_data_revise[j]=uart4_rx_data[j+i];
				}
			}
		}
		data_Rx_analysis(uart4_rx_data_revise, &Send_data);
		HAL_UART_Receive_DMA(&huart4,uart4_rx_data,sizeof(uart4_rx_data));//重新开启DMA接收
	}
}

////遥控器串口DMA接收初始化
//void send_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
//{
//	//使能DMA串口接收
//	SET_BIT(huart4.Instance->CR3, USART_CR3_DMAR);

//	//使能空闲中断
//	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);

//	//失效DMA
//	__HAL_DMA_DISABLE(&hdma_uart4_rx);
//	while(hdma_uart4_rx.Instance->CR & DMA_SxCR_EN)
//	{
//		__HAL_DMA_DISABLE(&hdma_uart4_rx);
//	}
//	
//	hdma_uart4_rx.Instance->PAR = (uint32_t) & (UART4->DR);

//	//内存缓冲区1
//	hdma_uart4_rx.Instance->M0AR = (uint32_t)(rx1_buf);

//	//内存缓冲区2
//	hdma_uart4_rx.Instance->M1AR = (uint32_t)(rx2_buf);

//	//数据长度
//	hdma_uart4_rx.Instance->NDTR = dma_buf_num;

//	//使能双缓冲区
//	SET_BIT(hdma_uart4_rx.Instance->CR, DMA_SxCR_DBM);

//	//使能DMA
//	__HAL_DMA_ENABLE(&hdma_uart4_rx);
//}

////串口中断函数
//void UART4_IRQHandler(void)
//{
//	if(huart4.Instance->SR & UART_FLAG_RXNE)//接收到数据
//	{
//		__HAL_UART_CLEAR_PEFLAG(&huart4);
//	}
//	else if(UART4->SR & UART_FLAG_IDLE)
//	{
//		static uint16_t this_rx_len = 0;
//		
//		__HAL_UART_CLEAR_PEFLAG(&huart4);

//		if ((hdma_uart4_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//		{
//			//失效DMA
//			__HAL_DMA_DISABLE(&hdma_uart4_rx);

//			//获取接收数据长度,长度 = 设定长度 - 剩余长度
//			this_rx_len = Commu_dobule_FRAME_LENGTH - hdma_uart4_rx.Instance->NDTR;

//			//重新设定数据长度
//			hdma_uart4_rx.Instance->NDTR = Commu_dobule_FRAME_LENGTH;

//			//设定缓冲区1
//			hdma_uart4_rx.Instance->CR |= DMA_SxCR_CT;

//			//使能DMA
//			__HAL_DMA_ENABLE(&hdma_uart4_rx);

//			if(this_rx_len == Commu_FRAME_LENGTH)
//			{
//				data_Rx_analysis(uart4_rx_data[0], &Send_data);
//			}
//		}
//		else
//		{
//			//失效DMA
//			__HAL_DMA_DISABLE(&hdma_uart4_rx);

//			//获取接收数据长度,长度 = 设定长度 - 剩余长度
//			this_rx_len = Commu_dobule_FRAME_LENGTH - hdma_uart4_rx.Instance->NDTR;

//			//重新设定数据长度
//			hdma_uart4_rx.Instance->NDTR = Commu_dobule_FRAME_LENGTH;

//			//设定缓冲区0
//			DMA1_Stream2->CR &= ~(DMA_SxCR_CT);
//			
//			//使能DMA
//			__HAL_DMA_ENABLE(&hdma_uart4_rx);

//			if(this_rx_len == Commu_FRAME_LENGTH)
//			{
//				//处理遥控器数据
//				data_Rx_analysis(uart4_rx_data[1], &Send_data);
//			}
//		}
//	}
//}



