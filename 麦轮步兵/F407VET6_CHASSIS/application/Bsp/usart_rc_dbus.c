#include "usart_rc_dbus.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
RC_ctrl_t rc_ctrl; 
uint8_t sbus_rx_buf[2][RC_FRAME_LENGTH];//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��

//ң������ʼ��
void rc_dbus_init(void)
{
//	RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
	USART_RxDMA_DoubleBuffer_Init(&huart6,sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM); 
}

////ң��������DMA���ճ�ʼ��
//void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
//{
//	//ʹ��DMA���ڽ���
//	SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

//	//ʹ�ܿ����ж�
//	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);

//	//ʧЧDMA
//	__HAL_DMA_DISABLE(&hdma_usart6_rx);
//	while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
//	{
//		__HAL_DMA_DISABLE(&hdma_usart6_rx);
//	}
//	
//	hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);

//	//�ڴ滺����1
//	hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);

//	//�ڴ滺����2
//	hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);

//	//���ݳ���
//	hdma_usart6_rx.Instance->NDTR = dma_buf_num;

//	//ʹ��˫������
//	SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

//	//ʹ��DMA
//	__HAL_DMA_ENABLE(&hdma_usart6_rx);
//}

//˫���������ڳ�ʼ��
void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength)
{ 
 huart->ReceptionType=HAL_UART_RECEPTION_TOIDLE; 

 huart->RxEventType=HAL_UART_RXEVENT_IDLE; 

 huart->RxXferSize=DataLength; 

 SET_BIT(huart->Instance->CR3,USART_CR3_DMAR); 

 __HAL_UART_ENABLE_IT(huart,UART_IT_IDLE);  
 
 HAL_DMAEx_MultiBufferStart(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength); 
}

//ң�������ݽ��㣬��DMA���յ������ݴ���ṹ���Ա
void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
											 (sbus_buf[4] << 10)) &0x07ff;
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
	rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
	rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
//	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

	rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
//	rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

//�����жϺ���
void USART6_IRQHandler(void)
{
	if(huart6.Instance->SR & UART_FLAG_RXNE)//���յ�����
	{
		__HAL_UART_CLEAR_PEFLAG(&huart6);
	}
	else if(USART6->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;

		__HAL_UART_CLEAR_PEFLAG(&huart6);

		if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			//ʧЧDMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

			//�����趨���ݳ���
			hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//�趨������1
			hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;

			//ʹ��DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
			}
		}
		else
		{
			//ʧЧDMA
			__HAL_DMA_DISABLE(&hdma_usart6_rx);

			//��ȡ�������ݳ���,���� = �趨���� - ʣ�೤��
			this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart6_rx.Instance->NDTR;

			//�����趨���ݳ���
			hdma_usart6_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

			//�趨������0
			DMA2_Stream1->CR &= ~(DMA_SxCR_CT);
			
			//ʹ��DMA
			__HAL_DMA_ENABLE(&hdma_usart6_rx);

			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				//����ң��������
				sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
			}
		}
	}
}

//static void USER_USART5_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
//{ 
//	 if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET) 
//	 { 
//			__HAL_DMA_DISABLE(huart->hdmarx); 

//			((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT; 
//	 
//			__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM); 

//			if(Size == RC_FRAME_LENGTH) 
//			{ 
//				sbus_to_rc(sbus_rx_buf[0],&rc_ctrl); 
//			}
//	 }
//	 else
//	 { 
//		 __HAL_DMA_DISABLE(huart->hdmarx); 

//			((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT); 
//	 
//			__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM); 

//			if(Size == RC_FRAME_LENGTH) 
//			{ 
//				sbus_to_rc(sbus_rx_buf[1],&rc_ctrl); 
//			}			 
//	 } 
//	 __HAL_DMA_ENABLE(huart->hdmarx);				 
//}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size) 
//{ 
//	 if(huart == &huart6)
//	 {
//		 USER_USART5_RxHandler(huart,Size); 
//	 }  
//}