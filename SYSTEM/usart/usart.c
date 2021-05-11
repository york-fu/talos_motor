#include "sys.h"
#include "usart.h"	
#include "communication.h"
u8 MotoRxBuff[0xff]={0};
u8 MotoRXCtp=0;
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 

////////////////////////////////////////////////////////////////////////////////// 	  
 

//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 bound){
  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//�������ڽ����ж�
  USART_Cmd(USART1, DISABLE);                    //ʹ�ܴ���1 

  RX_MODE;
}
//void TXenInit()
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//	USART1->CR1 &= 0xffdb ;
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
//	
//}
//void TXdisInit()
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������  GPIO_Mode_AIN
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
//	
//	USART1->CR1 |= 0x0024 ;
//	
//}

void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 temp,Status;
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE))
	{
		temp=USART_ReceiveData(USART1);
		cbWrite(&rxbuf,&temp); 
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	Status = USART_GetFlagStatus(USART1, USART_FLAG_NE|USART_FLAG_PE|USART_FLAG_FE|USART_FLAG_ORE);
	if(Status!=RESET){//�����������Ӻ��Խ��յ�����
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1,Status);//�Ѵ����־���
		return;
	}
} 
/*
	����1���ͺ���
	void usart_send()
*/
void usart_send(u8 *usart1send_my_buff,u8 len_usart1_send)
{
	u8 usart1send_ctp;
	
	TIM_Cmd(TIM2, DISABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE );
	TIM_Cmd(TIM4, DISABLE);
	TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE );
	
//	TXenInit();
	TX_MODE;
	delay_us(1);
	USART_ClearFlag(USART1,USART_FLAG_TC);
	for(usart1send_ctp=0;usart1send_ctp<len_usart1_send;usart1send_ctp++)
	{
		USART_SendData(USART1,usart1send_my_buff[usart1send_ctp]);
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
	}
//	TXdisInit();
	cbInit(&rxbuf,rx_buf_len,rxbuf_st);
	RX_MODE;
	
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE );
	TIM_Cmd(TIM4, ENABLE);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE );
}
#endif
