#include "sys.h"
#include "usart.h"	
#include "communication.h"
u8 MotoRxBuff[0xff]={0};
u8 MotoRXCtp=0;
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 

////////////////////////////////////////////////////////////////////////////////// 	  
 

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
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
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void uart_init(u32 bound){
  //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  
  USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);//开启串口接收中断
  USART_Cmd(USART1, DISABLE);                    //使能串口1 

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
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
//	
//}
//void TXdisInit()
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入  GPIO_Mode_AIN
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
//	
//	USART1->CR1 |= 0x0024 ;
//	
//}

void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 temp,Status;
	if(USART_GetFlagStatus(USART1,USART_IT_RXNE))
	{
		temp=USART_ReceiveData(USART1);
		cbWrite(&rxbuf,&temp); 
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	Status = USART_GetFlagStatus(USART1, USART_FLAG_NE|USART_FLAG_PE|USART_FLAG_FE|USART_FLAG_ORE);
	if(Status!=RESET){//如果发生错误接忽略接收的数据
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1,Status);//把错误标志清楚
		return;
	}
} 
/*
	串口1发送函数
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
