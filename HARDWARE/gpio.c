#include "gpio.h"
#include "stm32f10x.h"

u8 f_flag;
u8  direction_flag = 0;

void LED_Init(void)//LED7  测试用
{
	
	GPIO_InitTypeDef	GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//使能 AFIO 时钟，才可使调用GPIO_PinRemapConfig()生效
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能外设 PA 时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	//禁止JTAG，从而PA15可以作为普通IO使用，否则PA15不能当做普通IO口使用

	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);
}

void Delay_Us(u16 myUs)   
{
  u16 i;
  while(myUs--)
  {
    i=8;
    while(i--);
  }
}

void IO_Init(void)
{
	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA,ENABLE);//使能晚外设 PB PA 时钟
	
	//PB12
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//	GPIO_Init(GPIOB, &GPIO_InitStructure);		  
//	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);	 
//	GPIO_ResetBits(GPIOB,GPIO_Pin_13);

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure); 
//	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	
	
//74HC126   
	//TX_en PB10
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	//RX_en PB11   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB,GPIO_Pin_11);

//SD en	PA1  pwm_out
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//MAX9934  CS PA12
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_12);

	LED_Init();
}

void IO_test(void)
{
	if(f_flag)
			{
				f_flag=0;
			}
			else {
				f_flag=1;
				}

}



void pwm1EN(u16 PWM1WwantNum)
{
	TIM3->CCR1 = PWM1WwantNum;
	TIM3->CCER = 0x0001;
}

void pwm2EN(u16 PWM2WwantNum)
{
	TIM3->CCR2 = PWM2WwantNum;
	TIM3->CCER = 0x0010;
}


void SD_enable(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_1);	
}

void SD_disable(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
}

void ClosePWM(void)
{
	TIM3->CCER = 0x0000;
	direction_flag=3;
}

void PWM1Run(u16 pwmnunms)
{
	
	TIM3->CCER &= 0x1101;//closepwm 2
	if(direction_flag==1)
		Delay_Us(2);
	direction_flag=0;
	pwm1EN(pwmnunms);
	SD_enable();
}

void PWM2Run(u16 pwmnunmf)
{
	
	TIM3->CCER &= 0x1110;//closepwm 1
	if(direction_flag==0)
		Delay_Us(2);
	direction_flag=1;
	pwm2EN(pwmnunmf);
	SD_enable();
}

