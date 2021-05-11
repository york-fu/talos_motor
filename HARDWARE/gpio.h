#ifndef __GPIO_H
#define __GPIO_H
#include "stm32f10x.h"

#define ledON 	GPIO_ResetBits(GPIOA, GPIO_Pin_15)	
#define ledOFF 	GPIO_SetBits(GPIOA, GPIO_Pin_15)

#define PWMout_en GPIO_SetBits(GPIOA,GPIO_Pin_1);
#define PWMout_dis GPIO_ResetBits(GPIOA,GPIO_Pin_1);

void IO_Init(void);
void IO_test(void);

void pwm1EN(u16 PWM1WwantNum);
void pwm2EN(u16 PWM1WwantNum);
void PWM1Run(u16 pwmnunms);
void PWM2Run(u16 pwmnunms);
void SD_disable(void);
void ClosePWM(void);
void Delay_Us(u16 myUs);

	
#endif 
