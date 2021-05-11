#ifndef __TIM_H
#define __TIM_H
#include "stm32f10x.h"

#define TIMLongOut  TIM1->CNT>20000
#define min_limit	200
#define max_limit	4000

extern float kp,ki,kd;

extern int Compare_PWM_OUT,calculate_PWM;
extern float Error,ErrorOld,Differential;

extern float target_position;
extern u16 ADC_Collect_temp[5];//CH17»ù×¼ DWQ POWER Current Temperature
extern u16 count_TIM4;

void Tim3PwmInit(void);
void TIM2_Int_Init(u16 arr,u16 psc);
void TIM4_Int_Init(u16 arr,u16 psc);
void TIM1Init(void);
void OTstart(void);
void OTdisen(void);
void ParameterInit(u8 Version0,u8 Version1,u8 Version2);
#endif
