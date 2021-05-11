#ifndef __ADC_H
#define	__ADC_H


#include "stm32f10x.h"

#define current_multiple 5.11/1.02     //λ???
#define SAMPLING_RES 0.05f   
#define b_constant 3380
#define vol_multiple (39.2+6.19)/(6.19)


extern __IO uint16_t ADC_Collect[10][5];
extern __IO uint16_t ADC_ConvertedValue;

extern float Refer_ADC_Value;
extern float Pos_ADC_Value;
extern float POWER_ADC_Value;
extern float Current_ADC_Value;
extern float Temperature_ADC_Value;

extern float battery_v;//当前电压 （0.1V）
extern float current_ma;//当前电流(MA)
extern float temperature_c;//当前温度（℃）
extern float ntc_v;
extern float ntc_res;

void ADC1_Init(void);


#endif /* __ADC_H */
