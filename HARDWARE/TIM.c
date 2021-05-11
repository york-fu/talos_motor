#include "TIM.h"
#include <stdlib.h>
#include "gpio.h"
#include "DataFlash.h"
#include "adc.h"
#include "delay.h"
#include "math.h"
#include "UsarDataDIS.h"

extern u8 Compute_flag;
extern int uart_send;
extern u8 MotoFlashEEPROM[0XFF];
extern u8 MotoFlashREG[0xff];

float Runstep=5;//20ms 加一个step
float StepTemp=5;

float target_position=2000;
int Compare_PWM_OUT=0,calculate_PWM=0;
float Error,ErrorOld=0,Integral_error=0;
float Differential=0;

int T_target_position=0;
int U_target_position=2000;
u8  U_torque_status = 0;



void Tim3PwmInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);//????,tim3,gpioA,afio
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_DeInit(TIM3);//TIM3
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = 3600;//3600->20khz
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//初始化通道1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 3600/2;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	//初始化通道2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = 3600/2;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM3, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
	TIM_CtrlPWMOutputs(TIM3, ENABLE);
	TIM3->CCER=0x0000;
	//NOMClose();
}



//通用定时器2中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M  
//arr：自动重装值。
//psc：时钟预分频数
//**TIM2
void TIM2_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //时钟使能
	
	//定时器TIM2初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM2,TIM_IT_Update,DISABLE ); //使能指定的TIM2中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

	TIM_Cmd(TIM2, ENABLE);  //使能TIMx，开启时钟				 
}


//通用定时器4中断初始化
//这里时钟选择为APB1的2倍，而APB1为36M  
//arr：自动重装值。
//psc：时钟预分频数
//**TIM4
void TIM4_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //时钟使能
	
	//定时器TIM4初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM4,TIM_IT_Update,DISABLE ); //使能指定的TIM3中断,允许更新中断

	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //初始化NVIC寄存器

	TIM_Cmd(TIM4, ENABLE);  //使能TIMx，开启时钟				 
}

/*
 *函数名：TIM1初始化
 *创建人：王宋
 *功能：只用计数器，循环累加，用作接受超时判断
 * */
void TIM1Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能
	
	//定时器TIM2初始化
	TIM_TimeBaseStructure.TIM_Period = 65534; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =71; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_Cmd(TIM1, DISABLE);  //使能TIMx，开启时钟

}
//初始化超时定时器
void OTstart(void)
{
	TIM_Cmd(TIM1,DISABLE);
	TIM_SetCounter(TIM1,0x0);
	TIM_Cmd(TIM1,ENABLE);
	
}
void OTdisen(void)
{
	TIM_Cmd(TIM1,DISABLE);
	TIM_SetCounter(TIM1,0x0);
}

extern int updatareq;
void ParameterInit(u8 Version0,u8 Version1,u8 Version2)
{
	delay_ms(20);
	U_target_position=Pos_ADC_Value;
	MotoFlashEEPROM[Goal_PositionH]=MotoFlashEEPROM[Present_PositionH]=(int)Pos_ADC_Value>>8&0xff;
	MotoFlashEEPROM[Goal_PositionL]=MotoFlashEEPROM[Present_PositionL]=(int)Pos_ADC_Value&0xff;
	target_position=U_target_position;
	MotoFlashEEPROM[Torque_Enable]=0;
	updatareq=MotoFlashEEPROM[Firmware_Version];
	
	MotoFlashEEPROM[P_Gain]=6;
	MotoFlashEEPROM[I_Gain]=0;
	MotoFlashEEPROM[D_Gain]=0;
	MotoFlashEEPROM[LED]=0;
	
	MotoFlashEEPROM[60] = Version0;
	MotoFlashEEPROM[61] = Version1;
	MotoFlashEEPROM[62] = Version2;
}


int GetMedianNum(int * bArray, int iFilterLen) 
{ 
	int i,j;// 循环变量 
	int bTemp; 
	// 用冒泡法对数组进行排序 
	for (j = 0; j < iFilterLen - 1; j ++) 
	{ 
		for (i = 0; i < iFilterLen - j - 1; i ++) 
		{ 
			if (bArray[i] > bArray[i + 1]) 
			{ 
				// 互换 
				bTemp = bArray[i]; 
				bArray[i] = bArray[i + 1]; 
				bArray[i + 1] = bTemp; 
			} 
		} 
	} 
	// 计算中值 
	if ((iFilterLen & 1) > 0) 
	{ 
		// 数组有奇数个元素，返回中间一个元素 
		bTemp = bArray[(iFilterLen + 1) / 2]; 
	} 
	else 
	{ 
		// 数组有偶数个元素，返回中间两个元素平均值 
		bTemp = (bArray[iFilterLen / 2] + bArray[iFilterLen / 2 + 1]) / 2; 
	} 
	return bTemp; 
}



u16 OverTorqueCount=0;     //过流时间计算
u16 OverTorqueTime=3000;   //过流触发时间（ms）

//舵机异常状态监听
u8 Statuslistening()
{
  if(MotoFlashEEPROM[Present_Temperature]>MotoFlashEEPROM[Temperature_Limit])
    return 0;
  if((MotoFlashEEPROM[Present_Voltage]<MotoFlashEEPROM[Min_Voltage_Limit])||(MotoFlashEEPROM[Present_Voltage]>MotoFlashEEPROM[Max_Voltage_Limit]))
    return 0;
  if((MotoFlashEEPROM[Present_LoadH]<<8|MotoFlashEEPROM[Present_LoadL])>((MotoFlashEEPROM[Max_TorqueH]<<8|MotoFlashEEPROM[Max_TorqueL]))*10)
  {
    if(OverTorqueCount<OverTorqueTime) 
      OverTorqueCount+=60;
    else  
    {
      OverTorqueCount=0;
      return 0;
    }
  }
  else 
    OverTorqueCount=0;
  return 1;
}

u16 count_TIM2=0;
//定时器2中断服务程序  //
void TIM2_IRQHandler(void)   //TIM2中断  
{
	u8 h=0;
  float ntc_v;
	float ntc_res;
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)  //检查TIM2更新中断发生与否
		{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIMx更新中断标志 
		TIM2->CNT = 0;	
		count_TIM2++;
    if(count_TIM2%3==0)        //60ms
    {
      count_TIM2=0;
      ADC_Collect_temp[2]=0;//POWER
			ADC_Collect_temp[4]=0;//Temperature
			for(h=0;h<10;h++)
			{
				ADC_Collect_temp[2] += ADC_Collect[h][2];
				ADC_Collect_temp[4] += ADC_Collect[h][4];
			}
			POWER_ADC_Value=ADC_Collect_temp[2]/10;
			Temperature_ADC_Value = ADC_Collect_temp[4]/10;
			battery_v = POWER_ADC_Value/Refer_ADC_Value*1.2*vol_multiple ;
			ntc_v =Temperature_ADC_Value/4096*3.3;
			ntc_res=ntc_v/(3.3-ntc_v);
		  temperature_c=b_constant/((b_constant/(25+273.15))-log(10/ntc_res))-273.15;
      
			MotoFlashEEPROM[Present_Voltage] = battery_v*10;
			MotoFlashEEPROM[Present_Temperature] = temperature_c;
      if(!Statuslistening())
      {
        MotoFlashEEPROM[Torque_Enable]=0;
        U_torque_status = 0;
      }
    }      
		
		if(MotoFlashEEPROM[Torque_Enable]==0)//更新解锁状态
			U_torque_status = 0;
		
		kp=MotoFlashEEPROM[P_Gain];
		ki=MotoFlashEEPROM[I_Gain];
		kd=MotoFlashEEPROM[D_Gain];
		if(MotoFlashEEPROM[LED]==1)
			ledON;
		else ledOFF;
    
    
    
		}
}



u16 DiffAdc_Zone=1;
float kp=6,kpLittle=6;
float kd=0,kdLittle=150;
float ki=0;

u16 maxPWM=3500;
u16 minPWM=10;
u16 kpDeadZone = 310;
u16 Realtime=0;

u8  k_num=0;
int kp_error[256]={0};
int kd_error_errorold[256]={0};
int result_calculate_PWM[256]={0};

u16 ADC_Collect_temp[5];//CH17基准 DWQ POWER Current Temperature



//定时器4中断服务程序  
void TIM4_IRQHandler(void)   //TIM4中断
{
	u8 i;
	u16 DiffAdc;
	u16 pos_adc_flash=0;//中间变量 当前adc值
	float cur_adc_out=0;
	static u8 SETTarget = 0;
	int ADC_Pos_temp[10];
	
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
		{
			TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIMx更新中断标志 

			/*ADC buffer*/
			ADC_Collect_temp[0]=0;
			ADC_Collect_temp[1]=0;
			ADC_Collect_temp[3]=0;
			for(i=0;i<10;i++)
			{
				ADC_Collect_temp[0] += ADC_Collect[i][0];//Refer
				ADC_Collect_temp[1] += ADC_Collect[i][1];//pos
				ADC_Pos_temp[i] = ADC_Collect[i][1];//pos
				ADC_Collect_temp[3] += ADC_Collect[i][3];//current
			}
			Refer_ADC_Value=ADC_Collect_temp[0]/10;
//			Pos_ADC_Value=pos_adc_flash=ADC_Collect_temp[1]/10;
			Pos_ADC_Value=pos_adc_flash=GetMedianNum(ADC_Pos_temp,10);//使用中值滤波
			cur_adc_out=(float)ADC_Collect_temp[3]/10/Refer_ADC_Value*1.2;
			current_ma=(float)cur_adc_out/SAMPLING_RES/current_multiple*1000;   //当前电流(MA)

			
			
			
			/*target_position LOCK CHANGE*/
			U_target_position = MotoFlashEEPROM[Goal_PositionH]<<8|MotoFlashEEPROM[Goal_PositionL];//对表内目标位置进行读取
			
			if(U_target_position != target_position)	
				MotoFlashEEPROM[Torque_Enable] = 1;
			
			if((U_torque_status == 0) && (MotoFlashEEPROM[Torque_Enable] == 1))//解锁变加锁 更新加锁状态
			{
				if(U_target_position == target_position)
				{
				U_target_position = Pos_ADC_Value;
				MotoFlashEEPROM[Goal_PositionH]=U_target_position>>8&0xff;
				MotoFlashEEPROM[Goal_PositionL]=U_target_position&0xff;
				}
				
				U_torque_status = 1;
			}
			
			
//fftest
//			if(SETTarget)
//			{
//				SETTarget=0;
//				U_target_position=1000;
//			}
//			else
//			{
//				SETTarget=1;
//				U_target_position=2000;
//			}//fftest
			if(T_target_position) U_target_position = T_target_position ;//fftest
			
			
			
			if(U_target_position<min_limit)
			{
				U_target_position=min_limit;
				MotoFlashEEPROM[Goal_PositionH]=U_target_position>>8&0xff;
				MotoFlashEEPROM[Goal_PositionL]=U_target_position&0xff;
			}
			if(U_target_position>max_limit)
			{
				U_target_position=max_limit;
				MotoFlashEEPROM[Goal_PositionH]=U_target_position>>8&0xff;
				MotoFlashEEPROM[Goal_PositionL]=U_target_position&0xff;
			}
				
			
			
			target_position=U_target_position;

			
			/*calculator*/
			kpLittle=kp;
			kdLittle=kd;
			//calculator
			ErrorOld=Error;
			Error=Pos_ADC_Value-target_position;										//计算偏差
			Integral_error+=Error;														//求出偏差的积分
			if(Integral_error>500)  	Integral_error=500;          //===积分限幅
			if(Integral_error<-500)		Integral_error=-500;         //===积分限幅
			Differential=Error-ErrorOld;
//			calculate_PWM = kp*(Error)+ki*Integral_error+kd*Differential;
			if((Error<kpDeadZone)&&(Error>-kpDeadZone))
			{
			calculate_PWM = kpLittle*(Error)+ki*Integral_error+kdLittle*Differential;
			}
			else
			{
			calculate_PWM = kp*(Error)+ki*Integral_error+kd*Differential;
			}
			
			Compare_PWM_OUT = calculate_PWM;
			
			
//			k_num++;
//			result_calculate_PWM[k_num] = calculate_PWM;
//			kp_error[k_num] = kp*(Error);
//			kd_error_errorold[k_num] = kd*(Error-ErrorOld);
			
			
		  //Lock
		  MotoFlashEEPROM[Present_PositionH]=pos_adc_flash >>8&0xff;
		  MotoFlashEEPROM[Present_PositionL]=pos_adc_flash &0xff;
      
		  //Present Load
		  MotoFlashEEPROM[Present_LoadH]=(u16)current_ma >>8&0xff;
		  MotoFlashEEPROM[Present_LoadL]=(u16)current_ma &0xff;
		  
		  //Realtime Tick
		  Realtime++;
		  Realtime=(Realtime>=32767)? 0:Realtime;
		  MotoFlashEEPROM[Realtime_TickH]=(u16)Realtime >>8&0xff;
		  MotoFlashEEPROM[Realtime_TickL]=(u16)Realtime &0xff;
      
			if(U_torque_status == 0)
			{
				Compare_PWM_OUT=0;
				//goto PWM_OUT;
			}
			
			//目标死区
			DiffAdc = (Pos_ADC_Value>target_position)?Pos_ADC_Value-target_position:target_position-Pos_ADC_Value;
			if(DiffAdc<DiffAdc_Zone)
			{
				Compare_PWM_OUT=0;
				//goto PWM_OUT;
			}
				
			
			//极限限位 *****
			if(Pos_ADC_Value>max_limit)//4000
			{
				Compare_PWM_OUT=0;
				//goto PWM_OUT;
			}
			if(Pos_ADC_Value<min_limit)//200
			{
				Compare_PWM_OUT=0;
				//goto PWM_OUT;
			}
			
			
			
			PWM_OUT:
			//PWM输出
			if((Compare_PWM_OUT<minPWM) && (Compare_PWM_OUT>-minPWM))
				Compare_PWM_OUT = 0;
			
			if(Compare_PWM_OUT<0)
			{
				if(Compare_PWM_OUT<-maxPWM)
						Compare_PWM_OUT=-maxPWM;
				PWM1Run(-Compare_PWM_OUT);
			}
			else if(Compare_PWM_OUT>0)
			{
				if(Compare_PWM_OUT>maxPWM)
						Compare_PWM_OUT=maxPWM;
				PWM2Run(Compare_PWM_OUT);
			}
			else if(Compare_PWM_OUT==0)
			{
				ClosePWM();
				SD_disable();
			}	
			
		}
}
