#include "adc.h"
// PA.0 DMA-ADC

__IO uint16_t ADC_Collect[10][5];
__IO uint16_t ADC_ConvertedValue;


float Refer_ADC_Value=0;
float Pos_ADC_Value=2000;
float POWER_ADC_Value=0;
float Current_ADC_Value=0;
float Temperature_ADC_Value=0;

float battery_v=0;//当前电压 （0.1V）
float current_ma=0;//当前电流(MA)
float temperature_c=0;//当前温度（℃）


/*
 * 函数名：ADC_GPIO_Config
 * 描述  ：使能ADC1和DMA1的时钟，初始化PA.00
 * 输入  : 无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable DMA clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* Enable ADC1  and GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);
	
	/* Configure PA.00  as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);				// PA0,输入时不用设置速率
	
	//POWER 12V
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Current
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//Temperature
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}


/* 函数名：ADC1_Mode_Config
 * 描述  ：配置ADC1的工作模式为MDA模式   ADC1 ch0
 * 输入  : 无
 * 输出  ：无
 * 调用  ：内部调用
 */
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	
	/* DMA channel1 ADC1 configuration */
	DMA_DeInit(DMA1_Channel1);															//将通道一寄存器设为默认值
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	 				//ADC地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_Collect;							//内存地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;									//该参数规定了外设是作为数据传输的目的地还是来源，此处是作为来源
	DMA_InitStructure.DMA_BufferSize = 10*5;												//定义指定DMA通道的DMA缓存的大小,单位为数据单位。这里是 ADC_Collect 的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;					//外设地址固定
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;        						//内存地址递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;			//半字
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;					//数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;										//循环传输
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);
	
	/* ADC1 configuration */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;									//独立ADC模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 									//扫描模式，扫描模式用于多通道采集
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;									//开启连续转换模式，即不停地进行ADC转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;					//不使用外部触发转换
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 								//采集数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 5;	 											//要转换的通道数目5
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*配置ADC时钟，为PCLK2的6分频，即12MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); 
		
		
	/*配置ADC1的通道1 为71.5个采样周期，序列为1 */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17,1, ADC_SampleTime_71Cycles5);//基准电压，CH17 1.20V
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 2, ADC_SampleTime_71Cycles5);//dwq
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_71Cycles5);//POWER
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_71Cycles5);//Current
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_71Cycles5);//Temperature

	
	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*复位校准寄存器 */   
	ADC_ResetCalibration(ADC1);
	/*等待校准寄存器复位完成 */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADC校准 */
	ADC_StartCalibration(ADC1);
	/* 等待校准完成*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	ADC_TempSensorVrefintCmd(ENABLE);
	/* 由于没有采用外部触发，所以使用软件触发ADC转换 */ //软件开启ADC转换的方法
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/*
 * 函数名：ADC1_Init
 * 描述  ：无
 * 输入  ：无
 * 输出  ：无
 * 调用  ：外部调用
 */
void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}
