/*************************************************************************
    > File Name: DataFlash.h
    > Author:WS
    > Mail:
    > Created Time: 2018年05月04日 星期五 14时45分40秒
 ************************************************************************/

#ifndef _DataFlash_H
#define _DataFlash_H
#include "stm32f10x.h"


#define MotoFlashHead  0x0800ec00//舵机flash存储头
#define MotoId  MotoFlashEEPROM[ID]
#define TorqueSwitchFlag  MotoFlashEEPROM[TorqueSwitch]
//内存控制表EEPROM
//enum {
//VersionH      = 0x03,//软件版本H
//VersionL      = 0x04,//软件版本L
//MotoIdctp     = 0x05,//ID
//BaudRate      = 0x06,//波特率
//RtDelayTime   = 0x07,//返回延时
//RTStaLVL      = 0x08,//应答状态等级
//AngleMinH     = 0x09,//最小角度限制H
//AngleMinL     = 0x0A,//最小角度限制L
//AngleMaxH     = 0x0B,//最大角度限制H
//AngleMaxL     = 0x0C,//最大角度限制L
//TemperatureMax= 0x0D,//温度上限
//VoltageMax    = 0x0E,//电压上限
//VoltageMin		= 0x0F,//电压下限
//TorqueMaxH    = 0x10,//最大扭矩H
//TorqueMaxL    = 0x11,//最大扭矩L
//SpeedReg			= 0x12,//速度调整
//Unloading			= 0x13,//卸载条件
//MedianRegH    = 0x14,//中位调整H
//MedianRegL    = 0x15,//中位调整L
//ElectricityH  = 0x16,//电流阀值H
//ElectricityL  = 0x17,//电流阀值L
//PWMoutMinH    = 0x18,//最小PWMH
//PWMoutMinL    = 0x19,//最小PWML
//ADCDenL       = 0x1A,//顺时针不灵敏区
//ADCDenR       = 0x1B,//逆时针不灵敏区
//LEDAlarm      = 0x1C,//LED警报
//TorqueDenAlarm= 0x1D,//撤销扭矩警报
//};

//enum {
//TorqueSwitch     = 0x28,//扭矩开关
//LEDSwitch        = 0x29,//LED开关
//TargetIocationH  = 0x2A,//目标位置H
//TargetIocationL  = 0x2B,//目标位置L
//ElapsedTimeH     = 0x2C,//运行时间H
//ElapsedTimeL     = 0x2D,//运行时间L
//CurrentElectricH = 0x2E,//当前电流H
//CurrentElectricL = 0x2f,//当前电流L
//Shackles         = 0x30,//锁标志
//CurrentPositionH = 0x38,//当前位置H
//CurrentPositionL = 0x39,//当前位置L
//RunningSpeedH    = 0x3A,//运行速度H
//RunningSpeedL    = 0x3B,//运行速度L
//CurrentLoadH		 = 0x3C,//当前负载H
//CurrentLoadL     = 0x3D,//当前负载L
//CurrentVoltage   = 0x3E,//当前电压
//CurrentTemperature=0x3F,//当前温度
//RegWRI           = 0x40,//REGwriFlag
//};

//Control Table of EEPROM Area
enum {
	Model_NumberL				= 0,
	Model_NumberH				= 1,
	Firmware_Version		= 2,
	ID									= 3,
	Baud_Rate						= 4,
	Return_Delay_Time		= 5,
	CW_Angle_LimitL			= 6,
	CW_Angle_LimitH			= 7,
	CCW_Angle_LimitL		= 8,
	CCW_Angle_LimitH		= 9,
	Temperature_Limit		= 11,
	Min_Voltage_Limit		= 12,
	Max_Voltage_Limit		= 13,
	Max_TorqueL					= 14,
	Max_TorqueH					= 15,
	Status_Return_Level	= 16,
	Alarm_LED						= 17,
	Shutdown						= 18,
	Multi_Turn_OffsetL	= 20,
	Multi_Turn_OffsetH  = 21,
	Resolution_Divider	= 22,
};
//Control Table of RAM Area
enum {
	Torque_Enable				= 24,
	LED									= 25,
	D_Gain							= 26,
	I_Gain							= 27,
	P_Gain							= 28,
	Goal_PositionL			= 30,
	Goal_PositionH			= 31,
	Moving_SpeedL				= 32,
	Moving_SpeedH				= 33,
	Torque_LimitL				= 34,
	Torque_LimitH				= 35,
	Present_PositionL		= 36,
	Present_PositionH		= 37,
	Present_SpeedL		  = 38,
	Present_SpeedH 		  = 39,
	Present_LoadL			  = 40,
	Present_LoadH				= 41,
	Present_Voltage			= 42,
	Present_Temperature	= 43,
	Registered					= 44,
	Moving							= 46,
	Lock								= 47,
	PunchL							= 48,
	PunchH							= 49,
	Realtime_TickL			= 50,
	Realtime_TickH			= 51,
	Goal_Acceleration		= 73,
	
	pressure_valueA_L			= 90,
	pressure_valueA_H			= 91,
	pressure_valueB_L			= 92,
	pressure_valueB_H			= 93,
	pressure_valueC_L			= 94,
	pressure_valueC_H			= 95,
	pressure_valueD_L			= 96,
	pressure_valueD_H			= 97,
};


enum {
PingCmd         = 0x01,//Ping
ReadCmd					= 0x02,//读取控制表
WriteCmd        = 0x03,//写入控制表
RegWRICmd				= 0X04,//异步写
Action					= 0x05,//执行异步写
SyncWRCMD				= 0x83,//
BulkReadCMD     = 0X92,
};  

#endif

