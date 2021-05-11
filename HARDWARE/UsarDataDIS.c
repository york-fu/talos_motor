/*************************************************************************
    > File Name: UsarDataDIS.c
    > Author: zll
    > Mail: zhnllion@126.com 
    > Created Time: 2018年05月04日 星期五 10时21分17秒
 ************************************************************************/

#include "UsarDataDIS.h"
#include "usart.h"
#include "TIM.h"
#include "DataFlash.h"
#include "string.h"
#include "stmflash.h"
#include "communication.h"

//程序没有对EEPROM Area和RAM Area作区分，而是共用一个表
//MotoFlashREG作为暂存表，MotoFlashEEPROM作为FLASH映射表
u8 MotoFlashEEPROM[0XFF] = {0};
u8 MotoFlashREG[0xff] = {0};
u8 MotoDataBuff[0xff] = {0}; //通信有效数据缓存
u8 RegWIRflag = 0;

u8 MotoDisCtp = 0;
extern u8 MotoRxBuff[0xff];
extern u8 MotoRXCtp;

extern void dxl_portInit(uint8_t value);

void MotoFlashEEPROMinit(void)
{

  MotoFlashEEPROM[0] = 0x1d; //0x1d moto   0x7b sensor
  MotoFlashEEPROM[1] = 0x00;
  MotoFlashEEPROM[2] = 0x26; //Firmware_Version
  MotoFlashEEPROM[3] = 0x00; //ID

  MotoFlashEEPROM[4] = 0x00;
  MotoFlashEEPROM[5] = 0x00;
  MotoFlashEEPROM[6] = 0x00;

  MotoFlashEEPROM[7] = 0x00;
  MotoFlashEEPROM[8] = 0xff;
  MotoFlashEEPROM[9] = 0x0f;
  MotoFlashEEPROM[10] = 0x00;

  MotoFlashEEPROM[11] = 0x50;
  MotoFlashEEPROM[12] = 0x64;
  MotoFlashEEPROM[13] = 0xa0;
  MotoFlashEEPROM[14] = 0x20;

  MotoFlashEEPROM[15] = 0x03;
  MotoFlashEEPROM[16] = 0x01;
  MotoFlashEEPROM[17] = 0x24;
  MotoFlashEEPROM[18] = 0x24;

  MotoFlashEEPROM[19] = 0x00;
  MotoFlashEEPROM[20] = 0x00;
  MotoFlashEEPROM[21] = 0x00;
  MotoFlashEEPROM[22] = 0x01;

  MotoFlashEEPROM[23] = 0x00;
  MotoFlashEEPROM[24] = 0x00;
  MotoFlashEEPROM[25] = 0x00;
  MotoFlashEEPROM[26] = 0x00;

  MotoFlashEEPROM[27] = 0x00;
  MotoFlashEEPROM[28] = 0x20;
  MotoFlashEEPROM[29] = 0x00;
  MotoFlashEEPROM[30] = 0xD0; //2000

  MotoFlashEEPROM[31] = 0x07;
  MotoFlashEEPROM[32] = 0x00;
  MotoFlashEEPROM[33] = 0x00;
  MotoFlashEEPROM[34] = 0xff;

  MotoFlashEEPROM[35] = 0x03;
  MotoFlashEEPROM[36] = 0xe3;
  MotoFlashEEPROM[37] = 0x0f;
  MotoFlashEEPROM[38] = 0x00;

  MotoFlashEEPROM[39] = 0x00;
  MotoFlashEEPROM[40] = 0x00;
  MotoFlashEEPROM[41] = 0x00;
  MotoFlashEEPROM[42] = 0x4f;

  MotoFlashEEPROM[43] = 0x20;
  MotoFlashEEPROM[44] = 0x00;
  MotoFlashEEPROM[45] = 0x00;
  MotoFlashEEPROM[46] = 0x00;

  MotoFlashEEPROM[47] = 0x00;
  MotoFlashEEPROM[48] = 0x00;
  MotoFlashEEPROM[49] = 0x00;
  MotoFlashEEPROM[50] = 0x00;

  MotoFlashEEPROM[51] = 0x00;
  MotoFlashEEPROM[52] = 0xe2;
  MotoFlashEEPROM[53] = 0x0f;
  MotoFlashEEPROM[54] = 0xff;

  MotoFlashEEPROM[55] = 0x03;
  MotoFlashEEPROM[56] = 0x00;
  MotoFlashEEPROM[57] = 0x00;
  MotoFlashEEPROM[58] = 0x00;

  MotoFlashEEPROM[59] = 0x00;
  MotoFlashEEPROM[60] = 0x00;
  MotoFlashEEPROM[61] = 0x00;
  MotoFlashEEPROM[62] = 0x00;
  MotoFlashEEPROM[63] = 0x00;
  MotoFlashEEPROM[64] = 0x00;
  MotoFlashEEPROM[65] = 0x00;
  MotoFlashEEPROM[66] = 0x00;
  MotoFlashEEPROM[67] = 0x00;
  MotoFlashEEPROM[68] = 0x00;
  MotoFlashEEPROM[69] = 0x00;
  MotoFlashEEPROM[70] = 0x00;
  MotoFlashEEPROM[71] = 0x00;
  MotoFlashEEPROM[72] = 0x00;
  MotoFlashEEPROM[73] = 0x00;
}

/*
 * 函数名：ping操作
 * */
void Ping()
{
  u8 PingSendBuff[] = {0xff, 0xff, 0xff, 0x02, 0x00, 0x00};
  PingSendBuff[2] = MotoId;
  PingSendBuff[5] = MotoId + 0x02 + 0x00;
  PingSendBuff[5] = ~PingSendBuff[5];
  usart_send(PingSendBuff, 6);
}

/*
 *函数名：写操作
 * */
extern int updatareq;
void Write(u8 WriteFlashAddrs, u8 *WriteBuff, u8 WriteFlashLEN)
{
  u8 ctp;
  uint8_t buffer[100] = {0xff, 0xff, 0x00, 0x00, 0x00, 0x00};
  uint8_t i, SUM = 0, SendBuffCount = 2;

  for (ctp = 0; ctp < WriteFlashLEN - 1; ctp++)
  {
    MotoFlashEEPROM[WriteFlashAddrs + ctp] = WriteBuff[ctp];
  }
  if (WriteFlashAddrs < 23) //EEPROM Area
  {
    STMFLASH_Write(MotoFlashHead, (u16 *)MotoFlashEEPROM, 11);
  }
  
  buffer[SendBuffCount++] = MotoId; //buffer[2]
  buffer[SendBuffCount++] = 2;
  buffer[SendBuffCount++] = 0x00; //ERR
  for (i = 2; i < SendBuffCount; i++)
    SUM += buffer[i];
  buffer[SendBuffCount++] = ~SUM;
  usart_send(buffer, SendBuffCount);
  
  memcpy(MotoFlashREG, MotoFlashEEPROM, 0xff);
  MotoDisCtp = MotoRXCtp;
  updatareq = MotoFlashEEPROM[Firmware_Version];
}
/*
 *函数名：读操作
 * */
void Read(u8 ReadFlashAddrs, u8 ReadFlashLen)
{
  u8 ReadSendBuff[100] = {0xff, 0xff, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00};
  u8 i, SUM = 0, SendBuffCount = 2;

  //	if(ReadFlashAddrs<0x20)
  //	{
  //	STMFLASH_Read(MotoFlashHead,(u16 *)MotoFlashEEPROM,15);
  //	}
  ReadSendBuff[SendBuffCount++] = MotoId; //ReadSendBuff[2]
  ReadSendBuff[SendBuffCount++] = ReadFlashLen + 2;
  ReadSendBuff[SendBuffCount++] = 0x00; //ERR
  for (i = 0; i < ReadFlashLen; i++)
    ReadSendBuff[SendBuffCount++] = MotoFlashEEPROM[ReadFlashAddrs + i];
  for (i = 2; i < SendBuffCount; i++)
    SUM += ReadSendBuff[i];
  ReadSendBuff[SendBuffCount++] = ~SUM;
  usart_send(ReadSendBuff, SendBuffCount);
  MotoDisCtp = MotoRXCtp;
}

/*
 *函数名：异步写
 * */
void REGwri(u8 RegWriFlashAddrs, u8 RegWrilenFLASH)
{
  u8 i = 0;
  u8 PingSendBuff[] = {0xff, 0xff, 0X00, 0x02, 0x00, 0x00};

  PingSendBuff[2] = MotoId;

  PingSendBuff[5] = ~(PingSendBuff[2] + PingSendBuff[3] + PingSendBuff[4]);

  for (i = 0; i < RegWrilenFLASH; i++)
  {
    MotoFlashREG[RegWriFlashAddrs + i] = MotoDataBuff[i + 1];
  }

  RegWIRflag = 1;
  MotoFlashREG[Registered] = 1;

  usart_send(PingSendBuff, 6);
}
/*
 *函数名：执行异步写
 * */
void action()
{
  RegWIRflag = 0;
  MotoFlashREG[Registered] = 0;
  memcpy(MotoFlashEEPROM, MotoFlashREG, 0xff);
  MotoDisCtp = MotoRXCtp;
}
/*
 *函数名：多个ID控制
  只有当要写入的控制表的地址和长度相同时，才能使用SYNC WRITE命令。另外，ID应该以广播ID的形式发送。
  Length (L+1) X N + 4   (L: Data Length per RX-64, N: the number of RX-64s)
  注意数据包要结合缓冲区长度，不能过长
 * */
void SyncWrite(u8 datalen)
{
  u8 statraddr, writelen, i, j, id_num, startid;

  statraddr = MotoDataBuff[0];
  writelen = MotoDataBuff[1];
  startid = MotoDataBuff[2];
  id_num = (datalen - 2) / (writelen + 1);

  for (i = 0; i < id_num; i++)
  {
    if (MotoDataBuff[2 + i * (writelen + 1)] == MotoId)
    {
      for (j = 0; j < writelen; j++)
      {
        MotoFlashEEPROM[statraddr + j] = MotoDataBuff[2 + 1 + j + i * (writelen + 1)];
      }
      break;
    }
  }
}

/*
*函数名：读多个舵机数据
ID 0XFE
Length 3N+3
Instruction 0X92
Parameter1  0X00
Parameter2  读取数据长度
Parameter3  ID of the first module
Parameter4  Starting address of the data to be read from the first module
 * */
void BulkRead(u8 datalen)
{

  u8 BulkReadSendBuff[48] = {0};
  u8 statraddr, readlen, i, id_num, startid;
  u8 checksum = 0;
  u8 forwardCount = 0;
  u8 returnCount = 0;
  u8 aa = 0, bb = 0;

  //	readlen=MotoDataBuff[1];
  //	startid=MotoDataBuff[2];
  //	statraddr=MotoDataBuff[3];
  id_num = (datalen - 1) / 3;

  for (i = 0; i < id_num; i++)
  {
    readlen = MotoDataBuff[1 + i * 3];
    startid = MotoDataBuff[2 + i * 3];
    statraddr = MotoDataBuff[3 + i * 3];
    if (startid == MotoId)
    {
      OTstart();
      returnCount = 0;
      while (1)
      {
        if (TIMLongOut)
        {
          cbInit(&rxbuf, rx_buf_len, rxbuf_st);
          OTdisen();
          return;
        }
        if (cbCount(&rxbuf) > 1)
        {
          aa = bb;
          cbRead(&rxbuf, &bb);
          returnCount += 1;
          if ((aa == 0xff) && (bb == 0xff))
            OTstart();
        }
        if ((cbCount(&rxbuf) + returnCount) >= (forwardCount + 6 * i))
          break;
      }

      BulkReadSendBuff[0] = 0xFF;
      BulkReadSendBuff[1] = 0xFF;
      BulkReadSendBuff[2] = MotoId;
      BulkReadSendBuff[3] = readlen + 2;
      BulkReadSendBuff[4] = 0x00;

      for (i = 0; i < readlen; i++)
      {
        BulkReadSendBuff[5 + i] = MotoFlashEEPROM[statraddr + i];
      }

      for (i = 2; i < readlen + 5; i++)
      {
        checksum += BulkReadSendBuff[i];
      }

      BulkReadSendBuff[5 + readlen] = ~checksum;

      usart_send(BulkReadSendBuff, 6 + readlen);

      break;
    }
    else
      forwardCount += MotoDataBuff[1 + i * 3];
  }
}

/*
 * 函数名：自算校验和
 * */
u8 DataSumCheck(u8 dataID, u8 datalen, u8 datacmd, u8 *databuffwill)
{
  u8 temp, ctp;
  temp = dataID + datalen + 2 + datacmd;
  for (ctp = 0; ctp < datalen; ctp++)
  {
    temp += databuffwill[ctp];
  }
  temp = ~temp;
  return temp;
}

/*
MX-28
http://emanual.robotis.com/docs/en/dxl/protocol1/#protocol
*/

void DataDis(void)
{
  u8 MotoIDtest;
  u8 MotoLenTest;
  u8 MotoLenCtpTest;
  u8 MotoDataTest;
  u8 MotoDataCmdTest;
  u8 MotoDataSumCh;
  if (cbCount(&rxbuf) > 4)
  {
    cbRead(&rxbuf, &MotoDataTest);
    if (MotoDataTest != 0xff)
    {
      return;
    }
    cbRead(&rxbuf, &MotoDataTest);
    if (MotoDataTest != 0xff)
    {
      return;
    }

  The_tree_ff:

    while (cbCount(&rxbuf) < 3)
    {
      if (TIMLongOut)
      {
        cbInit(&rxbuf, rx_buf_len, rxbuf_st);
        OTdisen();
        return;
      }
    }

    cbRead(&rxbuf, &MotoIDtest);
    if (MotoIDtest == 0xff)
      goto The_tree_ff;

    cbRead(&rxbuf, &MotoLenTest);     //Length LEN
    MotoLenTest -= 2;                 //LEN-1(INST)-1(CKSM)->参数个数
    cbRead(&rxbuf, &MotoDataCmdTest); //CMD INST
    OTstart();
    while (cbCount(&rxbuf) < MotoLenTest + 1)
    {
      if (TIMLongOut)
      {
        cbInit(&rxbuf, rx_buf_len, rxbuf_st);
        OTdisen();
        return;
      }
    }
    if ((MotoIDtest != MotoId) && (MotoIDtest != 0xfe))
    {
      for (MotoLenCtpTest = 0; MotoLenCtpTest < MotoLenTest; ++MotoLenCtpTest) //Parameter
      {
        cbRead(&rxbuf, &MotoDataTest);
        // discard
      }
      return;
    }

    for (MotoLenCtpTest = 0; MotoLenCtpTest < MotoLenTest; ++MotoLenCtpTest) //Parameter
    {
      cbRead(&rxbuf, &MotoDataTest);
      MotoDataBuff[MotoLenCtpTest] = MotoDataTest; //数据内容存储到数组MotoDataBuff[]   0,1,2,3...n  -> P1,P2,P3...Pn
    }
    cbRead(&rxbuf, &MotoDataSumCh);
    if (MotoDataSumCh == DataSumCheck(MotoIDtest, MotoLenTest, MotoDataCmdTest, MotoDataBuff)) //Checksum
    {
      switch (MotoDataCmdTest)
      {
      case PingCmd:
        Ping();
        break;
      case WriteCmd:
        Write(MotoDataBuff[0], MotoDataBuff + 1, MotoLenTest);
        break;
      case ReadCmd:
        Read(MotoDataBuff[0], MotoDataBuff[1]);
        break;
      case RegWRICmd:
        REGwri(MotoDataBuff[0], MotoLenTest - 1);
        break;
      case Action:
        action();
        break;
      case SyncWRCMD:
        SyncWrite(MotoLenTest);
        break;
      case BulkReadCMD:
        BulkRead(MotoLenTest);
        break;
      }
    }
  }
}
