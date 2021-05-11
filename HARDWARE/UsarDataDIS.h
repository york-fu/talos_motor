/*************************************************************************
    > File Name: UsarDataDIS.h
    > Author: WS
		> Mail:  
    > Created Time: 2018年05月04日 星期五 10时22分58秒
 ************************************************************************/

#ifndef _UsarDataDIS_H
#define _UsarDataDIS_H
#include "stm32f10x.h"




void MotoFlashEEPROMinit(void);

void Read(u8 ReadFlashAddrs,u8 ReadFlashLen);
void DataDis(void);
void SyncWrite(u8 datalen);
void BulkRead(u8 datalen);
u8 DataSumCheck(u8 dataID,u8 datalen,u8 datacmd,u8 *databuffwill);
#endif

