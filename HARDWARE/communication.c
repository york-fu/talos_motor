#include "communication.h"



CircularBuffer rxbuf;//串口接收环形缓冲区
//CircularBuffer txbuf;//串口发送环形缓冲区
//CircularBuffer VOEbuff;//USART1串口接收缓冲区
//CircularBuffer VOE2buff;//USART2串口接收缓冲区

u8 SendBuff[tx_buf_len];//串口DMA发送数据缓冲区
u8 rxbuf_st[rx_buf_len];//串口缓冲区实际位置
//u8 txbuf_st[rx_buf_len];//串口缓冲区实际位置
//u8 VOEbuff_st[rx_buf_len];//语音串口缓冲器接收
//u8 VOE2buff_st[rx_buf_len];//蓝牙串口缓冲器接收


/**
  * @brief   初始化环形缓冲区
  * @param   *cb 环形缓冲区结构体地址 size 环形缓冲区大小 state 缓冲区实际位置头指针
  * @retval  
  */
void cbInit(CircularBuffer *cb, int size,u8* state) {
    cb->size  = size;
    cb->start = 0;
    cb->end   = 0;
    //cb->elems = (ElemType *)calloc(cb->size, sizeof(ElemType));
		cb->elems = state;
		cb->count=0;
}
 
/**
  * @brief   判断缓冲区是否为满
  * @param   缓冲区结构体 *cb
  * @retval  1--缓冲区满 0--缓冲区空
  */
int cbIsFull(CircularBuffer *cb) {
    return cb->end == (cb->start ^ cb->size); /* This inverts the most significant bit of start before comparison */ }

/**
  * @brief	判断缓冲区是否为空   
  * @param 	缓冲区结构体 *cb
  * @retval	如为空返回0 非空返回数据长度 
  */
int cbIsEmpty(CircularBuffer *cb) {
   // return cb->end == cb->start;
		return !(cb->count);
 }
/**
  * @brief	移动缓冲区数据指针位置，若缓冲区不满则增长，若到线性区域终点则返黄鹗钾
  * @param 	缓冲区结构体 *cb 缓冲区索引位置（start or end） 
  * @retval	缓冲区索引正确位置
  */ 
int cbIncr(CircularBuffer *cb, int p) {
    return (p + 1)&(2*cb->size-1); /* start and end pointers incrementation is done modulo 2*size */
}
/**
  * @brief	在缓冲区内写入数据
  * @param 	缓冲区结构体 *cb 待写入数据指针
  * @retval	
  */  
void cbWrite(CircularBuffer *cb, ElemType *elem) {
    cb->elems[cb->end&(cb->size-1)] = *elem;
    if (cbIsFull(cb)) /* full, overwrite moves start pointer */
        cb->start = cbIncr(cb, cb->start);
    cb->end = cbIncr(cb, cb->end);
		cb->count+=1;
}
/**
  * @brief	从缓冲区末尾读出数据   
  * @param 	缓冲区结构体 *cb 待写入缓冲区的指针
  * @retval	 
  */ 
void cbRead(CircularBuffer *cb, ElemType *elem) {
    *elem = cb->elems[cb->start&(cb->size-1)];
    cb->start = cbIncr(cb, cb->start);
		cb->count-=1;
}
/**
  * @brief	返回缓冲区长度   
  * @param 	缓冲区结构体 *cb 
  * @retval	返回缓冲区现有长度值 
  */
int cbCount(CircularBuffer *cb)
{
u16 rxbuf_count = 0;
u16 cb_count = 0;
	
//	cb_count = cb->count;
	
	if(cb->end >= cb->start)
		rxbuf_count = cb->end - cb->start;
	else
		rxbuf_count = rx_buf_len *2 - cb->start + cb->end ;
	return  rxbuf_count;

}






