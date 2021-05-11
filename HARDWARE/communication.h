#ifndef __COMMUNICCATION_H
#define	__COMMUNICCATION_H

#include "stm32f10x.h"
#include "delay.h"

#define rx_buf_len 512	
#define tx_buf_len 512
#define ElemType u8


typedef struct {
    int         size;   /* maximum number of elements           */
    int         start;  /* index of oldest element              */
    int         end;    /* index at which to write new element  */
	  int         count;    /* index at which to write new element  */
    ElemType   *elems;  /* vector of elements                   */
} CircularBuffer;




extern CircularBuffer rxbuf;

extern u8 SendBuff[tx_buf_len];
extern u8 rxbuf_st[rx_buf_len]; 


void rx_buf_init(void);

//cb func
void cbInit(CircularBuffer *cb, int size,u8* state);
int cbIsFull(CircularBuffer *cb);
int cbIsEmpty(CircularBuffer *cb);
int cbCount(CircularBuffer *cb);
int cbIncr(CircularBuffer *cb, int p);
void cbWrite(CircularBuffer *cb, ElemType *elem);
void cbRead(CircularBuffer *cb, ElemType *elem);

void txrx_buf_init(void);

#endif
