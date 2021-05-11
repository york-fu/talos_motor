#include "communication.h"



CircularBuffer rxbuf;//���ڽ��ջ��λ�����
//CircularBuffer txbuf;//���ڷ��ͻ��λ�����
//CircularBuffer VOEbuff;//USART1���ڽ��ջ�����
//CircularBuffer VOE2buff;//USART2���ڽ��ջ�����

u8 SendBuff[tx_buf_len];//����DMA�������ݻ�����
u8 rxbuf_st[rx_buf_len];//���ڻ�����ʵ��λ��
//u8 txbuf_st[rx_buf_len];//���ڻ�����ʵ��λ��
//u8 VOEbuff_st[rx_buf_len];//�������ڻ���������
//u8 VOE2buff_st[rx_buf_len];//�������ڻ���������


/**
  * @brief   ��ʼ�����λ�����
  * @param   *cb ���λ������ṹ���ַ size ���λ�������С state ������ʵ��λ��ͷָ��
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
  * @brief   �жϻ������Ƿ�Ϊ��
  * @param   �������ṹ�� *cb
  * @retval  1--�������� 0--��������
  */
int cbIsFull(CircularBuffer *cb) {
    return cb->end == (cb->start ^ cb->size); /* This inverts the most significant bit of start before comparison */ }

/**
  * @brief	�жϻ������Ƿ�Ϊ��   
  * @param 	�������ṹ�� *cb
  * @retval	��Ϊ�շ���0 �ǿշ������ݳ��� 
  */
int cbIsEmpty(CircularBuffer *cb) {
   // return cb->end == cb->start;
		return !(cb->count);
 }
/**
  * @brief	�ƶ�����������ָ��λ�ã����������������������������������յ��򷵻��ʼ�
  * @param 	�������ṹ�� *cb ����������λ�ã�start or end�� 
  * @retval	������������ȷλ��
  */ 
int cbIncr(CircularBuffer *cb, int p) {
    return (p + 1)&(2*cb->size-1); /* start and end pointers incrementation is done modulo 2*size */
}
/**
  * @brief	�ڻ�������д������
  * @param 	�������ṹ�� *cb ��д������ָ��
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
  * @brief	�ӻ�����ĩβ��������   
  * @param 	�������ṹ�� *cb ��д�뻺������ָ��
  * @retval	 
  */ 
void cbRead(CircularBuffer *cb, ElemType *elem) {
    *elem = cb->elems[cb->start&(cb->size-1)];
    cb->start = cbIncr(cb, cb->start);
		cb->count-=1;
}
/**
  * @brief	���ػ���������   
  * @param 	�������ṹ�� *cb 
  * @retval	���ػ��������г���ֵ 
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






