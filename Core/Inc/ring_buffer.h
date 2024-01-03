#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include <stdio.h>
#include <stdlib.h>

// リングバッファの構造体
typedef struct
{
  float * buffer;  // float型のデータを格納する配列
  int size;        // バッファのサイズ
  int front;       // データの先頭位置
  int rear;        // データの末尾位置
  int count;       // データの数
} RingBuffer;

RingBuffer * initRingBuffer(int size);
void enqueue(RingBuffer * rb, float data);
float dequeue(RingBuffer * rb);
void freeRingBuffer(RingBuffer * rb);
float sumNewestN(RingBuffer * rb, int n);
#endif