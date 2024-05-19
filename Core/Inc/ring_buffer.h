#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include <stdio.h>
#include <stdlib.h>

#include "management.h"

RingBuffer * initRingBuffer(int size);
void enqueue(RingBuffer * rb, float data);
float dequeue(RingBuffer * rb);
void freeRingBuffer(RingBuffer * rb);
float sumNewestN(RingBuffer * rb, int n);
#endif