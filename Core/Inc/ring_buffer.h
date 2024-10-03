#ifndef _RING_BUFFER_H_
#define _RING_BUFFER_H_

#include <stdio.h>
#include <stdlib.h>

#include "management.h"

ring_buffer_t * initRingBuffer(int size);
void enqueue(ring_buffer_t * rb, float data);
float dequeue(ring_buffer_t * rb);
void freeRingBuffer(ring_buffer_t * rb);
float sumNewestN(ring_buffer_t * rb, int n);
#endif