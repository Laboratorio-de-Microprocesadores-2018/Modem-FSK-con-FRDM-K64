/////////////////////////////////////////////////////////////////////////////////
//                              CONTROL DE ACCESO                              //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @file CircularBuffer.c
 *
 */

#include "CircularBuffer.h"
#include <string.h>


bool push(CircularBuffer *this,void* data)
{
	if(this->count == this->capacity)
		return false;
	else
	{
		memcpy(this->head, data, this->size);
		this->head = this->head + this->size;
	    if(this->head == this->buffer_end)
	    	this->head = this->buffer;
	    this->count++;
	    return true;
	}
}

bool pop(CircularBuffer *this,void * data)
{
	if(this->count == 0)
		return false;
	else
	{
		memcpy(data, this->tail, this->size);
		this->tail = this->tail + this->size;
		if(this->tail == this->buffer_end)
			this->tail = this->buffer;
		this->count--;
		return true;
	}
}

void flush(CircularBuffer * this)
{
	this->head = this->tail;
	this->count = 0;
}

int numel(CircularBuffer *this)
{
	return this->count;
}

bool isEmpty(CircularBuffer *this)
{
	return (this->count == 0);
}

bool isFull(CircularBuffer *this)
{
	return (this->count == this->capacity);
}

