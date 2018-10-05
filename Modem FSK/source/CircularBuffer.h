/////////////////////////////////////////////////////////////////////////////////
//                              CONTROL DE ACCESO                              //
//          Grupo 3 - Laboratorio de Microprocesadores - ITBA - 2018           //
//	                                                                           //
/////////////////////////////////////////////////////////////////////////////////


/**
 *	@file     CircularBuffer.h
 *	@brief    Generic circular buffer implementation
 *	@author   Tobias Lifschitz
 *
 *	Generic circular buffer implementation using void pointers and static memory
 *	allocation. The buffer is allocated with the help of a macro defined below,
 *	which creates a static array with the necessary number of bytes and a struct
 *	to handle the buffer.
 */

#ifndef __CIRCULARBUFFER_H_
#define __CIRCULARBUFFER_H_

/////////////////////////////////////////////////////////////////////////////////
//                             Included header files                           //
/////////////////////////////////////////////////////////////////////////////////
#include <stdbool.h>

/////////////////////////////////////////////////////////////////////////////////
//                    Enumerations, structures and typedefs                    //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Struct containing circular buffer internal variables.
 */
typedef struct 
{
	char * const buffer;     /** Pointer to statically reserved memory array. */
	char * const buffer_end; /** Pointer to end of the array. */
	char * head;	         /** Pointer to the head of the buffer. */
	char * tail;	         /** Pointer to the tail of the buffer. */
	int capacity;            /** Maximum number of elements in the buffer. */
	int count;               /** Number of elements in the buffer. */
	int size;                /** Size of each element in the buffer. */
} CircularBuffer;

/////////////////////////////////////////////////////////////////////////////////
//                       Constants and macro definitions                       //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Macro to construct a circular buffer.
 *
 * It allocates static memory and creates a structure with the given name. This
 * name is meant to be passed as the "this" pointer to the buffer functions. *
 */
#define NEW_CIRCULAR_BUFFER(name,_capacity,_size)      	\
        static char name##_arr[_capacity*_size];       	\
        static CircularBuffer name =             		\
        {                                				\
            .buffer = name##_arr,     				    \
			.buffer_end = name##_arr + _capacity*_size,	\
            .head = name##_arr,      				    \
            .tail = name##_arr,       				    \
            .capacity = _capacity,        				\
			.count = 0,                  				\
			.size = _size                 				\
        }

/////////////////////////////////////////////////////////////////////////////////
//                         Global function prototypes                          //
/////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Push an element to the buffer.
 * @param this Pointer to the buffer structure.
 * @param data Pointer to the data to be pushed.
 * @return False if the buffer is full, true instead.
 */
bool push(CircularBuffer *this,void * data);

/**
 * @brief Pop an element from the buffer.
 * @param this Pointer to the buffer structure.
 * @param data Pointer to a variable to store the data.
 * @return False if the buffer is empty, true instead.
 */
bool pop(CircularBuffer *this,void * data);


/**
 * @brief Delete all elements from the buffer.
 * @param this Pointer to the buffer structure.
 */
void flush(CircularBuffer *this);

/**
 * @brief Number of elements currently stored in the buffer.
 * @param this Pointer to the buffer structure.
 * @return The number of elements.
 */
int numel(CircularBuffer *this);

/**
 * @brief Check if the buffer is empty
 * @return True if the buffer is empty
 */
bool isEmpty(CircularBuffer *this);

/**
 * @brief Check if the buffer is full
 * @return True if the buffer is full
 */
bool isFull(CircularBuffer *this);

#endif // __CIRCULARBUFFER_H_
