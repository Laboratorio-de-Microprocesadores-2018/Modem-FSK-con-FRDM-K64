#ifndef FLOATBUFFER_H_
#define FLOATBUFFER_H_

// Head points always to the oldest value. Buffer is never full, oldest value is overriden.
// 			 __________________________________________________________
//			|  tail   |         |          |            |	  head    |
//			| 	n     |	  n-1   |   n-2    | 	n-3     |   n-(size-1)|
//			 ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯
// In memory:
// 			 __________________________________________________________
//			|         |         |   tail-> |    head->  |	          |
//			| 	n-2   |	  n-1   |     n    | 	n-4     |     n-3     |
//			 ¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯

typedef struct{
	float * data;
	uint32_t head;
	uint32_t size;
}FloatBuffer;

/**
*
*/
#define NEW_FLOAT_BUFFER(name,size_) \
	static float name##_data[(size_)]; \
	static FloatBuffer name = {	.data= name##_data, \
								.head=0, 			\
								.size=(size_)          }

/**
*
*/
#define PUSH(name,value) \
	name.data[name.head] = value;			\
	name.head = (name.head + 1) % name.size

/**
*	Return tail + i element
*	Note: GET(name,i) == GET(name,i+size)
*/
#define GET(name,i)       \
	name.data[ (name.size+name.head-(i+1)) % name.size ]

/**
*	Return oldest element
*/
#define GET_HEAD(name) \
	name.data[name.head]

/**
*	Return last element
*/
#define GET_TAIL(name)\
	name.data[(name.size+name.head-1)%name.size]


#endif /* FLOATBUFFER_H_ */
