#ifndef _SINTABLE_
#define _SINTABLE_
#include <math.h>

#define BIT_FREC 1200
#define PWM_FREC 98400

void createSineTables(uint16_t *arr1,uint16_t *arr2, uint16_t size)//size=PWM_FREC/BIT_FREC
{
	double a=0;
	for(uint16_t i; i<size; i++)
	{
		//DUTY VALUES
		a=(1+sin(2*PI*i/size))/2.0;


		arr1[i]=(uint16_t)(a*);

	}




	for(uint16_t i; i<size; i++)
	{
		a=(1+sin(2*PI*i*2/size))/2.0;
	}




}

#endif//_SINTABLE_
